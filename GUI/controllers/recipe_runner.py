"""Discover, inspect, and drive recipes from the GUI.

Recipes are standalone scripts (recipe/<name>/recipe.py) that already accept an
*injected* ``RecipeContext`` with ``session``/``arduino``/``camera_session``. We
reuse the recipe's own ``parse_args([])`` -> ``resolve_params`` -> ``load_positions``
plumbing, then build a context backed by the GUI's already-connected hardware so
no second process fights for the single robot/camera connection.

Supports: run-full (with a loop count), step phase-by-phase, and a speed scale
applied to the recipe's velocity params.
"""

from __future__ import annotations

import importlib
import inspect

import config


class RecipeRunner:
    def __init__(self, logger, executor, robot, machine, camera) -> None:
        self._logger = logger
        self._executor = executor
        self._robot = robot
        self._machine = machine
        self._camera = camera

        self._name: str | None = None
        self._module = None
        self._phases: list = []        # ordered (name, fn) tuples
        self._poses: dict = {}
        self._ctx = None
        self._ctx_speed: float | None = None
        self._phase_index = 0

    # ----- discovery -----

    def list_recipes(self) -> list[str]:
        if not config.RECIPE_DIR.exists():
            return []
        return sorted(
            d.name for d in config.RECIPE_DIR.iterdir()
            if (d / "recipe.py").is_file()
        )

    def _ordered_phases(self, module) -> list:
        # Honour an explicit GUI_PHASES list if a recipe defines one; otherwise
        # collect phase_* callables in source-definition order.
        if hasattr(module, "GUI_PHASES"):
            return [(n, getattr(module, n)) for n in module.GUI_PHASES]
        fns = [
            (name, fn)
            for name, fn in inspect.getmembers(module, inspect.isfunction)
            if name.startswith("phase_") and fn.__module__ == module.__name__
        ]
        fns.sort(key=lambda nf: nf[1].__code__.co_firstlineno)
        return fns

    def select(self, name: str) -> dict:
        if name not in self.list_recipes():
            raise ValueError(f"unknown recipe: {name}")
        module = importlib.import_module(f"recipe.{name}.recipe")
        self._name = name
        self._module = module
        self._phases = self._ordered_phases(module)
        self._ctx = None
        self._ctx_speed = None
        self._phase_index = 0

        # Load key positions for display using the recipe's own loader.
        poses = {}
        try:
            args = module.parse_args([])
            params = module.resolve_params(args)
            key_dir = module._script_path(str(params["KEY_POSITION_DIR"]))
            poses = module.load_positions(key_dir, self._logger)
        except Exception as exc:  # noqa: BLE001 - display is best-effort
            self._logger.warn(f"could not load positions for {name}: {exc}")
        self._poses = poses
        return self.detail()

    def detail(self) -> dict:
        return {
            "name": self._name,
            "phases": [n for n, _ in self._phases],
            "phase_index": self._phase_index,
            "key_positions": sorted(self._poses.keys()),
        }

    # ----- context construction -----

    def _apply_speed(self, params: dict, speed_scale: float) -> dict:
        """Scale velocity-like params by ``speed_scale`` (0.1-1.0 typical)."""
        scaled = dict(params)
        for key, value in params.items():
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                continue
            if "VEL" not in key.upper():
                continue
            new = value * speed_scale
            # Joint velocity *scales* are integer percentages with a floor of 1.
            if "SCALE" in key.upper():
                new = max(1, int(round(new)))
            scaled[key] = new
        return scaled

    def _build_ctx(self, speed_scale: float):
        module = self._module
        args = module.parse_args([])
        params = module.resolve_params(args)
        params = self._apply_speed(params, speed_scale)
        key_dir = module._script_path(str(params["KEY_POSITION_DIR"]))
        poses = module.load_positions(key_dir, self._logger)
        self._poses = poses

        # Reuse the GUI's connected hardware handles.
        self._robot.session.switch_mode("NRT_PRIMITIVE_EXECUTION")
        ctx = module.RecipeContext(
            params=params,
            poses=poses,
            logger=self._logger,
            dry_run=False,
            session=self._robot.session,
            arduino=self._machine._client,
            camera_session=self._camera._session,
        )
        self._ctx = ctx
        self._ctx_speed = speed_scale
        return ctx

    def _ensure_ctx(self, speed_scale: float):
        if self._ctx is None or self._ctx_speed != speed_scale:
            return self._build_ctx(speed_scale)
        return self._ctx

    # ----- run / step -----

    def run_full(self, speed_scale: float = 1.0, loops: int = 1) -> bool:
        if self._module is None:
            raise ValueError("no recipe selected")

        def op():
            ctx = self._ensure_ctx(speed_scale)
            for i in range(max(1, int(loops))):
                if self._executor.stop_event.is_set():
                    self._logger.warn("recipe loop stopped by request")
                    break
                self._logger.info(f"=== recipe '{self._name}' run {i + 1}/{loops} ===")
                self._module.run_full_recipe(ctx, yes=True)
            self._phase_index = 0
        return self._executor.submit(f"recipe.run:{self._name}", op)

    def step(self, phase_name: str | None = None, speed_scale: float = 1.0) -> bool:
        if self._module is None:
            raise ValueError("no recipe selected")
        if not self._phases:
            raise ValueError("recipe exposes no phases")

        if phase_name is None:
            index = self._phase_index
        else:
            names = [n for n, _ in self._phases]
            if phase_name not in names:
                raise ValueError(f"unknown phase: {phase_name}")
            index = names.index(phase_name)
        if index >= len(self._phases):
            index = 0
        name, fn = self._phases[index]

        def op():
            ctx = self._ensure_ctx(speed_scale)
            self._logger.info(f"=== recipe '{self._name}' phase: {name} ===")
            # phase_cut and friends take keyword-only yes/skip_cut.
            sig = inspect.signature(fn)
            kwargs = {}
            if "yes" in sig.parameters:
                kwargs["yes"] = True
            if "skip_cut" in sig.parameters:
                kwargs["skip_cut"] = False
            fn(ctx, **kwargs)
            self._phase_index = index + 1
        return self._executor.submit(f"recipe.step:{self._name}:{name}", op)

    def stop(self) -> dict:
        self._executor.request_stop()
        try:
            self._robot.session.robot.Stop()
        except Exception:  # noqa: BLE001
            pass
        try:
            self._machine.stop()
        except Exception:  # noqa: BLE001
            pass
        return {"stopped": True}
