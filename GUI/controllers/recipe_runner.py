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


# Operator-facing recipe phases. Each maps to one or more internal recipe
# ``phase_*`` functions (run in listed order when stepping). The live highlight
# follows whichever group the currently-executing internal phase belongs to.
DISPLAY_PHASES = [
    ("Initialization",        ["phase_startup_transfer"]),
    ("Module Localization",   ["phase_calibrate_tags"]),
    ("Pick Injectable",       ["phase_pick_from_camera_align_grasp"]),
    ("Load Injectable",       ["phase_load_vise"]),
    ("Ultrasonic Cutting",    ["phase_cut"]),
    ("Component Disassembly", [
        "phase_twist_and_drop_cap",
        "phase_return_injectable_to_plate",
        "phase_remove_spring",
        "phase_remove_yellow_plastic",
        "phase_remove_shell",
        "phase_return_middle",
        "phase_shutdown",
    ]),
]


class RecipeRunner:
    def __init__(self, logger, executor, robot, machine, camera) -> None:
        self._logger = logger
        self._executor = executor
        self._robot = robot
        self._machine = machine
        self._camera = camera

        self._name: str | None = None
        self._module = None
        self._phases: list = []        # raw introspected (name, fn) tuples
        self._groups: list = []        # [{"name", "members", "fns"}] display groups
        self._poses: dict = {}
        self._ctx = None
        self._ctx_speed: float | None = None
        self._phase_index = -1         # display group running now; -1 = no highlight
        self._next_group = 0           # cursor for the "Next Phase" button
        # Follow the recipe's own "=== phase_* ===" log markers so the highlight
        # advances live during run_full_recipe (which we don't drive phase-by-phase).
        self._logger.add_observer(self._on_log)

    def _on_log(self, level: str, msg: str) -> None:
        if not self._groups:
            return
        m = msg.strip()
        if not m.startswith("=== phase_"):
            return
        token = m[4:].split(" ===")[0].split(" (")[0].strip()
        for gi, group in enumerate(self._groups):
            for member in group["members"]:
                # markers may be richer than the fn name, e.g.
                # "phase_remove_shell_and_glass" for phase_remove_shell.
                if token == member or token.startswith(member):
                    self._phase_index = gi
                    self._next_group = gi + 1
                    return

    def _build_groups(self) -> None:
        present = {name for name, _ in self._phases}
        by_name = dict(self._phases)
        self._groups = []
        for display, members in DISPLAY_PHASES:
            fns = [(m, by_name[m]) for m in members if m in present]
            if fns:
                self._groups.append({"name": display, "members": members, "fns": fns})

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
        self._build_groups()
        self._ctx = None
        self._ctx_speed = None
        self._phase_index = -1   # nothing highlighted until a run starts
        self._next_group = 0
        return self.detail()

    def detail(self) -> dict:
        return {
            "name": self._name,
            "phases": [g["name"] for g in self._groups],
            "phase_index": self._phase_index,
        }

    def state(self) -> dict:
        """Lightweight live state for /api/status (drives the phase highlight)."""
        return {
            "name": self._name,
            "phase_index": self._phase_index,
            "phase_count": len(self._groups),
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
                # Phase highlight advances via the log observer (_on_log).
                self._module.run_full_recipe(ctx, yes=True)
            # Finished: clear the highlight per spec (no phase highlighted at rest).
            self._phase_index = -1
            self._next_group = 0
        return self._executor.submit(f"recipe.run:{self._name}", op)

    def _invoke_phase(self, ctx, fn) -> None:
        # phase_cut and friends take keyword-only yes/skip_cut.
        sig = inspect.signature(fn)
        kwargs = {}
        if "yes" in sig.parameters:
            kwargs["yes"] = True
        if "skip_cut" in sig.parameters:
            kwargs["skip_cut"] = False
        fn(ctx, **kwargs)

    def step(self, phase_name: str | None = None, speed_scale: float = 1.0) -> bool:
        if self._module is None:
            raise ValueError("no recipe selected")
        if not self._groups:
            raise ValueError("recipe exposes no phases")

        if phase_name is None:
            gi = self._next_group
        else:
            names = [g["name"] for g in self._groups]
            if phase_name not in names:
                raise ValueError(f"unknown phase: {phase_name}")
            gi = names.index(phase_name)
        if gi >= len(self._groups):
            gi = 0  # wrap back to the start once the last group has run
        group = self._groups[gi]

        def op():
            ctx = self._ensure_ctx(speed_scale)
            self._next_group = gi + 1
            # Each internal phase logs its "=== phase_* ===" marker, which keeps
            # the highlight on this group via _on_log.
            for _name, fn in group["fns"]:
                if self._executor.stop_event.is_set():
                    break
                self._invoke_phase(ctx, fn)
        return self._executor.submit(f"recipe.step:{self._name}:{group['name']}", op)

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
