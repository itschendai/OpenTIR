"""Interactive pose recorder for the injectable pipeline.

Walks the operator through the checklist in ``pose_schema.checklist_entries`` using
``FloatingJoint`` so the arm can be hand-guided. Writes a structured YAML file that
the orchestrator consumes at startup.

Run with ``--help`` for the full CLI.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Callable, Iterable

import pose_schema
from flexiv_helpers import RobotSession, execute_primitive


DEFAULT_ROBOT_SN = "Rizon4-062930"
DEFAULT_OUTPUT = "pipeline_poses.yaml"
DEFAULT_GRIPPER_NAME = "Flexiv-GN01"
FLOATING_JOINTS = [1.0] * 7
FLOATING_DAMPING_LEVEL = [0.0] * 7
FLOATING_RESPONSE_TORQUE = [1.5, 1.5, 1.5, 1.5, 0.5, 0.5, 0.3]

# Trainer-local defaults for the interactive [g]/[v]/[x] device actions. These
# mirror the orchestrator's PARAMS for realism but use a gentler gripper force
# since the operator may close on air or be tuning the geometry by hand.
TRAINER_GRIPPER_CLOSE_WIDTH_M = 0.005
TRAINER_GRIPPER_OPEN_WIDTH_M = 0.04
TRAINER_GRIPPER_VELOCITY_M_S = 0.05
TRAINER_GRIPPER_FORCE_LIMIT_N = 10.0
TRAINER_VISE_TARGET_FORCE_KG = 4.0
TRAINER_VISE_RELEASE_FORCE_KG = 0.2
TRAINER_CUT_Z_MM = 134.0
TRAINER_CUT_X_MM = 111.0
TRAINER_CUT_DEG = 359.0

# Contextual hints printed when reaching specific checkpoints. They remind the
# operator that fixture-state actions ([g]/[v]/[x]) typically belong here.
_HINTS: dict[str, str] = {
    "pickup_grasp": (
        "Now is the moment to actually grip the part. Press [g] to close the "
        "gripper on it, then [Enter] to capture the pose. Skip [g] if just "
        "demoing the motion without a real part."
    ),
    "above_vise": (
        "Above the vise. Realistic sequence here: hand-guide the arm down "
        "into the vise, press [g] to release the part, press [v] to close "
        "the vise, then retreat back to this pose and [Enter] to capture."
    ),
    "safe_intermediate": (
        "Park-during-cut pose. After [Enter] to capture, you can press [x] "
        "to fire CUT_HEIGHT (DANGER: blade fires; hard confirmation required)."
    ),
    "disposal": (
        "Disposal point. Press [g] to open the gripper (release into the bin), "
        "then [Enter] to capture."
    ),
}


def _script_dir() -> str:
    return os.path.dirname(os.path.abspath(__file__))


def _resolve_path(path: str) -> str:
    if os.path.isabs(path):
        return path
    return os.path.abspath(os.path.join(_script_dir(), path))


def gripper_snapshot(gripper) -> tuple[float, float]:
    """Read width and force from the gripper (zeros if no gripper attached)."""
    if gripper is None:
        return 0.0, 0.0
    states = gripper.states()
    width = float(getattr(states, "width", 0.0))
    force = float(getattr(states, "force", 0.0))
    return width, force


class _DeviceActions:
    """Fires gripper + Arduino commands during the walk-through.

    Each method returns a one-line human-readable string the trainer prints.
    A missing gripper or arduino is tolerated — actions return an explanatory
    message rather than raising, so a partial setup still walks the script.
    """

    def __init__(self, gripper=None, arduino=None, logger=None) -> None:
        self._gripper = gripper
        self._arduino = arduino
        self._logger = logger
        self._gripper_open = True  # toggle-state assumption: gripper boots open

    def home_arduino(self) -> str:
        """Send HOME_ALL via the existing serial connection (no reconnect)."""
        if self._arduino is None:
            return "no arduino (re-run with --arduino to enable)"
        try:
            r = self._arduino.home_all()
        except Exception as exc:  # noqa: BLE001
            return f"HOME_ALL failed: {exc}"
        return (
            f"homed: x={r.get('x_mm')}mm z={r.get('z_mm')}mm rot={r.get('rot_deg')}deg"
        )

    def toggle_gripper(self) -> str:
        if self._gripper is None:
            return "no gripper (re-run with --init-gripper to enable)"
        if self._gripper_open:
            width = TRAINER_GRIPPER_CLOSE_WIDTH_M
            verb = "closing"
        else:
            width = TRAINER_GRIPPER_OPEN_WIDTH_M
            verb = "opening"
        try:
            self._gripper.Move(
                width, TRAINER_GRIPPER_VELOCITY_M_S, TRAINER_GRIPPER_FORCE_LIMIT_N
            )
        except Exception as exc:  # noqa: BLE001 - surface verbatim
            return f"gripper {verb} failed: {exc}"
        self._gripper_open = not self._gripper_open
        return f"gripper {verb} to {width*1000:.1f}mm at {TRAINER_GRIPPER_FORCE_LIMIT_N:.0f}N"

    def toggle_vise(self) -> str:
        if self._arduino is None:
            return "no arduino (re-run with --arduino to enable)"
        try:
            status = self._arduino.get_status()
        except Exception as exc:  # noqa: BLE001
            return f"vise: status read failed: {exc}"
        if status.get("vise_state") == "CLOSED":
            try:
                r = self._arduino.open_vise(target_force_kg=TRAINER_VISE_RELEASE_FORCE_KG)
                return f"vise opened: state={r.get('vise_state')} force_kg={r.get('force_kg')}"
            except Exception as exc:  # noqa: BLE001
                return f"vise open failed: {exc}"
        try:
            r = self._arduino.close_vise(target_force_kg=TRAINER_VISE_TARGET_FORCE_KG)
            return f"vise closed: state={r.get('vise_state')} force_kg={r.get('force_kg')}"
        except Exception as exc:  # noqa: BLE001
            return f"vise close failed: {exc}"

    def fire_cut(self, *, input_fn: Callable[[str], str] = input) -> str:
        if self._arduino is None:
            return "no arduino (re-run with --arduino to enable)"
        prompt = (
            f"!! CUT_HEIGHT z={TRAINER_CUT_Z_MM} x={TRAINER_CUT_X_MM} "
            f"deg={TRAINER_CUT_DEG} - BLADE FIRES. Type 'YES' to confirm: "
        )
        resp = input_fn(prompt).strip()
        if resp != "YES":
            return "cut aborted (confirmation not 'YES')"
        try:
            r = self._arduino.cut_height(
                z_mm=TRAINER_CUT_Z_MM, x_mm=TRAINER_CUT_X_MM, deg=TRAINER_CUT_DEG
            )
            return (
                f"cut complete: x={r.get('x_mm')}mm z={r.get('z_mm')}mm "
                f"rot={r.get('rot_deg')}deg"
            )
        except Exception as exc:  # noqa: BLE001
            return f"cut failed: {exc}"

    def print_status(self) -> str:
        lines: list[str] = []
        if self._gripper is not None:
            try:
                states = self._gripper.states()
                w = float(getattr(states, "width", 0.0))
                f = float(getattr(states, "force", 0.0))
                lines.append(f"gripper: width={w*1000:.1f}mm force={f:.2f}N")
            except Exception as exc:  # noqa: BLE001
                lines.append(f"gripper status failed: {exc}")
        if self._arduino is not None:
            try:
                s = self._arduino.get_status()
                lines.append(
                    f"arduino: homed={s.get('homed')} vise={s.get('vise_state')} "
                    f"x={s.get('x_mm')}mm z={s.get('z_mm')}mm "
                    f"rot={s.get('rot_deg')}deg "
                    f"blade={'ON' if s.get('blade_on') else 'OFF'}"
                )
            except Exception as exc:  # noqa: BLE001
                lines.append(f"arduino status failed: {exc}")
        if not lines:
            return "no devices configured (re-run with --init-gripper / --arduino)"
        return "; ".join(lines)


class PoseTrainer:
    """Encapsulates the trainer's non-interactive logic so tests can drive it.

    The interactive shell lives in :func:`run_interactive`; this class is what the
    shell calls to actually capture poses and waypoints.
    """

    def __init__(
        self,
        session: RobotSession,
        state: pose_schema.TrainerState,
        output_path: str,
        gripper=None,
        logger=None,
    ) -> None:
        self._session = session
        self._state = state
        self._output_path = output_path
        self._gripper = gripper
        self._logger = logger
        self._pending_paths: dict[str, list[dict]] = {}

    # ----- capture API -----

    def capture_pose(self, name: str) -> dict:
        q_rad, tcp = self._read_arm()
        width, force = gripper_snapshot(self._gripper)
        entry = pose_schema.build_pose_entry(q_rad, tcp, width, force)
        self._state.record_pose(name, entry)
        self._info(f"captured pose {name!r}")
        self._flush()
        return entry

    def add_path_waypoint(self, path_name: str, waypoint_name: str) -> dict:
        q_rad, tcp = self._read_arm()
        width, force = gripper_snapshot(self._gripper)
        entry = pose_schema.build_waypoint_entry(waypoint_name, q_rad, tcp, width, force)
        self._pending_paths.setdefault(path_name, []).append(entry)
        self._info(f"path {path_name!r}: added waypoint {waypoint_name!r}")
        return entry

    def finalize_path(self, path_name: str) -> dict:
        spec = pose_schema.path_spec_by_name(path_name)
        waypoints = self._pending_paths.pop(path_name, [])
        entry = pose_schema.build_path_entry(spec.from_pose, spec.to_pose, waypoints)
        self._state.record_path(path_name, entry)
        self._info(f"path {path_name!r}: finalized with {len(waypoints)} waypoints")
        self._flush()
        return entry

    def discard_path(self, path_name: str) -> None:
        self._pending_paths.pop(path_name, None)

    # ----- state access -----

    @property
    def state(self) -> pose_schema.TrainerState:
        return self._state

    def has_pending_path(self, path_name: str) -> bool:
        return bool(self._pending_paths.get(path_name))

    # ----- I/O -----

    def _flush(self) -> None:
        pose_schema.write_yaml(self._state.document, self._output_path)

    def _read_arm(self) -> tuple[list[float], list[float]]:
        _, state = self._session.selected_arm_state()
        q_rad = [float(v) for v in getattr(state, "q", [])]
        tcp = [float(v) for v in getattr(state, "tcp_pose", [])]
        if len(q_rad) != 7:
            raise RuntimeError(f"arm state has {len(q_rad)} joint values; expected 7")
        if len(tcp) != 7:
            raise RuntimeError(f"arm state has {len(tcp)} TCP values; expected 7")
        return q_rad, tcp

    def _info(self, msg: str) -> None:
        if self._logger is not None:
            self._logger.info(msg)
        else:
            print(msg, flush=True)


# ---------- interactive shell ----------


def _auto_home_arduino(arduino, logger=None) -> None:
    """Send HOME_ALL only when the firmware reports ``homed=False``.

    The Arduino DTR-resets on every fresh serial open so a freshly-connected
    client always sees ``homed=False``. This helper makes a phase-scoped
    trainer session usable for the [v]/[x] device actions without forcing the
    operator to home from a separate process (which would race with the open
    port). If status is unreadable we log and continue; the operator can still
    press [h] manually mid-session.
    """
    try:
        status = arduino.get_status()
    except Exception as exc:  # noqa: BLE001 - non-fatal probe
        _say(logger, f"Arduino status read failed during auto-home: {exc}", error=True)
        return
    if status.get("homed"):
        _say(logger, "Arduino already homed; auto-home skipped")
        return
    _say(logger, "Arduino not homed; sending HOME_ALL (X / Z / rotary will move)")
    try:
        r = arduino.home_all()
    except Exception as exc:  # noqa: BLE001
        _say(
            logger,
            f"auto-home HOME_ALL failed: {exc} — press [h] to retry mid-session",
            error=True,
        )
        return
    _say(
        logger,
        f"Arduino homed: x={r.get('x_mm')}mm z={r.get('z_mm')}mm "
        f"rot={r.get('rot_deg')}deg",
    )


def _start_floating(session: RobotSession, logger=None) -> None:
    session.switch_mode("NRT_PRIMITIVE_EXECUTION")
    execute_primitive(
        session.robot,
        "FloatingJoint",
        {
            "floatingJoint": FLOATING_JOINTS,
            "dampingLevel": FLOATING_DAMPING_LEVEL,
            "responseTorque": FLOATING_RESPONSE_TORQUE,
            "diEnableFloating": "NONE",
        },
        flexivrdk_module=session.flexivrdk,
    )
    if logger is not None:
        logger.info("FloatingJoint enabled — drag the arm by hand to each target")


def _prompt_action(
    prompt: str,
    *,
    input_fn: Callable[[str], str] = input,
    allow_done: bool = False,
    device_actions: "_DeviceActions | None" = None,
) -> str:
    while True:
        response = input_fn(prompt).strip().lower()
        if response in {"", "c", "capture", "enter"}:
            return "capture"
        if response in {"r", "redo"}:
            return "redo"
        if response in {"s", "skip"}:
            return "skip"
        if response in {"q", "quit", "exit"}:
            return "quit"
        if allow_done and response in {"d", "done"}:
            return "done"
        if device_actions is not None:
            if response == "g":
                print(device_actions.toggle_gripper(), flush=True)
                continue
            if response == "v":
                print(device_actions.toggle_vise(), flush=True)
                continue
            if response == "x":
                print(device_actions.fire_cut(input_fn=input_fn), flush=True)
                continue
            if response == "h":
                print(device_actions.home_arduino(), flush=True)
                continue
            if response == "?":
                print(device_actions.print_status(), flush=True)
                continue
        help_parts = ["[Enter] capture", "r redo", "s skip"]
        if allow_done:
            help_parts.append("d done")
        help_parts.append("q quit")
        if device_actions is not None:
            help_parts += ["g gripper", "v vise", "x CUT(!)", "h home", "? status"]
        print("Options: " + " | ".join(help_parts), flush=True)


def run_interactive(args, logger=None) -> int:
    output_path = _resolve_path(args.output)

    if args.overwrite and os.path.exists(output_path):
        os.unlink(output_path)

    if args.resume and os.path.exists(output_path):
        state = pose_schema.TrainerState.resume_from(output_path)
        _say(logger, f"Resuming from {output_path}: "
                     f"{len(state.captured_poses)} poses, {len(state.captured_paths)} paths already present")
    else:
        state = pose_schema.TrainerState.fresh(args.robot_sn)

    checklist = pose_schema.phase_entries(
        args.phase, include_optional=args.include_optional
    )
    if args.start_at:
        idx = next(
            (i for i, (_, name) in enumerate(checklist) if name == args.start_at), None
        )
        if idx is None:
            print(f"--start-at: {args.start_at!r} is not in the checklist", file=sys.stderr)
            return 2
        checklist = checklist[idx:]

    arduino = None
    if args.arduino:
        try:
            from arduino_client import ArduinoClient, DEFAULT_READY_TIMEOUT_S

            arduino = ArduinoClient(
                port=args.arduino_port,
                logger=logger,
                ready_timeout_s=DEFAULT_READY_TIMEOUT_S,
                done_timeout_s=60.0,  # HOME_ALL can take >30s if axes travel far
            )
            arduino.connect()
            _say(logger, "Arduino connected — [v] toggles vise, [x] fires CUT_HEIGHT")
            if args.arduino_auto_home:
                _auto_home_arduino(arduino, logger=logger)
        except Exception as exc:  # noqa: BLE001
            _say(
                logger,
                f"Arduino setup failed: {exc} — continuing without Arduino actions",
                error=True,
            )
            arduino = None

    try:
        with RobotSession(args.robot_sn, logger=logger) as session:
            gripper = None
            if args.init_gripper:
                gripper = session.setup_gripper(args.gripper_name, init=True)
            _start_floating(session, logger=logger)
            trainer = PoseTrainer(
                session=session,
                state=state,
                output_path=output_path,
                gripper=gripper,
                logger=logger,
            )
            device_actions = _DeviceActions(
                gripper=gripper, arduino=arduino, logger=logger
            )

            for kind, name in checklist:
                if kind == "pose":
                    if state.is_pose_captured(name) and args.resume:
                        _say(logger, f"[skip] pose {name!r} already captured")
                        continue
                    if name in _HINTS:
                        print(f"[hint] {_HINTS[name]}", flush=True)
                    action = "redo"
                    while action == "redo":
                        action = _prompt_action(
                            f"Jog to pose [{name}], then press Enter to capture> ",
                            device_actions=device_actions,
                        )
                        if action == "capture":
                            trainer.capture_pose(name)
                        elif action == "quit":
                            _say(logger, "operator quit — partial YAML written")
                            return 0
                elif kind == "path":
                    if state.is_path_captured(name) and args.resume:
                        _say(logger, f"[skip] path {name!r} already captured")
                        continue
                    spec = pose_schema.path_spec_by_name(name)
                    print(
                        f"Path [{name}] from {spec.from_pose!r} to {spec.to_pose!r} — "
                        f"capture intermediate waypoints. [Enter] add | d done | s skip path | q quit",
                        flush=True,
                    )
                    wp_index = 1
                    while True:
                        action = _prompt_action(
                            f"Path {name} waypoint {wp_index}> ",
                            allow_done=True,
                            device_actions=device_actions,
                        )
                        if action == "capture":
                            trainer.add_path_waypoint(name, f"wp{wp_index}")
                            wp_index += 1
                        elif action == "done":
                            trainer.finalize_path(name)
                            break
                        elif action == "skip":
                            trainer.finalize_path(name)
                            break
                        elif action == "redo":
                            trainer.discard_path(name)
                            wp_index = 1
                            _say(
                                logger,
                                f"path {name!r}: cleared, restart waypoint capture",
                            )
                        elif action == "quit":
                            _say(logger, "operator quit — partial YAML written")
                            return 0

            _say(
                logger,
                f"Phase {args.phase!r}: all entries captured. YAML at {output_path}",
            )
            if args.phase == "all":
                try:
                    pose_schema.validate(state.document)
                    _say(logger, "Final document passes schema validation.")
                except pose_schema.PoseFileError as exc:
                    _say(
                        logger,
                        f"Final document fails validation: {exc}",
                        error=True,
                    )
                    return 1
            else:
                _say(
                    logger,
                    f"Partial run: skipping full-document validation. Run with "
                    f"--phase <next> --resume to add more, or --phase all to finish.",
                )
        return 0
    finally:
        if arduino is not None:
            try:
                arduino.close()
            except Exception:  # noqa: BLE001 - best-effort cleanup
                pass


def _say(logger, msg: str, *, error: bool = False) -> None:
    if logger is not None:
        if error:
            getattr(logger, "error", logger.info)(msg)
        else:
            logger.info(msg)
    else:
        stream = sys.stderr if error else sys.stdout
        print(msg, file=stream, flush=True)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="train_pipeline_poses",
        description="Capture poses and paths for the injectable pipeline.",
    )
    parser.add_argument("--robot-sn", default=DEFAULT_ROBOT_SN)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--resume", action="store_true", help="Skip entries already in the output YAML"
    )
    parser.add_argument(
        "--overwrite", action="store_true", help="Delete the output YAML before starting"
    )
    parser.add_argument(
        "--start-at",
        default=None,
        help="Start the checklist at the named pose or path (skips earlier entries)",
    )
    parser.add_argument(
        "--include-optional",
        action="store_true",
        help="Also capture optional per-component disposal poses + paths",
    )
    parser.add_argument(
        "--init-gripper",
        action="store_true",
        help="Initialize the gripper at startup (required for the [g] device action)",
    )
    parser.add_argument("--gripper-name", default=DEFAULT_GRIPPER_NAME)
    parser.add_argument(
        "--arduino",
        action="store_true",
        help="Connect to the cutting-machine Arduino so [v] / [x] / [h] / [?] work during the walk-through",
    )
    parser.add_argument(
        "--arduino-port",
        default=None,
        help="Arduino serial port (auto-detect if omitted)",
    )
    parser.add_argument(
        "--arduino-auto-home",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Send HOME_ALL on Arduino connect if firmware reports homed=false (default on)",
    )
    parser.add_argument(
        "--phase",
        choices=pose_schema.PHASE_NAMES,
        default="all",
        help=(
            "Capture only a phase subset: 'pickup' = tray->vise (Phases 1+2), "
            "'cut' = park+cut (Phase 3), 'dispose' = vise->disposal->home "
            "(Phases 4-6), 'all' = full cycle (default)"
        ),
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    try:
        import spdlog  # type: ignore

        logger = spdlog.ConsoleLogger("PoseTrainer")
    except Exception:
        logger = None
    args = parse_args(argv)
    if args.resume and args.overwrite:
        print("--resume and --overwrite are mutually exclusive", file=sys.stderr)
        return 2
    return run_interactive(args, logger=logger)


if __name__ == "__main__":
    raise SystemExit(main())
