"""Injectable Processing Pipeline orchestrator.

Single-process Python script that coordinates the Flexiv Rizon4 arm and the
cutting-machine Arduino through the six-phase cycle defined in
``planning/specifications.md`` §1.

This M4 milestone lands the PARAMS block, state machine, pose loader, and CLI with
stubbed phase implementations. The hardware-driving body of each phase is filled in by
later milestones (M5–M10); ``--dry-run`` walks the full state graph without touching
hardware and is the M4 acceptance criterion.
"""

from __future__ import annotations

import argparse
import enum
import os
import sys
import time
import traceback
from dataclasses import dataclass, field
from typing import Any, Callable

import pose_schema


# ---------------------------------------------------------------------------
# PARAMS — single source of tunable values. Mirrors ``specifications.md`` §1.1.
# Adding a key requires updating that table in the same change set.
# ---------------------------------------------------------------------------

PARAMS: dict[str, Any] = {
    # Robot
    "ROBOT_SN": "Rizon4-062930",
    # Arduino
    "ARDUINO_PORT": None,
    "ARDUINO_BAUD": 115200,
    "ARDUINO_DONE_TIMEOUT_S": 30.0,
    # Pose file
    "POSE_FILE": "pipeline_poses.yaml",
    # Cut recipe (Recipe1 second cut, frees the plastic top)
    "CUT_Z_MM": 134.0,
    "CUT_X_MM": 111.0,
    "CUT_DEG": 359.0,
    # Vise
    "VISE_TARGET_FORCE_KG": 4.0,
    "VISE_RELEASE_FORCE_KG": 0.2,
    # Hard geometric constraint: the injectable must drop into / be lifted from the
    # vise slot along world +Z only. Any motion that enters or exits the vise
    # region must include this many mm of pure-Z travel before any lateral
    # component is permitted, so the part stays aligned with the slot. Affected
    # phases: LOAD descent, REMOVE_TOP/SPRING/BODY descent + post-grasp lift.
    "VISE_VERTICAL_APPROACH_MM": 100.0,
    # Phase 1 / 2 compliance
    "PICKUP_Z_LIFT_MM": 50.0,
    "IMPEDANCE_KX_NM": 50.0,
    "IMPEDANCE_KY_NM": 50.0,
    "IMPEDANCE_KZ_NM": 3000.0,
    "IMPEDANCE_KROT_NMRAD": 40.0,
    "PICKUP_APPROACH_SPEED_MM_S": 5.0,
    "INSERT_FORCE_THRESHOLD_N": 15.0,
    "INSERT_DEPTH_MAX_MM": 40.0,
    "INSERT_SPEED_MM_S": 5.0,
    # Phase 4 — top removal
    "TWIST_ANGLE_DEG": 90.0,
    "TWIST_AXIS": "J7",
    "TWIST_LIFT_MM": 30.0,
    # Gripper
    "GRIPPER_OPEN_WIDTH_M": 0.04,
    "GRIPPER_CLOSE_WIDTH_M": 0.0,
    "GRIPPER_VELOCITY_M_S": 0.05,
    "GRIPPER_FORCE_LIMIT_N": 40.0,
    # MovePTP
    "MOVE_JNT_VEL_SCALE": 10,
    "MOVE_ZONE_RADIUS": "ZFine",
    # Cadence / logging
    "PHASE_PAUSE_S": 0.5,
    "LOG_DIR": "logs",
}


# ---------------------------------------------------------------------------
# Error codes — mirrors specifications.md §6.
# ---------------------------------------------------------------------------

class ErrorCode(str, enum.Enum):
    E_FLEXIV_FAULT = "E_FLEXIV_FAULT"
    E_FLEXIV_NOT_OPERATIONAL = "E_FLEXIV_NOT_OPERATIONAL"
    E_ARDUINO_TIMEOUT = "E_ARDUINO_TIMEOUT"
    E_ARDUINO_ERR = "E_ARDUINO_ERR"
    E_FORCE_LIMIT = "E_FORCE_LIMIT"
    E_POSE_MISSING = "E_POSE_MISSING"
    E_POSE_SCHEMA = "E_POSE_SCHEMA"
    E_GRIPPER_INIT = "E_GRIPPER_INIT"
    E_INTERRUPTED = "E_INTERRUPTED"
    E_STATE_MACHINE = "E_STATE_MACHINE"
    E_UNEXPECTED = "E_UNEXPECTED"


class OrchestratorError(Exception):
    def __init__(self, code: ErrorCode, message: str) -> None:
        super().__init__(f"{code.value}: {message}")
        self.code = code
        self.message = message


# ---------------------------------------------------------------------------
# Phase enum + ordered cycle.
# ---------------------------------------------------------------------------

class Phase(enum.Enum):
    STARTUP = "STARTUP"
    PICKUP = "PICKUP"
    LOAD = "LOAD"
    CUT = "CUT"
    REMOVE_TOP = "REMOVE_TOP"
    REMOVE_SPRING = "REMOVE_SPRING"
    REMOVE_BODY = "REMOVE_BODY"
    FAULT = "FAULT"
    SHUTDOWN = "SHUTDOWN"


CYCLE_PHASES: tuple[Phase, ...] = (
    Phase.PICKUP,
    Phase.LOAD,
    Phase.CUT,
    Phase.REMOVE_TOP,
    Phase.REMOVE_SPRING,
    Phase.REMOVE_BODY,
)


LEGAL_TRANSITIONS: dict[Phase, set[Phase]] = {
    Phase.STARTUP: {Phase.PICKUP, Phase.FAULT, Phase.SHUTDOWN},
    Phase.PICKUP: {Phase.LOAD, Phase.FAULT},
    Phase.LOAD: {Phase.CUT, Phase.FAULT},
    Phase.CUT: {Phase.REMOVE_TOP, Phase.FAULT},
    Phase.REMOVE_TOP: {Phase.REMOVE_SPRING, Phase.FAULT},
    Phase.REMOVE_SPRING: {Phase.REMOVE_BODY, Phase.FAULT},
    Phase.REMOVE_BODY: {Phase.PICKUP, Phase.SHUTDOWN, Phase.FAULT},
    Phase.FAULT: {Phase.SHUTDOWN},
    Phase.SHUTDOWN: set(),
}


# ---------------------------------------------------------------------------
# Pose loader.
# ---------------------------------------------------------------------------

def load_pose_document(path: str) -> dict:
    try:
        document = pose_schema.read_yaml(path)
        pose_schema.validate(document)
    except FileNotFoundError as exc:
        raise OrchestratorError(
            ErrorCode.E_POSE_MISSING, f"pose file not found: {path}"
        ) from exc
    except pose_schema.PoseFileError as exc:
        code = ErrorCode.E_POSE_MISSING if exc.code == "E_POSE_MISSING" else ErrorCode.E_POSE_SCHEMA
        raise OrchestratorError(code, exc.message) from exc
    return document


# ---------------------------------------------------------------------------
# Dry-run / mock clients.
# ---------------------------------------------------------------------------

class _PrintingLogger:
    """Minimal logger backstop when ``spdlog`` is unavailable."""

    def info(self, msg: str) -> None:
        print(f"[info]  {msg}", flush=True)

    def warn(self, msg: str) -> None:
        print(f"[warn]  {msg}", flush=True)

    def error(self, msg: str) -> None:
        print(f"[error] {msg}", file=sys.stderr, flush=True)

    def debug(self, msg: str) -> None:
        # Drop debug noise; orchestrator does not depend on debug log lines.
        pass


class MockArduinoClient:
    """Stand-in for ArduinoClient used in dry-run and ``--skip-arduino`` mode.

    Every method returns a fields dict shaped like the real one so the orchestrator
    can read fields like ``vise_state`` without branching.
    """

    def __init__(self, logger=None) -> None:
        self._logger = logger
        self._cmd_id = 0
        self._homed = True
        self._vise = "OPEN"
        self.calls: list[tuple[str, dict]] = []

    def _record(self, command: str, **fields) -> dict:
        self._cmd_id += 1
        if self._logger is not None:
            self._logger.info(f"[arduino-mock] {command} {fields}")
        result = dict(fields)
        result["command"] = command
        result["cmd_id"] = self._cmd_id
        self.calls.append((command, fields))
        return result

    # Lifecycle no-ops
    def connect(self) -> None: pass
    def close(self) -> None: pass

    def home_all(self) -> dict:
        self._homed = True
        return self._record("HOME_ALL", x_mm=0.0, z_mm=0.0, rot_deg=0.0,
                            busy=False, homed=True, faulted=False)

    def cut_height(self, z_mm: float, x_mm: float, deg: float) -> dict:
        return self._record(
            "CUT_HEIGHT", z_mm=z_mm, x_mm=x_mm, deg=deg,
            blade_on=False, busy=False, homed=True, faulted=False,
        )

    def close_vise(self, target_force_kg: float = 4.0) -> dict:
        self._vise = "CLOSED"
        return self._record("CLOSE_VISE", vise_state="CLOSED",
                            force_kg=target_force_kg, busy=False,
                            homed=True, faulted=False)

    def open_vise(self, target_force_kg: float = 0.2) -> dict:
        self._vise = "OPEN"
        return self._record("OPEN_VISE", vise_state="OPEN",
                            force_kg=0.0, busy=False, homed=True, faulted=False)

    def stop_all(self) -> dict:
        return self._record("STOP_ALL", blade_on=False, busy=False,
                            homed=True, faulted=False)

    def clear_faults(self) -> dict:
        return self._record("CLEAR_FAULTS", faulted=False, busy=False, homed=True)

    def get_status(self) -> dict:
        return self._record(
            "GET_STATUS", homed=True, busy=False, faulted=False,
            x_mm=0.0, z_mm=0.0, rot_deg=0.0, blade_on=False,
            vise_state=self._vise, force_kg=0.0, x_limit=False, z_limit=False,
            active_command="NONE",
        )


class MockRobotSession:
    """Stand-in for RobotSession in dry-run / ``--skip-flexiv`` mode."""

    def __init__(self, logger=None) -> None:
        self._logger = logger
        self.calls: list[tuple[str, tuple, dict]] = []
        self.entered = False

    def __enter__(self):
        self.entered = True
        if self._logger is not None:
            self._logger.info("[robot-mock] session enter")
        return self

    def __exit__(self, *_args):
        if self._logger is not None:
            self._logger.info("[robot-mock] session exit")

    def _record(self, name: str, *args, **kwargs):
        if self._logger is not None:
            self._logger.info(f"[robot-mock] {name}({args}, {kwargs})")
        self.calls.append((name, args, kwargs))
        return None

    def setup_gripper(self, gripper_name: str, init: bool = True):
        self._record("setup_gripper", gripper_name, init=init)
        return self  # return a sentinel; orchestrator only checks truthiness

    def switch_mode(self, mode_name: str):
        return self._record("switch_mode", mode_name)

    def execute_primitive(self, name: str, params: dict):
        return self._record("execute_primitive", name, params=params)

    def wait_for_primitive(self, state_key: str = "reachedTarget", dt: float = 0.2,
                           timeout_s: float | None = None):
        return self._record("wait_for_primitive", state_key)

    def selected_arm_state(self):
        # Return canned zero state.
        class _S:
            q = [0.0] * 7
            tcp_pose = [0.0] * 7
            ext_wrench_in_tcp = [0.0] * 6
        return "ARM", _S()

    def set_cartesian_impedance(self, kx, ky, kz, k_rot):
        return self._record("set_cartesian_impedance", kx, ky, kz, k_rot)

    def read_external_wrench(self) -> dict:
        return {k: 0.0 for k in ("fx", "fy", "fz", "mx", "my", "mz")}


# ---------------------------------------------------------------------------
# Orchestrator.
# ---------------------------------------------------------------------------


@dataclass
class CycleResult:
    cycle_index: int
    phases: list[Phase] = field(default_factory=list)
    fault: OrchestratorError | None = None


class Orchestrator:
    """Drives the six-phase cycle.

    The M4 implementation walks every phase as a stub: each phase prints its intent
    and the orchestrator-level state transitions. Later milestones replace each stub
    with the real hardware-driving body.
    """

    def __init__(
        self,
        params: dict,
        pose_document: dict,
        arduino_client,
        robot_session,
        logger=None,
        dry_run: bool = False,
    ) -> None:
        self.params = dict(params)
        self.poses = pose_document
        self.arduino = arduino_client
        self.session = robot_session
        self.logger = logger or _PrintingLogger()
        self.dry_run = dry_run
        self._phase = Phase.STARTUP
        self.transition_log: list[tuple[Phase, Phase]] = []

    @property
    def phase(self) -> Phase:
        return self._phase

    def transition(self, target: Phase) -> None:
        if target not in LEGAL_TRANSITIONS.get(self._phase, set()):
            raise OrchestratorError(
                ErrorCode.E_STATE_MACHINE,
                f"illegal phase transition {self._phase.value} -> {target.value}",
            )
        self.logger.info(f"phase: {self._phase.value} -> {target.value}")
        self.transition_log.append((self._phase, target))
        self._phase = target

    def run(self, cycles: int) -> int:
        try:
            try:
                self.transition(Phase.PICKUP)
                for cycle_index in range(1, cycles + 1):
                    self.logger.info(f"--- cycle {cycle_index}/{cycles} ---")
                    if cycle_index > 1:
                        # Coming off REMOVE_BODY of the previous cycle, walk back into PICKUP.
                        self.transition(Phase.PICKUP)
                    self._run_one_cycle(cycle_index)
                # After the final cycle the state is REMOVE_BODY → SHUTDOWN is legal.
                self.transition(Phase.SHUTDOWN)
                return 0
            except OrchestratorError as exc:
                self._handle_fault(exc)
                return 1
            except KeyboardInterrupt:
                self.logger.warn("interrupted by user")
                self._handle_fault(
                    OrchestratorError(ErrorCode.E_INTERRUPTED, "SIGINT")
                )
                return 130
            except Exception as exc:  # noqa: BLE001 - intentional broad catch
                # An unexpected hardware/SDK exception escaped the typed handlers.
                # Preserve the traceback for debugging, then drop into the fault
                # path so STOP_ALL fires before we return.
                self.logger.error(traceback.format_exc())
                wrapped = OrchestratorError(ErrorCode.E_UNEXPECTED, str(exc))
                self._handle_fault(wrapped)
                return 1
        finally:
            # Final safety net: covers paths that escape every except clause
            # (a bug inside ``_handle_fault`` itself, or a BaseException such as
            # SystemExit). Idempotent on the firmware side; never raises.
            self._best_effort_stop_all()

    def _best_effort_stop_all(self) -> None:
        try:
            self.arduino.stop_all()
        except Exception as inner:  # noqa: BLE001 - finally must never raise
            self.logger.warn(f"best-effort STOP_ALL raised: {inner}")

    def _run_one_cycle(self, cycle_index: int) -> None:
        cycle_phases = list(CYCLE_PHASES)
        # We're at PICKUP on entry (the run loop set it).
        for idx, phase in enumerate(cycle_phases):
            if idx > 0:
                self.transition(phase)
            self._dispatch(phase)
            self._pause()
        # Cycle ends with the state machine sitting on REMOVE_BODY.

    def _pause(self) -> None:
        pause = float(self.params.get("PHASE_PAUSE_S", 0.0))
        if self.dry_run or pause <= 0:
            return
        time.sleep(pause)

    def _dispatch(self, phase: Phase) -> None:
        handler = {
            Phase.PICKUP: self._phase_pickup,
            Phase.LOAD: self._phase_load,
            Phase.CUT: self._phase_cut,
            Phase.REMOVE_TOP: self._phase_remove_top,
            Phase.REMOVE_SPRING: self._phase_remove_spring,
            Phase.REMOVE_BODY: self._phase_remove_body,
        }[phase]
        handler()

    # ----- phase stubs (M4 only prints; real bodies arrive in M5..M10) -----

    def _phase_pickup(self) -> None:
        self.logger.info(
            "PICKUP: MovePTP home -> pickup_pre_grasp; compliant grasp; "
            "Z lift; path to above_vise"
        )

    def _phase_load(self) -> None:
        self.logger.info(
            "LOAD: compliant Z descent; CLOSE_VISE; open gripper; retreat to "
            "safe_intermediate"
        )

    def _phase_cut(self) -> None:
        self.logger.info(
            "CUT: send CUT_HEIGHT z={CUT_Z_MM}, x={CUT_X_MM}, deg={CUT_DEG}; "
            "wait for DONE".format(**self.params)
        )
        # In dry-run, exercise the mock Arduino so the planned wire calls are visible.
        if isinstance(self.arduino, MockArduinoClient) or self.dry_run:
            self.arduino.cut_height(
                z_mm=self.params["CUT_Z_MM"],
                x_mm=self.params["CUT_X_MM"],
                deg=self.params["CUT_DEG"],
            )

    def _phase_remove_top(self) -> None:
        self.logger.info(
            "REMOVE_TOP: path to above_vise; descent; grip; twist + lift; dispose"
        )

    def _phase_remove_spring(self) -> None:
        self.logger.info(
            "REMOVE_SPRING: return to above_vise; descent to spring; grip; lift; dispose"
        )

    def _phase_remove_body(self) -> None:
        self.logger.info(
            "REMOVE_BODY: descent; grip body; OPEN_VISE; lift; dispose"
        )
        if isinstance(self.arduino, MockArduinoClient) or self.dry_run:
            self.arduino.open_vise(target_force_kg=self.params["VISE_RELEASE_FORCE_KG"])

    # ----- fault path -----

    def _handle_fault(self, exc: OrchestratorError) -> None:
        self.logger.error(f"fault: {exc}")
        if self._phase not in {Phase.STARTUP, Phase.SHUTDOWN}:
            try:
                if self._phase != Phase.FAULT:
                    self.transition(Phase.FAULT)
            except OrchestratorError:
                pass
        # Best-effort STOP_ALL.
        try:
            self.arduino.stop_all()
        except Exception as inner:  # noqa: BLE001
            self.logger.warn(f"STOP_ALL during fault failed: {inner}")


# ---------------------------------------------------------------------------
# CLI / wiring.
# ---------------------------------------------------------------------------


def _make_logger() -> Any:
    try:
        import spdlog  # type: ignore

        return spdlog.ConsoleLogger("PipelineOrchestrator")
    except Exception:
        return _PrintingLogger()


def _resolve_pose_file(path: str) -> str:
    if os.path.isabs(path):
        return path
    return os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), path))


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="pipeline_orchestrator",
        description="Run the injectable disassembly cycle.",
    )
    parser.add_argument("--robot-sn", default=None)
    parser.add_argument("--arduino-port", default=None)
    parser.add_argument("--pose-file", default=None)
    parser.add_argument("--cycles", type=int, default=None,
                        help="Number of cycles to run; default infinite until Ctrl-C")
    parser.add_argument("--once", action="store_true", help="Equivalent to --cycles 1")
    parser.add_argument("--dry-run", action="store_true",
                        help="Walk the state machine without connecting to hardware")
    parser.add_argument("--skip-arduino", action="store_true",
                        help="Mock the Arduino client")
    parser.add_argument("--skip-flexiv", action="store_true",
                        help="Mock the robot session")
    parser.add_argument("--yes", action="store_true",
                        help="Skip the startup confirmation prompt")
    return parser.parse_args(argv)


def resolve_params(args: argparse.Namespace) -> dict:
    params = dict(PARAMS)
    if args.robot_sn is not None:
        params["ROBOT_SN"] = args.robot_sn
    if args.arduino_port is not None:
        params["ARDUINO_PORT"] = args.arduino_port
    if args.pose_file is not None:
        params["POSE_FILE"] = args.pose_file
    return params


def _build_arduino(args, params, logger):
    if args.dry_run or args.skip_arduino:
        return MockArduinoClient(logger=logger)
    # Lazy import so unit tests do not need pyserial in CI.
    from arduino_client import ArduinoClient

    return ArduinoClient(
        port=params["ARDUINO_PORT"],
        baud=params["ARDUINO_BAUD"],
        done_timeout_s=params["ARDUINO_DONE_TIMEOUT_S"],
        logger=logger,
    )


def _build_session(args, params, logger):
    if args.dry_run or args.skip_flexiv:
        return MockRobotSession(logger=logger)
    from flexiv_helpers import RobotSession

    return RobotSession(robot_sn=params["ROBOT_SN"], logger=logger)


def _cycles_count(args) -> int:
    if args.once:
        return 1
    if args.cycles is None:
        # 10 ** 9 stands in for "infinite"; the operator can Ctrl-C.
        return 10**9
    return int(args.cycles)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logger = _make_logger()
    params = resolve_params(args)
    pose_path = _resolve_pose_file(params["POSE_FILE"])

    try:
        document = load_pose_document(pose_path)
    except OrchestratorError as exc:
        logger.error(str(exc))
        return 1

    arduino = _build_arduino(args, params, logger)
    session = _build_session(args, params, logger)

    if args.dry_run:
        logger.info(f"[dry-run] pose file: {pose_path}")
        logger.info(f"[dry-run] cycles: {_cycles_count(args)}")

    try:
        if hasattr(arduino, "connect") and not isinstance(arduino, MockArduinoClient):
            arduino.connect()
        with session:
            orch = Orchestrator(
                params=params,
                pose_document=document,
                arduino_client=arduino,
                robot_session=session,
                logger=logger,
                dry_run=args.dry_run,
            )
            return orch.run(_cycles_count(args))
    finally:
        try:
            arduino.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
