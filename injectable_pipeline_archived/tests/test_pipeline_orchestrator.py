"""Unit + integration tests for the M4 orchestrator skeleton."""

from __future__ import annotations

import io
import os
import shutil
import sys
from contextlib import redirect_stdout, redirect_stderr

import pytest

import pipeline_orchestrator as orc
import pose_schema


HERE = os.path.dirname(os.path.abspath(__file__))
FIXTURES = os.path.join(HERE, "fixtures")


# ----------------- PARAMS table -----------------

EXPECTED_PARAM_KEYS = {
    "ROBOT_SN", "ARDUINO_PORT", "ARDUINO_BAUD", "ARDUINO_DONE_TIMEOUT_S",
    "POSE_FILE",
    "CUT_Z_MM", "CUT_X_MM", "CUT_DEG",
    "VISE_TARGET_FORCE_KG", "VISE_RELEASE_FORCE_KG", "VISE_VERTICAL_APPROACH_MM",
    "PICKUP_Z_LIFT_MM",
    "IMPEDANCE_KX_NM", "IMPEDANCE_KY_NM", "IMPEDANCE_KZ_NM", "IMPEDANCE_KROT_NMRAD",
    "PICKUP_APPROACH_SPEED_MM_S",
    "INSERT_FORCE_THRESHOLD_N", "INSERT_DEPTH_MAX_MM", "INSERT_SPEED_MM_S",
    "TWIST_ANGLE_DEG", "TWIST_AXIS", "TWIST_LIFT_MM",
    "GRIPPER_OPEN_WIDTH_M", "GRIPPER_CLOSE_WIDTH_M",
    "GRIPPER_VELOCITY_M_S", "GRIPPER_FORCE_LIMIT_N",
    "MOVE_JNT_VEL_SCALE", "MOVE_ZONE_RADIUS",
    "PHASE_PAUSE_S", "LOG_DIR",
}


def test_params_block_matches_spec_keys():
    assert set(orc.PARAMS.keys()) == EXPECTED_PARAM_KEYS


def test_params_default_values():
    assert orc.PARAMS["ROBOT_SN"] == "Rizon4-062930"
    assert orc.PARAMS["ARDUINO_BAUD"] == 115200
    assert orc.PARAMS["CUT_Z_MM"] == 134.0
    assert orc.PARAMS["CUT_X_MM"] == 111.0
    assert orc.PARAMS["CUT_DEG"] == 359.0
    assert orc.PARAMS["VISE_TARGET_FORCE_KG"] == 4.0
    assert orc.PARAMS["VISE_RELEASE_FORCE_KG"] == 0.2
    assert orc.PARAMS["MOVE_ZONE_RADIUS"] == "ZFine"


# ----------------- pose loader -----------------


def test_load_pose_document_accepts_valid_fixture():
    doc = orc.load_pose_document(os.path.join(FIXTURES, "pipeline_poses_valid.yaml"))
    assert doc["schema_version"] == 1
    assert "above_vise" in doc["poses"]


def test_load_pose_document_reports_missing_entries():
    with pytest.raises(orc.OrchestratorError) as exc_info:
        orc.load_pose_document(os.path.join(FIXTURES, "pipeline_poses_missing.yaml"))
    assert exc_info.value.code == orc.ErrorCode.E_POSE_MISSING
    assert "safe_intermediate" in exc_info.value.message


def test_load_pose_document_missing_file_is_E_POSE_MISSING(tmp_path):
    with pytest.raises(orc.OrchestratorError) as exc_info:
        orc.load_pose_document(str(tmp_path / "does_not_exist.yaml"))
    assert exc_info.value.code == orc.ErrorCode.E_POSE_MISSING


# ----------------- state machine -----------------


def _build_orchestrator(dry_run=True):
    doc = orc.load_pose_document(os.path.join(FIXTURES, "pipeline_poses_valid.yaml"))
    logger = orc._PrintingLogger()
    return orc.Orchestrator(
        params=orc.PARAMS,
        pose_document=doc,
        arduino_client=orc.MockArduinoClient(logger=logger),
        robot_session=orc.MockRobotSession(logger=logger),
        logger=logger,
        dry_run=dry_run,
    )


def test_legal_transitions_table_covers_each_phase():
    for phase in orc.Phase:
        assert phase in orc.LEGAL_TRANSITIONS


def test_orchestrator_illegal_transition_raises():
    o = _build_orchestrator()
    with pytest.raises(orc.OrchestratorError):
        o.transition(orc.Phase.CUT)  # STARTUP -> CUT is not allowed


def test_orchestrator_legal_path_to_fault_from_any_phase():
    # STARTUP can fault. After moving through PICKUP, LOAD, ... we can still fault.
    o = _build_orchestrator()
    o.transition(orc.Phase.PICKUP)
    o.transition(orc.Phase.LOAD)
    o.transition(orc.Phase.FAULT)
    assert o.phase == orc.Phase.FAULT


# ----------------- dry-run full cycle -----------------


def _capture_dryrun(cycles: int = 1) -> str:
    o = _build_orchestrator(dry_run=True)
    buf = io.StringIO()
    with redirect_stdout(buf), redirect_stderr(buf):
        rc = o.run(cycles)
    assert rc == 0
    return buf.getvalue()


def test_dry_run_one_cycle_lists_each_phase_in_order():
    out = _capture_dryrun(cycles=1)
    for phrase in ("PICKUP", "LOAD", "CUT", "REMOVE_TOP", "REMOVE_SPRING", "REMOVE_BODY"):
        assert phrase in out, f"{phrase!r} not in dry-run output"


def test_dry_run_two_cycles_emits_two_pickup_load_cut_runs():
    out = _capture_dryrun(cycles=2)
    # Each phase logged twice (once per cycle) at minimum.
    assert out.count("PICKUP: MovePTP") == 2
    assert out.count("LOAD: compliant Z descent") == 2
    assert out.count("CUT: send CUT_HEIGHT") == 2


def test_dry_run_invokes_cut_height_on_mock_arduino():
    o = _build_orchestrator(dry_run=True)
    rc = o.run(1)
    assert rc == 0
    assert any(call[0] == "CUT_HEIGHT" for call in o.arduino.calls)
    assert any(call[0] == "OPEN_VISE" for call in o.arduino.calls)


# ----------------- CLI integration: main() with --dry-run --once -----------------


def test_main_dry_run_once_returns_zero(tmp_path, monkeypatch):
    # Stage the valid YAML next to the script so default
    # POSE_FILE = "pipeline_poses.yaml" resolves to it. If a live (possibly
    # partial) operator file is already there, temporarily move it aside so the
    # test always validates against the known-good fixture. The original is
    # restored on test exit, regardless of pass/fail.
    valid = os.path.join(FIXTURES, "pipeline_poses_valid.yaml")
    package_dir = os.path.dirname(os.path.abspath(orc.__file__))
    staged = os.path.join(package_dir, "pipeline_poses.yaml")
    backup = staged + ".test-backup"
    pre_existing = os.path.exists(staged)
    if pre_existing:
        os.rename(staged, backup)
    try:
        shutil.copyfile(valid, staged)
        rc = orc.main(["--dry-run", "--once", "--yes"])
        assert rc == 0
    finally:
        if os.path.exists(staged):
            os.unlink(staged)
        if pre_existing and os.path.exists(backup):
            os.rename(backup, staged)


def test_main_with_explicit_pose_file_arg():
    rc = orc.main(
        ["--dry-run", "--once",
         "--pose-file", os.path.join(FIXTURES, "pipeline_poses_valid.yaml")]
    )
    assert rc == 0


def test_main_failed_pose_load_returns_one():
    rc = orc.main(
        ["--dry-run", "--once",
         "--pose-file", os.path.join(FIXTURES, "pipeline_poses_missing.yaml")]
    )
    assert rc == 1


def test_main_does_not_touch_real_serial_in_dry_run(monkeypatch):
    # If we accidentally instantiated the real ArduinoClient, this would fail to find
    # a port on CI. The test passes only when --dry-run picks the mock.
    rc = orc.main(
        ["--dry-run", "--once",
         "--pose-file", os.path.join(FIXTURES, "pipeline_poses_valid.yaml")]
    )
    assert rc == 0


# ----------------- skip flags -----------------


def test_skip_arduino_uses_mock_client():
    rc = orc.main(
        ["--skip-arduino", "--skip-flexiv", "--once",
         "--pose-file", os.path.join(FIXTURES, "pipeline_poses_valid.yaml")]
    )
    assert rc == 0


def test_resolve_params_overrides():
    args = orc.parse_args(
        ["--robot-sn", "Rizon4-XXYY", "--arduino-port", "/dev/ttyACM7",
         "--pose-file", "elsewhere.yaml", "--once", "--dry-run"]
    )
    params = orc.resolve_params(args)
    assert params["ROBOT_SN"] == "Rizon4-XXYY"
    assert params["ARDUINO_PORT"] == "/dev/ttyACM7"
    assert params["POSE_FILE"] == "elsewhere.yaml"


def test_cycles_count_priority():
    assert orc._cycles_count(orc.parse_args(["--once"])) == 1
    assert orc._cycles_count(orc.parse_args(["--cycles", "3"])) == 3
    # Unbounded default → very large stand-in.
    assert orc._cycles_count(orc.parse_args([])) > 1000


# ----------------- M2: E_STATE_MACHINE on illegal transitions -----------------


def test_illegal_transition_raises_E_STATE_MACHINE():
    o = _build_orchestrator()
    with pytest.raises(orc.OrchestratorError) as exc_info:
        o.transition(orc.Phase.CUT)  # STARTUP -> CUT is not allowed
    assert exc_info.value.code == orc.ErrorCode.E_STATE_MACHINE


def test_error_code_enum_includes_state_machine_and_unexpected():
    assert orc.ErrorCode.E_STATE_MACHINE.value == "E_STATE_MACHINE"
    assert orc.ErrorCode.E_UNEXPECTED.value == "E_UNEXPECTED"


# ----------------- M3: broad-catch + try/finally STOP_ALL guard -----------------


class _BufferLogger:
    """Captures info/warn/error lines in memory for assertions."""

    def __init__(self) -> None:
        self.info_lines: list[str] = []
        self.warn_lines: list[str] = []
        self.error_lines: list[str] = []

    def info(self, msg: str) -> None:
        self.info_lines.append(msg)

    def warn(self, msg: str) -> None:
        self.warn_lines.append(msg)

    def error(self, msg: str) -> None:
        self.error_lines.append(msg)

    def debug(self, msg: str) -> None:
        pass

    @property
    def all_text(self) -> str:
        return "\n".join(self.info_lines + self.warn_lines + self.error_lines)


def _build_orchestrator_with_logger(logger):
    doc = orc.load_pose_document(os.path.join(FIXTURES, "pipeline_poses_valid.yaml"))
    return orc.Orchestrator(
        params=orc.PARAMS,
        pose_document=doc,
        arduino_client=orc.MockArduinoClient(logger=logger),
        robot_session=orc.MockRobotSession(logger=logger),
        logger=logger,
        dry_run=True,
    )


def test_run_catches_unexpected_runtime_error_and_stops_arduino(monkeypatch):
    logger = _BufferLogger()
    o = _build_orchestrator_with_logger(logger)

    def _boom(self) -> None:
        raise RuntimeError("simulated SDK explosion mid-pickup")

    monkeypatch.setattr(orc.Orchestrator, "_phase_pickup", _boom)

    rc = o.run(1)
    assert rc == 1
    stop_calls = [call for call in o.arduino.calls if call[0] == "STOP_ALL"]
    assert stop_calls, "STOP_ALL was not sent to the Arduino on unexpected exception"
    # Error log must include the wrapped code and a traceback.
    text = logger.all_text
    assert "E_UNEXPECTED" in text
    assert "Traceback" in text
    assert "simulated SDK explosion" in text


def test_run_finally_logs_when_arduino_stop_all_raises(monkeypatch):
    """Per M4.5 M3: arduino.stop_all() raising inside _handle_fault must not
    propagate out of run(); the finally-clause STOP_ALL attempt must be logged."""
    logger = _BufferLogger()
    o = _build_orchestrator_with_logger(logger)

    def _boom(self) -> None:
        raise RuntimeError("phase blew up")

    monkeypatch.setattr(orc.Orchestrator, "_phase_pickup", _boom)

    stop_attempts = {"count": 0}

    def _raise_stop():
        stop_attempts["count"] += 1
        raise RuntimeError("arduino refuses to STOP_ALL")

    o.arduino.stop_all = _raise_stop

    rc = o.run(1)
    # run() must still return — never propagate.
    assert rc == 1
    # Both _handle_fault and the finally-clause _best_effort_stop_all should have
    # called stop_all, so we expect at least two attempts.
    assert stop_attempts["count"] >= 2
    # The finally-clause attempt must be logged on the warn channel.
    assert any("best-effort STOP_ALL" in line for line in logger.warn_lines)


def test_best_effort_stop_all_swallows_arduino_exception(monkeypatch):
    logger = _BufferLogger()
    o = _build_orchestrator_with_logger(logger)

    def _raise_stop():
        raise RuntimeError("arduino blew up on STOP_ALL")

    o.arduino.stop_all = _raise_stop

    # Must not propagate.
    o._best_effort_stop_all()
    assert any("best-effort STOP_ALL" in line for line in logger.warn_lines)


def test_clean_run_still_fires_finally_stop_all():
    logger = _BufferLogger()
    o = _build_orchestrator_with_logger(logger)
    rc = o.run(1)
    assert rc == 0
    # The finally clause issues at least one redundant STOP_ALL on every clean exit.
    stop_calls = [call for call in o.arduino.calls if call[0] == "STOP_ALL"]
    assert stop_calls, "finally-clause STOP_ALL was missing from a clean run"
