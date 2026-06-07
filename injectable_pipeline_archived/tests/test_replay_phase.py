"""Unit tests for replay_phase (Path A visual sanity replay).

The hardware path is mocked through a FakeSession that records every
`switch_mode`, `execute_primitive`, and `wait_for_primitive` call. The
flexivrdk module itself is monkey-patched onto the fake session with stubbed
`Coord` and `JPos` constructors that just box their arguments — that way the
test verifies replay_phase passes through the right data without needing the
real SDK.
"""

from __future__ import annotations

import os
from typing import Any

import pytest

import pose_schema
import replay_phase as rp


# --------- fakes ---------


class _FakeFlexivrdk:
    """Minimal flexivrdk stand-in: Coord and JPos box their args verbatim."""

    class Coord:
        def __init__(self, position, orientation, ref_frame, ref_q_m, ref_q_e):
            self.position = list(position)
            self.orientation = list(orientation)
            self.ref_frame = list(ref_frame)
            self.ref_q_m = list(ref_q_m)
            self.ref_q_e = list(ref_q_e)

    class JPos:
        def __init__(self, q_deg):
            self.q_deg = list(q_deg)


class _FakeSession:
    """Stand-in for RobotSession that records every call."""

    def __init__(self) -> None:
        self.flexivrdk = _FakeFlexivrdk
        self.calls: list[tuple[str, Any]] = []

    def __enter__(self):
        self.calls.append(("__enter__", None))
        return self

    def __exit__(self, exc_type, exc, tb):
        self.calls.append(("__exit__", (exc_type, exc, tb)))
        return None

    def switch_mode(self, mode_name):
        self.calls.append(("switch_mode", mode_name))

    def execute_primitive(self, name, params):
        self.calls.append(("execute_primitive", (name, params)))

    def wait_for_primitive(self, state_key="reachedTarget", dt=0.2, timeout_s=None):
        self.calls.append(("wait_for_primitive", state_key))


# --------- argparse ---------


def test_parse_args_defaults():
    args = rp.parse_args([])
    assert args.robot_sn == rp.DEFAULT_ROBOT_SN
    assert args.pose_file == rp.DEFAULT_POSE_FILE
    assert args.phase == "pickup"
    assert args.jnt_vel_scale == rp.DEFAULT_JNT_VEL_SCALE


def test_parse_args_accepts_all_phases():
    for name in pose_schema.PHASE_NAMES:
        args = rp.parse_args(["--phase", name])
        assert args.phase == name


def test_parse_args_rejects_unknown_phase():
    with pytest.raises(SystemExit):
        rp.parse_args(["--phase", "garbage"])


# --------- run_replay error paths ---------


def test_run_replay_missing_pose_file_returns_2(tmp_path):
    args = rp.parse_args(["--pose-file", str(tmp_path / "nope.yaml")])
    rc = rp.run_replay(args)
    assert rc == 2


def test_run_replay_missing_entry_returns_1(tmp_path):
    # Write a YAML with only 'home' captured but ask for full pickup phase.
    out = tmp_path / "poses.yaml"
    state = pose_schema.TrainerState.fresh("Rizon4-062930")
    state.record_pose(
        "home",
        pose_schema.build_pose_entry([0.1] * 7, [0.5, 0, 0.4, 1, 0, 0, 0], 0.04, 0.0),
    )
    pose_schema.write_yaml(state.document, str(out))
    args = rp.parse_args(["--pose-file", str(out), "--phase", "pickup"])
    rc = rp.run_replay(args)
    assert rc == 1


# --------- happy path with valid fixture ---------


def _staged_full_yaml(tmp_path) -> str:
    """Write a YAML with every required pose + path so run_replay can succeed."""
    out = tmp_path / "poses.yaml"
    state = pose_schema.TrainerState.fresh("Rizon4-062930")
    for name in pose_schema.REQUIRED_POSES:
        state.record_pose(
            name,
            pose_schema.build_pose_entry(
                [0.1 * (i + 1) for i in range(7)],
                [0.5, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0],
                0.04,
                0.0,
            ),
        )
    for spec in pose_schema.REQUIRED_PATHS:
        # Each path gets a single intermediate waypoint so the replay loop has
        # something to dispatch.
        wp = pose_schema.build_waypoint_entry(
            "wp1",
            [0.2 * (i + 1) for i in range(7)],
            [0.5, 0.0, 0.45, 1.0, 0.0, 0.0, 0.0],
            0.04,
            0.0,
        )
        state.record_path(
            spec.name, pose_schema.build_path_entry(spec.from_pose, spec.to_pose, [wp])
        )
    pose_schema.write_yaml(state.document, str(out))
    return str(out)


def test_run_replay_pickup_dispatches_movePTP_in_order(tmp_path):
    pose_file = _staged_full_yaml(tmp_path)
    args = rp.parse_args(
        ["--pose-file", pose_file, "--phase", "pickup", "--jnt-vel-scale", "3"]
    )
    fake = _FakeSession()
    rc = rp.run_replay(args, session_factory=lambda: fake)
    assert rc == 0

    # The pickup phase has 5 poses + 1 path (1 waypoint each) = 6 MovePTPs.
    movePTPs = [c for c in fake.calls if c[0] == "execute_primitive"]
    waits = [c for c in fake.calls if c[0] == "wait_for_primitive"]
    assert len(movePTPs) == 6
    assert len(waits) == 6
    # First call sets the primitive-execution mode before any motion.
    mode_idx = next(i for i, c in enumerate(fake.calls) if c[0] == "switch_mode")
    first_move_idx = next(
        i for i, c in enumerate(fake.calls) if c[0] == "execute_primitive"
    )
    assert mode_idx < first_move_idx
    assert fake.calls[mode_idx][1] == "NRT_PRIMITIVE_EXECUTION"

    # Every MovePTP carries the conservative vel scale and the IK seed.
    for call_name, (prim_name, params) in movePTPs:
        assert prim_name == "MovePTP"
        assert params["jntVelScale"] == 3
        assert params["enableFixRefJntPos"] is True
        assert isinstance(params["refJntPos"], _FakeFlexivrdk.JPos)
        assert isinstance(params["target"], _FakeFlexivrdk.Coord)


def test_run_replay_skips_empty_path(tmp_path):
    """A path with zero waypoints (operator pressed 'd' immediately) is skipped."""
    out = tmp_path / "poses.yaml"
    state = pose_schema.TrainerState.fresh("Rizon4-062930")
    for name in pose_schema.REQUIRED_POSES:
        state.record_pose(
            name,
            pose_schema.build_pose_entry([0.1] * 7, [0.5, 0, 0.4, 1, 0, 0, 0], 0.04, 0.0),
        )
    for spec in pose_schema.REQUIRED_PATHS:
        # No waypoints on any path.
        state.record_path(
            spec.name, pose_schema.build_path_entry(spec.from_pose, spec.to_pose, [])
        )
    pose_schema.write_yaml(state.document, str(out))

    args = rp.parse_args(["--pose-file", str(out), "--phase", "pickup"])
    fake = _FakeSession()
    rc = rp.run_replay(args, session_factory=lambda: fake)
    assert rc == 0
    # 5 poses, 0 waypoints = 5 MovePTP calls.
    movePTPs = [c for c in fake.calls if c[0] == "execute_primitive"]
    assert len(movePTPs) == 5


def test_run_replay_cut_phase_uses_two_paths(tmp_path):
    pose_file = _staged_full_yaml(tmp_path)
    args = rp.parse_args(["--pose-file", pose_file, "--phase", "cut"])
    fake = _FakeSession()
    rc = rp.run_replay(args, session_factory=lambda: fake)
    assert rc == 0
    # cut phase: 2 poses (above_vise, safe_intermediate) + 2 paths (1 wp each).
    movePTPs = [c for c in fake.calls if c[0] == "execute_primitive"]
    assert len(movePTPs) == 4
