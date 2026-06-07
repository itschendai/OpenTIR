"""Unit tests for tune_pose."""

from __future__ import annotations

import math

import pytest

import pose_schema
import tune_pose as tp
from flexiv_helpers import quat_to_rpy_deg, rpy_deg_to_quat


# ---------- helper round-trips ----------


@pytest.mark.parametrize(
    "rpy",
    [
        (0.0, 0.0, 0.0),
        (180.0, 0.0, 0.0),
        (0.0, 0.0, 90.0),
        (-173.61, 80.18, -173.71),  # one of the captured pickup_grasp values
        (45.0, 30.0, -120.0),
    ],
)
def test_rpy_to_quat_round_trip(rpy):
    qw, qx, qy, qz = rpy_deg_to_quat(*rpy)
    back = quat_to_rpy_deg(qw, qx, qy, qz)
    # RPY representations can have +/- pi ambiguity; check by comparing the
    # resulting quaternions (which are unique up to global sign).
    qw2, qx2, qy2, qz2 = rpy_deg_to_quat(*back)
    if qw * qw2 + qx * qx2 + qy * qy2 + qz * qz2 < 0:
        qw2, qx2, qy2, qz2 = -qw2, -qx2, -qy2, -qz2
    assert qw == pytest.approx(qw2, abs=1e-6)
    assert qx == pytest.approx(qx2, abs=1e-6)
    assert qy == pytest.approx(qy2, abs=1e-6)
    assert qz == pytest.approx(qz2, abs=1e-6)


# ---------- helpers ----------


def _staged_doc(tmp_path):
    """Write a YAML with all required poses + one path waypoint."""
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
        wp = pose_schema.build_waypoint_entry(
            "wp1", [0.2] * 7, [0.5, 0.0, 0.45, 1.0, 0.0, 0.0, 0.0], 0.04, 0.0
        )
        state.record_path(
            spec.name, pose_schema.build_path_entry(spec.from_pose, spec.to_pose, [wp])
        )
    pose_schema.write_yaml(state.document, str(out))
    return str(out)


# ---------- error cases ----------


def test_missing_pose_exits_with_2(tmp_path, capsys):
    pose_file = _staged_doc(tmp_path)
    with pytest.raises(SystemExit) as exc:
        tp.main(["--pose-file", pose_file, "--pose", "bogus"])
    assert exc.value.code == 2


def test_missing_target_exits_with_2(tmp_path, capsys):
    pose_file = _staged_doc(tmp_path)
    with pytest.raises(SystemExit) as exc:
        tp.main(["--pose-file", pose_file])  # no --pose or --path
    assert exc.value.code == 2


def test_path_missing_waypoint_arg_exits_with_2(tmp_path):
    pose_file = _staged_doc(tmp_path)
    with pytest.raises(SystemExit) as exc:
        tp.main(["--pose-file", pose_file, "--path", "lifted_to_above_vise"])
    assert exc.value.code == 2


def test_path_waypoint_out_of_range_exits_with_2(tmp_path):
    pose_file = _staged_doc(tmp_path)
    with pytest.raises(SystemExit) as exc:
        tp.main(
            [
                "--pose-file",
                pose_file,
                "--path",
                "lifted_to_above_vise",
                "--waypoint",
                "5",
            ]
        )
    assert exc.value.code == 2


# ---------- dry-run behavior ----------


def test_dry_run_does_not_write(tmp_path):
    pose_file = _staged_doc(tmp_path)
    before = open(pose_file).read()
    rc = tp.main(["--pose-file", pose_file, "--pose", "home", "--set-yaw", "90"])
    assert rc == 0
    after = open(pose_file).read()
    assert before == after  # no --apply -> no write


def test_no_edits_returns_zero_no_change(tmp_path):
    pose_file = _staged_doc(tmp_path)
    before = open(pose_file).read()
    rc = tp.main(["--pose-file", pose_file, "--pose", "home"])
    assert rc == 0
    after = open(pose_file).read()
    assert before == after


def test_use_robot_without_edits_is_resync_request(tmp_path, monkeypatch):
    """--use-robot --apply with no edits should still attempt to settle.

    The expected behavior is "move the arm to the current TCP and recapture
    q_rad to match". The flag combination doesn't bail out with the
    no-edits-no-change shortcut. We monkey-patch _settle_via_robot so the
    test does not require a real robot.
    """
    pose_file = _staged_doc(tmp_path)
    settled = {"called": False}

    def fake_settle(entry, args, logger=None):
        settled["called"] = True

    monkeypatch.setattr(tp, "_settle_via_robot", fake_settle)
    rc = tp.main(
        ["--pose-file", pose_file, "--pose", "home", "--use-robot", "--apply"]
    )
    assert rc == 0
    assert settled["called"] is True


# ---------- edit semantics ----------


def test_apply_set_yaw_writes_new_quaternion(tmp_path):
    pose_file = _staged_doc(tmp_path)
    rc = tp.main(
        [
            "--pose-file",
            pose_file,
            "--pose",
            "home",
            "--set-yaw",
            "90",
            "--apply",
        ]
    )
    assert rc == 0
    doc = pose_schema.read_yaml(pose_file)
    tcp = doc["poses"]["home"]["tcp_pose_world"]["values"]
    rpy = quat_to_rpy_deg(tcp[3], tcp[4], tcp[5], tcp[6])
    assert rpy[2] == pytest.approx(90.0, abs=1e-4)


def test_apply_translate_mm_shifts_position(tmp_path):
    pose_file = _staged_doc(tmp_path)
    rc = tp.main(
        [
            "--pose-file",
            pose_file,
            "--pose",
            "home",
            "--translate-mm",
            "10",
            "-5",
            "20",
            "--apply",
        ]
    )
    assert rc == 0
    doc = pose_schema.read_yaml(pose_file)
    tcp = doc["poses"]["home"]["tcp_pose_world"]["values"]
    # Original was (0.5, 0.0, 0.4). After +10/-5/+20 mm: (0.510, -0.005, 0.420)
    assert tcp[0] == pytest.approx(0.510)
    assert tcp[1] == pytest.approx(-0.005)
    assert tcp[2] == pytest.approx(0.420)


def test_apply_set_z_mm_absolute(tmp_path):
    pose_file = _staged_doc(tmp_path)
    rc = tp.main(
        ["--pose-file", pose_file, "--pose", "home", "--set-z-mm", "250", "--apply"]
    )
    assert rc == 0
    doc = pose_schema.read_yaml(pose_file)
    tcp = doc["poses"]["home"]["tcp_pose_world"]["values"]
    assert tcp[2] == pytest.approx(0.250)
    # x and y unchanged
    assert tcp[0] == pytest.approx(0.5)
    assert tcp[1] == pytest.approx(0.0)


def test_lock_vertical_down_preserves_yaw(tmp_path):
    pose_file = _staged_doc(tmp_path)
    # Pre-set a known yaw so we can verify it survives the lock.
    tp.main(
        ["--pose-file", pose_file, "--pose", "home", "--set-yaw", "45", "--apply"]
    )
    rc = tp.main(
        [
            "--pose-file",
            pose_file,
            "--pose",
            "home",
            "--lock-vertical-down",
            "--apply",
        ]
    )
    assert rc == 0
    doc = pose_schema.read_yaml(pose_file)
    tcp = doc["poses"]["home"]["tcp_pose_world"]["values"]
    rpy = quat_to_rpy_deg(tcp[3], tcp[4], tcp[5], tcp[6])
    assert rpy[0] == pytest.approx(180.0, abs=1e-4)
    assert rpy[1] == pytest.approx(0.0, abs=1e-4)
    assert rpy[2] == pytest.approx(45.0, abs=1e-4)


def test_snap_yaw_rounds_to_nearest_step(tmp_path):
    pose_file = _staged_doc(tmp_path)
    tp.main(
        ["--pose-file", pose_file, "--pose", "home", "--set-yaw", "89.4", "--apply"]
    )
    rc = tp.main(
        [
            "--pose-file",
            pose_file,
            "--pose",
            "home",
            "--snap-yaw",
            "1",
            "--apply",
        ]
    )
    assert rc == 0
    doc = pose_schema.read_yaml(pose_file)
    tcp = doc["poses"]["home"]["tcp_pose_world"]["values"]
    rpy = quat_to_rpy_deg(tcp[3], tcp[4], tcp[5], tcp[6])
    assert rpy[2] == pytest.approx(89.0, abs=1e-4)


def test_set_rpy_replaces_all_three(tmp_path):
    pose_file = _staged_doc(tmp_path)
    rc = tp.main(
        [
            "--pose-file",
            pose_file,
            "--pose",
            "home",
            "--set-rpy",
            "10",
            "20",
            "30",
            "--apply",
        ]
    )
    assert rc == 0
    doc = pose_schema.read_yaml(pose_file)
    tcp = doc["poses"]["home"]["tcp_pose_world"]["values"]
    rpy = quat_to_rpy_deg(tcp[3], tcp[4], tcp[5], tcp[6])
    assert rpy[0] == pytest.approx(10.0, abs=1e-4)
    assert rpy[1] == pytest.approx(20.0, abs=1e-4)
    assert rpy[2] == pytest.approx(30.0, abs=1e-4)


def test_copy_orientation_from_uses_other_pose_rpy(tmp_path):
    pose_file = _staged_doc(tmp_path)
    # Give 'above_vise' a distinctive RPY.
    tp.main(
        [
            "--pose-file",
            pose_file,
            "--pose",
            "above_vise",
            "--set-rpy",
            "5",
            "-7",
            "11",
            "--apply",
        ]
    )
    # Copy its orientation onto 'home'. home's position must NOT change.
    rc = tp.main(
        [
            "--pose-file",
            pose_file,
            "--pose",
            "home",
            "--copy-orientation-from",
            "above_vise",
            "--apply",
        ]
    )
    assert rc == 0
    doc = pose_schema.read_yaml(pose_file)
    home_tcp = doc["poses"]["home"]["tcp_pose_world"]["values"]
    home_rpy = quat_to_rpy_deg(home_tcp[3], home_tcp[4], home_tcp[5], home_tcp[6])
    assert home_rpy[0] == pytest.approx(5.0, abs=1e-4)
    assert home_rpy[1] == pytest.approx(-7.0, abs=1e-4)
    assert home_rpy[2] == pytest.approx(11.0, abs=1e-4)
    # Position preserved.
    assert home_tcp[0] == pytest.approx(0.5)


def test_waypoint_edit_targets_correct_index(tmp_path):
    pose_file = _staged_doc(tmp_path)
    rc = tp.main(
        [
            "--pose-file",
            pose_file,
            "--path",
            "lifted_to_above_vise",
            "--waypoint",
            "1",
            "--translate-mm",
            "0",
            "0",
            "-5",
            "--apply",
        ]
    )
    assert rc == 0
    doc = pose_schema.read_yaml(pose_file)
    wp = doc["paths"]["lifted_to_above_vise"]["waypoints"][0]
    assert wp["tcp_pose_world"]["values"][2] == pytest.approx(0.445)


def test_apply_leaves_q_rad_unchanged_without_robot(tmp_path):
    """Paper edit must NOT modify q_rad; only TCP."""
    pose_file = _staged_doc(tmp_path)
    before_q = pose_schema.read_yaml(pose_file)["poses"]["home"]["q_rad"]
    rc = tp.main(
        ["--pose-file", pose_file, "--pose", "home", "--set-yaw", "90", "--apply"]
    )
    assert rc == 0
    after_q = pose_schema.read_yaml(pose_file)["poses"]["home"]["q_rad"]
    assert before_q == after_q
