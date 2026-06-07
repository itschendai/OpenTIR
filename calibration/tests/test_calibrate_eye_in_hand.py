from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest


CALIBRATION_DIR = Path(__file__).resolve().parents[1]
if str(CALIBRATION_DIR) not in sys.path:
    sys.path.insert(0, str(CALIBRATION_DIR))

import calibrate_eye_in_hand as calib  # noqa: E402


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return Rz @ Ry @ Rx


def transform_delta(T_est: np.ndarray, T_true: np.ndarray) -> tuple[float, float]:
    delta = calib.invert_transform(T_est) @ T_true
    trans_m = float(np.linalg.norm(delta[:3, 3]))
    rot_deg = calib.rotation_angle_deg(delta[:3, :3])
    return trans_m, rot_deg


def test_pose_vec_to_transform_uses_flexiv_quaternion_order() -> None:
    pose = [0.1, -0.2, 0.3, math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)]
    T = calib.pose_vec_to_transform(pose)

    np.testing.assert_allclose(T[:3, 3], [0.1, -0.2, 0.3], atol=1e-12)
    np.testing.assert_allclose(
        T[:3, :3],
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        atol=1e-12,
    )


def test_invert_transform_round_trips_identity() -> None:
    T = calib.transform_from_rt(
        rpy_to_matrix(math.radians(12), math.radians(-7), math.radians(31)),
        [0.42, -0.11, 0.08],
    )

    np.testing.assert_allclose(T @ calib.invert_transform(T), np.eye(4), atol=1e-12)
    np.testing.assert_allclose(calib.invert_transform(T) @ T, np.eye(4), atol=1e-12)


def test_matrix_quaternion_round_trip() -> None:
    R = rpy_to_matrix(math.radians(20), math.radians(-30), math.radians(45))
    q = calib.matrix_to_quat_wxyz(R)
    R2 = calib.quat_wxyz_to_matrix(q)

    np.testing.assert_allclose(R2, R, atol=1e-12)


def test_synthetic_hand_eye_recovers_tcp_camera_transform() -> None:
    cv2 = pytest.importorskip("cv2")
    if not hasattr(cv2, "calibrateHandEye"):
        pytest.skip("OpenCV build does not expose calibrateHandEye")

    T_tcp_camera_true = calib.transform_from_rt(
        rpy_to_matrix(math.radians(4), math.radians(-12), math.radians(6)),
        [0.035, -0.018, 0.092],
    )
    T_world_board = calib.transform_from_rt(
        rpy_to_matrix(math.radians(15), math.radians(0), math.radians(-20)),
        [0.58, -0.12, 0.28],
    )

    T_world_tcp_list = []
    T_camera_board_list = []
    for i in range(12):
        roll = math.radians(-25 + 5 * i)
        pitch = math.radians(18 * math.sin(i * 0.7))
        yaw = math.radians(-50 + 9 * i)
        translation = [
            0.42 + 0.025 * math.cos(i),
            -0.08 + 0.018 * math.sin(i * 0.6),
            0.34 + 0.012 * i,
        ]
        T_world_tcp = calib.transform_from_rt(rpy_to_matrix(roll, pitch, yaw), translation)
        T_camera_board = (
            calib.invert_transform(T_tcp_camera_true)
            @ calib.invert_transform(T_world_tcp)
            @ T_world_board
        )
        T_world_tcp_list.append(T_world_tcp)
        T_camera_board_list.append(T_camera_board)

    T_tcp_camera_est = calib.solve_hand_eye_from_transforms(
        cv2, T_world_tcp_list, T_camera_board_list, method="tsai"
    )
    trans_m, rot_deg = transform_delta(T_tcp_camera_est, T_tcp_camera_true)

    assert trans_m < 1e-6
    assert rot_deg < 1e-4


def test_summarize_pose_delta_reports_world_and_target_frame_errors() -> None:
    current_tcp = [0.10, 0.20, 0.30, 1.0, 0.0, 0.0, 0.0]
    target_tcp = [0.15, 0.18, 0.33, 1.0, 0.0, 0.0, 0.0]

    summary = calib.summarize_pose_delta(current_tcp, target_tcp)

    np.testing.assert_allclose(summary["delta_world_mm"], [50.0, -20.0, 30.0], atol=1e-9)
    np.testing.assert_allclose(summary["delta_target_mm"], [50.0, -20.0, 30.0], atol=1e-9)
    assert summary["dpos_mm"] == pytest.approx(math.sqrt(50.0**2 + 20.0**2 + 30.0**2))
    assert summary["drot_deg"] == pytest.approx(0.0)


def test_build_verify_motion_plan_prefers_via_then_linear_final() -> None:
    via_tcp = [0.2, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0]
    target_tcp = [0.2, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0]

    plan = calib.build_verify_motion_plan(
        target_tcp,
        via_tcp=via_tcp,
        final_primitive="movel",
    )

    assert [(step.name, step.primitive) for step in plan] == [
        ("approach_via", "MovePTP"),
        ("final_target", "MoveL"),
    ]
    np.testing.assert_allclose(plan[0].target_tcp, via_tcp, atol=1e-12)
    np.testing.assert_allclose(plan[1].target_tcp, target_tcp, atol=1e-12)


def test_build_verify_motion_plan_skips_duplicate_via_pose() -> None:
    target_tcp = [0.2, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0]

    plan = calib.build_verify_motion_plan(
        target_tcp,
        via_tcp=target_tcp,
        final_primitive="moveptp",
    )

    assert [(step.name, step.primitive) for step in plan] == [
        ("final_target", "MovePTP")
    ]
