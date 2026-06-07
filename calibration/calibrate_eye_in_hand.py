#!/usr/bin/env python3

"""Eye-in-hand calibration for a RealSense camera mounted near the TCP.

The workflow assumes the ChArUco board stays fixed in the workspace while the
robot moves the gripper-mounted camera through multiple views.

Examples:
    python calibrate_eye_in_hand.py capture --prompt
    python calibrate_eye_in_hand.py solve
    python calibrate_eye_in_hand.py validate
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml


HERE = Path(__file__).resolve().parent
PROJECT_DIR = HERE.parent
DEFAULT_BOARD_PATH = HERE / "tag_01.yaml"
DEFAULT_SAMPLES_DIR = HERE / "samples"
DEFAULT_CALIBRATION_PATH = HERE / "camera_tcp.yaml"
DEFAULT_ROBOT_SN = "Rizon4-062930"

TCP_POSE_ORDER = ["x", "y", "z", "qw", "qx", "qy", "qz"]

HAND_EYE_METHODS = {
    "tsai": "CALIB_HAND_EYE_TSAI",
    "park": "CALIB_HAND_EYE_PARK",
    "horaud": "CALIB_HAND_EYE_HORAUD",
    "andreff": "CALIB_HAND_EYE_ANDREFF",
    "daniilidis": "CALIB_HAND_EYE_DANIILIDIS",
}


@dataclass
class CharucoPose:
    T_camera_board: np.ndarray
    rvec: np.ndarray
    tvec: np.ndarray
    marker_count: int
    charuco_count: int
    reprojection_error_px: float


@dataclass
class SampleDetection:
    sample_dir: Path
    sample_yaml: Path
    image_path: Path
    metadata: dict[str, Any]
    T_world_tcp: np.ndarray
    T_camera_board: np.ndarray
    marker_count: int
    charuco_count: int
    reprojection_error_px: float


# ---------------------------------------------------------------------------
# Dependency loading


def require_cv2():
    try:
        import cv2  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "Missing dependency: cv2. Install OpenCV with ArUco/ChArUco support:\n"
            "  pip install opencv-contrib-python"
        ) from exc

    if not hasattr(cv2, "aruco"):
        raise SystemExit(
            "The installed OpenCV build does not include cv2.aruco.\n"
            "Install the contrib build instead:\n"
            "  pip install opencv-contrib-python"
        )
    return cv2


def require_realsense():
    try:
        import pyrealsense2 as rs  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "Missing dependency: pyrealsense2.\n"
            "Install it with:\n"
            "  pip install pyrealsense2"
        ) from exc
    return rs


def import_robot_session():
    pipeline_dir = PROJECT_DIR / "helper"
    if str(pipeline_dir) not in sys.path:
        sys.path.insert(0, str(pipeline_dir))
    try:
        from flexiv_helpers import RobotSession  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "Could not import the Flexiv helper module from "
            f"{pipeline_dir}. Make sure this script is run from the repo checkout."
        ) from exc
    return RobotSession


def import_flexiv_helpers():
    pipeline_dir = PROJECT_DIR / "helper"
    if str(pipeline_dir) not in sys.path:
        sys.path.insert(0, str(pipeline_dir))
    try:
        import flexiv_helpers  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "Could not import the Flexiv helper module from "
            f"{pipeline_dir}. Make sure this script is run from the repo checkout."
        ) from exc
    return flexiv_helpers


def temporarily_release_shared_camera(serial: str | None):
    if str(PROJECT_DIR) not in sys.path:
        sys.path.insert(0, str(PROJECT_DIR))
    try:
        from helper.injectable_camera_session import temporarily_release_shared_camera as _release
    except ImportError:
        from contextlib import nullcontext

        return nullcontext()
    return _release(serial)


def make_logger(name: str = "HandEyeCalibration"):
    try:
        import spdlog  # type: ignore

        return spdlog.ConsoleLogger(name)
    except Exception:
        return None


# ---------------------------------------------------------------------------
# YAML and formatting helpers


def read_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


def write_yaml(data: dict[str, Any], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def timestamp_slug() -> str:
    return datetime.now().strftime("%Y%m%dT%H%M%S_%f")


def relative_to_repo(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(PROJECT_DIR.parent.resolve()))
    except ValueError:
        return str(path.resolve())


def as_float(value: Any) -> float:
    return float(np.asarray(value).reshape(()))


def matrix_to_list(T: np.ndarray) -> list[list[float]]:
    return [[float(v) for v in row] for row in np.asarray(T, dtype=float)]


def vector_to_list(values: Iterable[float]) -> list[float]:
    return [float(v) for v in values]


# ---------------------------------------------------------------------------
# Rigid-transform helpers


def quat_wxyz_to_matrix(quat_wxyz: Iterable[float]) -> np.ndarray:
    q = np.asarray(list(quat_wxyz), dtype=float)
    if q.shape != (4,):
        raise ValueError(f"Quaternion must have 4 values, got {q.shape}")
    norm = np.linalg.norm(q)
    if norm == 0:
        raise ValueError("Quaternion norm is zero")
    w, x, y, z = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    R = np.asarray(R, dtype=float)
    if R.shape != (3, 3):
        raise ValueError(f"Rotation matrix must be 3x3, got {R.shape}")

    trace = float(np.trace(R))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(max(0.0, 1.0 + R[1, 1] - R[0, 0] - R[2, 2])) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(max(0.0, 1.0 + R[2, 2] - R[0, 0] - R[1, 1])) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    q = np.array([w, x, y, z], dtype=float)
    return q / np.linalg.norm(q)


def transform_from_rt(R: np.ndarray, t: Iterable[float]) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, :3] = np.asarray(R, dtype=float).reshape(3, 3)
    T[:3, 3] = np.asarray(list(t), dtype=float).reshape(3)
    return T


def pose_vec_to_transform(pose: Iterable[float]) -> np.ndarray:
    values = [float(v) for v in pose]
    if len(values) != 7:
        raise ValueError(f"tcp_pose must have 7 values, got {len(values)}")
    R = quat_wxyz_to_matrix(values[3:7])
    return transform_from_rt(R, values[:3])


def invert_transform(T: np.ndarray) -> np.ndarray:
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4):
        raise ValueError(f"Transform must be 4x4, got {T.shape}")
    R = T[:3, :3]
    t = T[:3, 3]
    inv = np.eye(4, dtype=float)
    inv[:3, :3] = R.T
    inv[:3, 3] = -R.T @ t
    return inv


def rotation_angle_deg(R: np.ndarray) -> float:
    R = np.asarray(R, dtype=float).reshape(3, 3)
    cos_angle = (float(np.trace(R)) - 1.0) / 2.0
    cos_angle = max(-1.0, min(1.0, cos_angle))
    return math.degrees(math.acos(cos_angle))


def mean_rotation_matrix(rotations: list[np.ndarray]) -> np.ndarray:
    if not rotations:
        raise ValueError("Cannot average an empty rotation list")
    accum = np.zeros((4, 4), dtype=float)
    ref = None
    for R in rotations:
        q = matrix_to_quat_wxyz(R)
        if ref is None:
            ref = q
        elif float(np.dot(ref, q)) < 0:
            q = -q
        accum += np.outer(q, q)
    eigvals, eigvecs = np.linalg.eigh(accum)
    q_mean = eigvecs[:, int(np.argmax(eigvals))]
    if q_mean[0] < 0:
        q_mean = -q_mean
    return quat_wxyz_to_matrix(q_mean)


def transform_record(T: np.ndarray, from_frame: str, to_frame: str) -> dict[str, Any]:
    """Represent a transform that maps coordinates from `to_frame` into `from_frame`."""
    T = np.asarray(T, dtype=float)
    return {
        "from_frame": from_frame,
        "to_frame": to_frame,
        "convention": "matrix maps homogeneous coordinates from to_frame into from_frame",
        "matrix": matrix_to_list(T),
        "translation_m": vector_to_list(T[:3, 3]),
        "quaternion_wxyz": vector_to_list(matrix_to_quat_wxyz(T[:3, :3])),
    }


# ---------------------------------------------------------------------------
# Board creation and ChArUco detection


def load_board_config(path: Path) -> dict[str, Any]:
    cfg = read_yaml(path)
    required = [
        "name",
        "squares_x",
        "squares_y",
        "square_length_m",
        "marker_length_m",
        "dictionary",
        "legacy_pattern",
        "ids_start",
    ]
    missing = [key for key in required if key not in cfg]
    if missing:
        raise ValueError(f"{path} is missing required key(s): {', '.join(missing)}")
    cfg = dict(cfg)
    cfg["path"] = str(path.resolve())
    cfg["squares_x"] = int(cfg["squares_x"])
    cfg["squares_y"] = int(cfg["squares_y"])
    cfg["square_length_m"] = float(cfg["square_length_m"])
    cfg["marker_length_m"] = float(cfg["marker_length_m"])
    cfg["legacy_pattern"] = bool(cfg["legacy_pattern"])
    cfg["ids_start"] = int(cfg["ids_start"])
    return cfg


def board_config_for_yaml(cfg: dict[str, Any]) -> dict[str, Any]:
    return {
        "name": cfg["name"],
        "path": relative_to_repo(Path(cfg["path"])),
        "squares_x": int(cfg["squares_x"]),
        "squares_y": int(cfg["squares_y"]),
        "square_length_m": float(cfg["square_length_m"]),
        "marker_length_m": float(cfg["marker_length_m"]),
        "dictionary": cfg["dictionary"],
        "legacy_pattern": bool(cfg["legacy_pattern"]),
        "ids_start": int(cfg["ids_start"]),
    }


def aruco_dictionary(cv2, dictionary_name: str):
    if not hasattr(cv2.aruco, dictionary_name):
        raise ValueError(f"OpenCV has no ArUco dictionary named {dictionary_name!r}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))


def set_legacy_pattern_if_supported(board, legacy_pattern: bool) -> None:
    if hasattr(board, "setLegacyPattern"):
        board.setLegacyPattern(bool(legacy_pattern))


def board_ids(board) -> np.ndarray:
    if hasattr(board, "getIds"):
        ids = board.getIds()
    elif hasattr(board, "ids"):
        ids = board.ids
    else:
        raise RuntimeError("Cannot read marker IDs from OpenCV ChArUco board")
    return np.asarray(ids, dtype=np.int32).reshape(-1, 1)


def create_charuco_board(cv2, cfg: dict[str, Any]):
    dictionary = aruco_dictionary(cv2, str(cfg["dictionary"]))
    aruco = cv2.aruco
    squares = (int(cfg["squares_x"]), int(cfg["squares_y"]))
    square_length = float(cfg["square_length_m"])
    marker_length = float(cfg["marker_length_m"])
    legacy_pattern = bool(cfg["legacy_pattern"])

    def make_board(ids=None):
        if hasattr(aruco, "CharucoBoard"):
            if ids is None:
                return aruco.CharucoBoard(squares, square_length, marker_length, dictionary)
            return aruco.CharucoBoard(squares, square_length, marker_length, dictionary, ids)
        if hasattr(aruco, "CharucoBoard_create"):
            board = aruco.CharucoBoard_create(
                squares[0], squares[1], square_length, marker_length, dictionary
            )
            if ids is not None:
                if hasattr(board, "ids"):
                    board.ids = np.asarray(ids, dtype=np.int32).reshape(-1, 1)
                else:
                    raise RuntimeError(
                        "This OpenCV build cannot assign non-default ChArUco marker IDs"
                    )
            return board
        raise RuntimeError(
            "OpenCV ArUco module does not expose CharucoBoard. "
            "Install opencv-contrib-python."
        )

    board = make_board()
    set_legacy_pattern_if_supported(board, legacy_pattern)

    ids_start = int(cfg["ids_start"])
    ids = board_ids(board)
    desired = np.arange(ids_start, ids_start + len(ids), dtype=np.int32).reshape(-1, 1)
    if not np.array_equal(ids, desired):
        board = make_board(desired)
        set_legacy_pattern_if_supported(board, legacy_pattern)
        actual = board_ids(board)
        if not np.array_equal(actual, desired):
            raise RuntimeError(
                "Could not configure ChArUco marker IDs from "
                f"{ids_start} to {ids_start + len(ids) - 1}"
            )
    return board, dictionary


def detector_parameters(cv2):
    if hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        return cv2.aruco.DetectorParameters_create()
    return None


def detect_markers(cv2, gray: np.ndarray, dictionary):
    params = detector_parameters(cv2)
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        return detector.detectMarkers(gray)
    if params is None:
        return cv2.aruco.detectMarkers(gray, dictionary)
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=params)


def detect_charuco_corners(
    cv2,
    gray: np.ndarray,
    board,
    dictionary,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
):
    """Return ChArUco corners/IDs and marker corners/IDs across OpenCV APIs."""
    if hasattr(cv2.aruco, "CharucoDetector"):
        charuco_params = (
            cv2.aruco.CharucoParameters()
            if hasattr(cv2.aruco, "CharucoParameters")
            else None
        )
        if charuco_params is not None:
            charuco_params.cameraMatrix = camera_matrix
            charuco_params.distCoeffs = dist_coeffs
        detector_params = detector_parameters(cv2)
        try:
            detector = cv2.aruco.CharucoDetector(
                board, charuco_params, detector_params
            )
        except TypeError:
            detector = cv2.aruco.CharucoDetector(board)
            if charuco_params is not None and hasattr(detector, "setCharucoParameters"):
                detector.setCharucoParameters(charuco_params)
            if detector_params is not None and hasattr(detector, "setDetectorParameters"):
                detector.setDetectorParameters(detector_params)
        return detector.detectBoard(gray)

    marker_corners, marker_ids, rejected = detect_markers(cv2, gray, dictionary)
    if marker_ids is None or len(marker_ids) == 0:
        return None, None, marker_corners, marker_ids

    if hasattr(cv2.aruco, "refineDetectedMarkers"):
        try:
            refined = cv2.aruco.refineDetectedMarkers(
                gray,
                board,
                marker_corners,
                marker_ids,
                rejected,
                camera_matrix,
                dist_coeffs,
            )
            marker_corners, marker_ids = refined[0], refined[1]
        except Exception:
            pass

    try:
        _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            marker_corners, marker_ids, gray, board, camera_matrix, dist_coeffs
        )
    except TypeError:
        _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            marker_corners, marker_ids, gray, board
        )
    return charuco_corners, charuco_ids, marker_corners, marker_ids


def board_chessboard_corners(board) -> np.ndarray:
    if hasattr(board, "getChessboardCorners"):
        corners = board.getChessboardCorners()
    elif hasattr(board, "chessboardCorners"):
        corners = board.chessboardCorners
    else:
        raise RuntimeError("Cannot read ChArUco chessboard corners from board")
    return np.asarray(corners, dtype=float).reshape(-1, 3)


def reprojection_error_px(
    cv2,
    board,
    charuco_corners: np.ndarray,
    charuco_ids: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> float:
    object_points_all = board_chessboard_corners(board)
    ids = np.asarray(charuco_ids, dtype=int).reshape(-1)
    valid = (ids >= 0) & (ids < len(object_points_all))
    if not np.all(valid):
        ids = ids[valid]
        charuco_corners = np.asarray(charuco_corners)[valid]
    if len(ids) == 0:
        return float("nan")

    object_points = object_points_all[ids].reshape(-1, 1, 3)
    image_points = np.asarray(charuco_corners, dtype=float).reshape(-1, 1, 2)
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, dist_coeffs
    )
    errors = np.linalg.norm(projected.reshape(-1, 2) - image_points.reshape(-1, 2), axis=1)
    return float(np.sqrt(np.mean(errors * errors)))


def estimate_charuco_pose(
    cv2,
    image: np.ndarray,
    board,
    dictionary,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    *,
    min_charuco_corners: int,
) -> CharucoPose:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    charuco_corners, charuco_ids, marker_corners, marker_ids = detect_charuco_corners(
        cv2, gray, board, dictionary, camera_matrix, dist_coeffs
    )
    if marker_ids is None or len(marker_ids) == 0:
        raise ValueError("no ArUco markers detected")

    count = 0 if charuco_ids is None else int(len(charuco_ids))
    if charuco_ids is None or charuco_corners is None or count < min_charuco_corners:
        raise ValueError(
            f"only {count} ChArUco corners detected; need at least "
            f"{min_charuco_corners}"
        )

    object_points_all = board_chessboard_corners(board)
    ids = np.asarray(charuco_ids, dtype=int).reshape(-1)
    object_points = object_points_all[ids].reshape(-1, 1, 3)
    image_points = np.asarray(charuco_corners, dtype=float).reshape(-1, 1, 2)
    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise ValueError("OpenCV solvePnP could not estimate the ChArUco board pose")

    R, _ = cv2.Rodrigues(rvec)
    T_camera_board = transform_from_rt(R, np.asarray(tvec, dtype=float).reshape(3))
    reproj = reprojection_error_px(
        cv2,
        board,
        charuco_corners,
        charuco_ids,
        rvec,
        tvec,
        camera_matrix,
        dist_coeffs,
    )
    return CharucoPose(
        T_camera_board=T_camera_board,
        rvec=np.asarray(rvec, dtype=float).reshape(3, 1),
        tvec=np.asarray(tvec, dtype=float).reshape(3, 1),
        marker_count=int(len(marker_ids)),
        charuco_count=count,
        reprojection_error_px=reproj,
    )


# ---------------------------------------------------------------------------
# RealSense capture


def realsense_devices(rs) -> list[dict[str, str]]:
    devices = []
    for device in rs.context().query_devices():
        devices.append(
            {
                "name": device.get_info(rs.camera_info.name),
                "serial": device.get_info(rs.camera_info.serial_number),
                "firmware": device.get_info(rs.camera_info.firmware_version),
            }
        )
    return devices


def select_camera(devices: list[dict[str, str]], requested_serial: str | None = None):
    if requested_serial:
        for device in devices:
            if device["serial"] == requested_serial:
                return device
        return None
    for device in devices:
        if "D405" in device["name"]:
            return device
    return devices[0] if devices else None


def find_color_sensor(rs, profile):
    for sensor in profile.get_device().query_sensors():
        if (
            sensor.supports(rs.option.white_balance)
            or sensor.supports(rs.option.enable_auto_white_balance)
            or sensor.supports(rs.option.exposure)
        ):
            return sensor
    return None


def set_sensor_option(rs, sensor, option, value, label: str) -> None:
    if value is None:
        return
    if not sensor.supports(option):
        print(f"Color sensor does not support {label}.")
        return

    option_range = sensor.get_option_range(option)
    if value < option_range.min or value > option_range.max:
        print(
            f"Warning: {label}={value} is outside supported range "
            f"{option_range.min:g}..{option_range.max:g}"
        )

    sensor.set_option(option, float(value))
    actual = sensor.get_option(option)
    print(f"Set {label}: {actual:g}")


def configure_color_sensor(
    rs,
    profile,
    *,
    exposure: float | None,
    gain: float | None,
    white_balance: float | None,
) -> None:
    color_sensor = find_color_sensor(rs, profile)
    if color_sensor is None:
        print("Could not find a configurable color sensor.")
        return

    if exposure is not None and color_sensor.supports(rs.option.enable_auto_exposure):
        color_sensor.set_option(rs.option.enable_auto_exposure, 0)
        print("Set auto exposure: off")
    set_sensor_option(rs, color_sensor, rs.option.exposure, exposure, "exposure")
    set_sensor_option(rs, color_sensor, rs.option.gain, gain, "gain")

    if white_balance is not None and color_sensor.supports(
        rs.option.enable_auto_white_balance
    ):
        color_sensor.set_option(rs.option.enable_auto_white_balance, 0)
        print("Set auto white balance: off")
    set_sensor_option(
        rs, color_sensor, rs.option.white_balance, white_balance, "white balance"
    )


def intrinsics_to_yaml(intrinsics) -> dict[str, Any]:
    camera_matrix = [
        [float(intrinsics.fx), 0.0, float(intrinsics.ppx)],
        [0.0, float(intrinsics.fy), float(intrinsics.ppy)],
        [0.0, 0.0, 1.0],
    ]
    coeffs = [float(v) for v in intrinsics.coeffs]
    return {
        "width": int(intrinsics.width),
        "height": int(intrinsics.height),
        "fx": float(intrinsics.fx),
        "fy": float(intrinsics.fy),
        "ppx": float(intrinsics.ppx),
        "ppy": float(intrinsics.ppy),
        "model": str(intrinsics.model),
        "coeffs": coeffs,
        "camera_matrix": camera_matrix,
        "dist_coeffs": coeffs,
    }


def capture_realsense_color(
    *,
    serial: str | None,
    width: int,
    height: int,
    fps: int,
    warmup_frames: int,
    exposure: float | None,
    gain: float | None,
    white_balance: float | None,
):
    rs = require_realsense()
    devices = realsense_devices(rs)
    if not devices:
        raise SystemExit("No Intel RealSense camera detected.")

    camera = select_camera(devices, serial)
    if camera is None:
        raise SystemExit(f"No RealSense camera found with serial number: {serial}")

    print(
        "Using camera: {name} | serial: {serial} | firmware: {firmware}".format(
            **camera
        )
    )

    with temporarily_release_shared_camera(camera["serial"]):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(camera["serial"])
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        try:
            profile = pipeline.start(config)
        except RuntimeError as exc:
            raise SystemExit(
                f"Could not start camera stream: {exc}\n"
                "Try a different resolution/FPS, for example: "
                "--width 640 --height 480 --fps 30"
            ) from exc

        configure_color_sensor(
            rs,
            profile,
            exposure=exposure,
            gain=gain,
            white_balance=white_balance,
        )

        try:
            stream_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
            intrinsics = intrinsics_to_yaml(stream_profile.get_intrinsics())
            color_frame = None
            for _ in range(max(1, warmup_frames)):
                frames = pipeline.wait_for_frames(timeout_ms=5000)
                color_frame = frames.get_color_frame()

            if not color_frame:
                raise SystemExit("Camera did not return a color frame.")

            image = np.asanyarray(color_frame.get_data()).copy()
            return image, intrinsics, camera
        finally:
            pipeline.stop()


# ---------------------------------------------------------------------------
# Robot capture


def read_current_tcp_pose(robot_sn: str, operational_timeout_s: float) -> list[float]:
    RobotSession = import_robot_session()
    logger = make_logger()
    with RobotSession(
        robot_sn, logger=logger, operational_timeout_s=operational_timeout_s
    ) as session:
        _, state = session.selected_arm_state()
        tcp = [float(v) for v in getattr(state, "tcp_pose", [])]
        if len(tcp) != 7:
            raise RuntimeError(
                f"current arm state tcp_pose has {len(tcp)} values; expected 7"
            )
        return tcp


# ---------------------------------------------------------------------------
# Sample loading and solving


def camera_matrix_and_dist(metadata: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    intr = metadata.get("camera", {}).get("intrinsics", {})
    if "camera_matrix" in intr:
        K = np.asarray(intr["camera_matrix"], dtype=float).reshape(3, 3)
    else:
        K = np.array(
            [
                [float(intr["fx"]), 0.0, float(intr["ppx"])],
                [0.0, float(intr["fy"]), float(intr["ppy"])],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
    dist = np.asarray(intr.get("dist_coeffs", intr.get("coeffs", [])), dtype=float)
    return K, dist.reshape(-1, 1)


def sample_image_path(sample_yaml: Path, metadata: dict[str, Any]) -> Path:
    rel = metadata.get("files", {}).get("color_image", "color.png")
    return (sample_yaml.parent / rel).resolve()


def sample_tcp_pose(metadata: dict[str, Any]) -> list[float]:
    robot = metadata.get("robot", {})
    if "tcp_pose_world" in robot and isinstance(robot["tcp_pose_world"], dict):
        values = robot["tcp_pose_world"].get("values", [])
    else:
        values = robot.get("tcp_pose", [])
    pose = [float(v) for v in values]
    if len(pose) != 7:
        raise ValueError(f"sample tcp_pose has {len(pose)} values; expected 7")
    return pose


def iter_sample_yamls(samples_dir: Path) -> list[Path]:
    return sorted(path for path in samples_dir.glob("*/sample.yaml") if path.is_file())


def detect_sample(
    cv2,
    sample_yaml: Path,
    board,
    dictionary,
    *,
    min_charuco_corners: int,
) -> SampleDetection:
    metadata = read_yaml(sample_yaml)
    image_path = sample_image_path(sample_yaml, metadata)
    image = cv2.imread(str(image_path))
    if image is None:
        raise ValueError(f"could not read image {image_path}")

    camera_matrix, dist_coeffs = camera_matrix_and_dist(metadata)
    pose = estimate_charuco_pose(
        cv2,
        image,
        board,
        dictionary,
        camera_matrix,
        dist_coeffs,
        min_charuco_corners=min_charuco_corners,
    )
    return SampleDetection(
        sample_dir=sample_yaml.parent,
        sample_yaml=sample_yaml,
        image_path=image_path,
        metadata=metadata,
        T_world_tcp=pose_vec_to_transform(sample_tcp_pose(metadata)),
        T_camera_board=pose.T_camera_board,
        marker_count=pose.marker_count,
        charuco_count=pose.charuco_count,
        reprojection_error_px=pose.reprojection_error_px,
    )


def collect_detections(
    cv2,
    samples_dir: Path,
    board,
    dictionary,
    *,
    min_charuco_corners: int,
) -> tuple[list[SampleDetection], list[dict[str, str]]]:
    detections: list[SampleDetection] = []
    rejected: list[dict[str, str]] = []

    sample_yamls = iter_sample_yamls(samples_dir)
    if not sample_yamls:
        raise SystemExit(f"No samples found in {samples_dir}")

    for sample_yaml in sample_yamls:
        try:
            detection = detect_sample(
                cv2,
                sample_yaml,
                board,
                dictionary,
                min_charuco_corners=min_charuco_corners,
            )
            detections.append(detection)
            print(
                f"accepted {sample_yaml.parent.name}: "
                f"{detection.charuco_count} corners, "
                f"reproj={detection.reprojection_error_px:.3f}px"
            )
        except Exception as exc:
            rejected.append(
                {
                    "sample": relative_to_repo(sample_yaml.parent),
                    "reason": str(exc),
                }
            )
            print(f"rejected {sample_yaml.parent.name}: {exc}")
    return detections, rejected


def hand_eye_method_value(cv2, method_name: str) -> int:
    attr = HAND_EYE_METHODS[method_name]
    if not hasattr(cv2, attr):
        raise ValueError(f"This OpenCV build does not expose cv2.{attr}")
    return int(getattr(cv2, attr))


def solve_hand_eye_from_transforms(
    cv2,
    T_world_tcp_list: list[np.ndarray],
    T_camera_board_list: list[np.ndarray],
    *,
    method: str = "tsai",
) -> np.ndarray:
    if len(T_world_tcp_list) != len(T_camera_board_list):
        raise ValueError("Robot and board pose lists must have the same length")
    if len(T_world_tcp_list) < 3:
        raise ValueError("At least 3 accepted samples are required for hand-eye solve")

    R_gripper2base = [np.asarray(T[:3, :3], dtype=float) for T in T_world_tcp_list]
    t_gripper2base = [
        np.asarray(T[:3, 3], dtype=float).reshape(3, 1) for T in T_world_tcp_list
    ]
    R_target2cam = [np.asarray(T[:3, :3], dtype=float) for T in T_camera_board_list]
    t_target2cam = [
        np.asarray(T[:3, 3], dtype=float).reshape(3, 1) for T in T_camera_board_list
    ]

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base,
        t_gripper2base,
        R_target2cam,
        t_target2cam,
        method=hand_eye_method_value(cv2, method),
    )
    return transform_from_rt(R_cam2gripper, np.asarray(t_cam2gripper).reshape(3))


def pose_spread_stats(T_world_board_list: list[np.ndarray]) -> dict[str, Any]:
    if not T_world_board_list:
        raise ValueError("No board poses available for validation")

    translations = np.array([T[:3, 3] for T in T_world_board_list], dtype=float)
    mean_translation = translations.mean(axis=0)
    translation_deviation_mm = np.linalg.norm(
        translations - mean_translation.reshape(1, 3), axis=1
    ) * 1000.0

    mean_R = mean_rotation_matrix([T[:3, :3] for T in T_world_board_list])
    rotation_deviation_deg = np.array(
        [rotation_angle_deg(mean_R.T @ T[:3, :3]) for T in T_world_board_list],
        dtype=float,
    )

    return {
        "sample_count": len(T_world_board_list),
        "mean_board_translation_world_m": vector_to_list(mean_translation),
        "translation_spread_mm": {
            "mean": float(np.mean(translation_deviation_mm)),
            "std": float(np.std(translation_deviation_mm)),
            "max": float(np.max(translation_deviation_mm)),
            "per_sample": vector_to_list(translation_deviation_mm),
        },
        "rotation_spread_deg": {
            "mean": float(np.mean(rotation_deviation_deg)),
            "std": float(np.std(rotation_deviation_deg)),
            "max": float(np.max(rotation_deviation_deg)),
            "per_sample": vector_to_list(rotation_deviation_deg),
        },
    }


def validation_stats(
    detections: list[SampleDetection], T_tcp_camera: np.ndarray
) -> dict[str, Any]:
    T_world_board = [
        det.T_world_tcp @ T_tcp_camera @ det.T_camera_board for det in detections
    ]
    stats = pose_spread_stats(T_world_board)
    reproj = np.array([det.reprojection_error_px for det in detections], dtype=float)
    stats["reprojection_error_px"] = {
        "mean": float(np.mean(reproj)),
        "std": float(np.std(reproj)),
        "max": float(np.max(reproj)),
        "per_sample": vector_to_list(reproj),
    }
    return stats


def accepted_sample_records(detections: list[SampleDetection]) -> list[dict[str, Any]]:
    return [
        {
            "sample": relative_to_repo(det.sample_dir),
            "image": relative_to_repo(det.image_path),
            "marker_count": int(det.marker_count),
            "charuco_count": int(det.charuco_count),
            "reprojection_error_px": float(det.reprojection_error_px),
        }
        for det in detections
    ]


def first_camera_record(detections: list[SampleDetection]) -> dict[str, Any]:
    camera = dict(detections[0].metadata.get("camera", {}))
    if "intrinsics" in camera:
        intr = dict(camera["intrinsics"])
        camera["intrinsics"] = intr
    return camera


def print_quality(stats: dict[str, Any]) -> None:
    t = stats["translation_spread_mm"]
    r = stats["rotation_spread_deg"]
    reproj = stats.get("reprojection_error_px")
    print("\nValidation quality:")
    print(
        f"  fixed-board translation spread: mean={t['mean']:.2f} mm, "
        f"std={t['std']:.2f} mm, max={t['max']:.2f} mm"
    )
    print(
        f"  fixed-board rotation spread:    mean={r['mean']:.3f} deg, "
        f"std={r['std']:.3f} deg, max={r['max']:.3f} deg"
    )
    if reproj:
        print(
            f"  ChArUco reprojection error:     mean={reproj['mean']:.3f} px, "
            f"std={reproj['std']:.3f} px, max={reproj['max']:.3f} px"
        )


def camera_axis_check(
    T_tcp_camera: np.ndarray,
    expected_axis_tcp: Iterable[float],
    max_angle_deg: float,
) -> dict[str, Any]:
    expected = np.asarray(list(expected_axis_tcp), dtype=float).reshape(3)
    norm = np.linalg.norm(expected)
    if norm == 0:
        raise ValueError("expected camera axis vector cannot be zero")
    expected = expected / norm

    optical_z_tcp = T_tcp_camera[:3, :3] @ np.array([0.0, 0.0, 1.0])
    optical_z_tcp = optical_z_tcp / np.linalg.norm(optical_z_tcp)
    dot = float(np.clip(np.dot(optical_z_tcp, expected), -1.0, 1.0))
    angle = math.degrees(math.acos(dot))
    return {
        "camera_optical_z_in_tcp": vector_to_list(optical_z_tcp),
        "expected_axis_tcp": vector_to_list(expected),
        "angle_deg": float(angle),
        "max_angle_deg": float(max_angle_deg),
        "within_threshold": bool(angle <= max_angle_deg),
    }


def print_axis_check(axis: dict[str, Any]) -> None:
    vec = axis["camera_optical_z_in_tcp"]
    print(
        "\nCamera optical +Z in TCP frame: "
        f"[{vec[0]:+.3f}, {vec[1]:+.3f}, {vec[2]:+.3f}] "
        f"({axis['angle_deg']:.1f} deg from expected)"
    )
    if not axis["within_threshold"]:
        print(
            "Warning: camera optical +Z is farther from the expected TCP direction "
            f"than {axis['max_angle_deg']:.1f} deg."
        )


# ---------------------------------------------------------------------------
# Commands


def save_calibration_sample(
    cv2,
    *,
    samples_dir: Path,
    sample_id: str | None,
    image: np.ndarray,
    intrinsics: dict[str, Any],
    camera: dict[str, str],
    tcp_pose: Iterable[float],
    board_cfg: dict[str, Any],
    robot_sn: str,
) -> Path:
    sample_id = sample_id or timestamp_slug()
    sample_dir = samples_dir / sample_id
    suffix = 1
    while sample_dir.exists():
        sample_dir = samples_dir / f"{sample_id}_{suffix:02d}"
        suffix += 1
    sample_dir.mkdir(parents=True)

    image_path = sample_dir / "color.png"
    if not cv2.imwrite(str(image_path), image):
        raise RuntimeError(f"Could not save image to {image_path}")

    sample = {
        "schema_version": 1,
        "sample_id": sample_dir.name,
        "captured_at": utc_timestamp(),
        "files": {"color_image": "color.png"},
        "camera": {
            "name": camera["name"],
            "serial": camera["serial"],
            "firmware": camera["firmware"],
            "frame": "realsense_color_optical",
            "image_size": [int(image.shape[1]), int(image.shape[0])],
            "intrinsics": intrinsics,
        },
        "robot": {
            "serial": robot_sn,
            "tcp_pose_world": {
                "order": list(TCP_POSE_ORDER),
                "values": vector_to_list(tcp_pose),
            },
        },
        "board": board_config_for_yaml(board_cfg),
    }
    write_yaml(sample, sample_dir / "sample.yaml")
    return sample_dir


def command_capture(args: argparse.Namespace) -> int:
    cv2 = require_cv2()
    board_cfg = load_board_config(Path(args.board))

    if args.prompt:
        input(
            "Move the robot/camera to a calibration view with the fixed board visible, "
            "then press Enter to capture ... "
        )

    tcp_pose = read_current_tcp_pose(args.robot_sn, args.operational_timeout_s)
    print(
        "Current TCP pose: "
        + ", ".join(f"{name}={value:.6f}" for name, value in zip(TCP_POSE_ORDER, tcp_pose))
    )

    image, intrinsics, camera = capture_realsense_color(
        serial=args.serial,
        width=args.width,
        height=args.height,
        fps=args.fps,
        warmup_frames=args.warmup_frames,
        exposure=args.exposure,
        gain=args.gain,
        white_balance=args.white_balance,
    )

    sample_dir = save_calibration_sample(
        cv2,
        samples_dir=Path(args.samples_dir),
        sample_id=args.sample_id,
        image=image,
        intrinsics=intrinsics,
        camera=camera,
        tcp_pose=tcp_pose,
        board_cfg=board_cfg,
        robot_sn=args.robot_sn,
    )
    print(f"\nSaved sample -> {sample_dir}")
    return 0


FLOATING_DAMPING_LEVEL = [0.0] * 7
FLOATING_RESPONSE_TORQUE = [1.5, 1.5, 1.5, 1.5, 0.5, 0.5, 0.3]
FLOATING_LOAD_COMPENSATION_SCALE = 1.2


def _sized_joint_vector(values: list[float], dof: int) -> list[float]:
    values = [float(v) for v in values]
    if len(values) >= dof:
        return values[:dof]
    return values + [values[-1] if values else 0.0] * (dof - len(values))


def enable_joint_floating(session) -> int:
    """Activate the FloatingJoint primitive so all joints can be hand-guided.

    Mirrors ``start_joint_floating`` in project/record_robot_waypoints.py.
    """
    _, state = session.selected_arm_state()
    q = list(getattr(state, "q", []))
    if q:
        dof = len(q)
    else:
        dof = int(session.robot.info().DoF)

    session.switch_mode("NRT_PRIMITIVE_EXECUTION")
    session.execute_primitive(
        "FloatingJoint",
        {
            "floatingJoint": [1.0] * dof,
            "dampingLevel": _sized_joint_vector(FLOATING_DAMPING_LEVEL, dof),
            "responseTorque": [
                v * FLOATING_LOAD_COMPENSATION_SCALE
                for v in _sized_joint_vector(FLOATING_RESPONSE_TORQUE, dof)
            ],
            "diEnableFloating": "NONE",
        },
    )
    return dof


def command_preview(args: argparse.Namespace) -> int:
    cv2 = require_cv2()
    rs = require_realsense()

    board_cfg = load_board_config(Path(args.board))
    board = None
    dictionary = None
    if not args.no_detect:
        board, dictionary = create_charuco_board(cv2, board_cfg)

    devices = realsense_devices(rs)
    if not devices:
        raise SystemExit("No Intel RealSense camera detected.")
    camera = select_camera(devices, args.serial)
    if camera is None:
        raise SystemExit(f"No RealSense camera found with serial number: {args.serial}")
    print(
        "Using camera: {name} | serial: {serial} | firmware: {firmware}".format(
            **camera
        )
    )

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(camera["serial"])
    config.enable_stream(
        rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps
    )
    profile = pipeline.start(config)
    configure_color_sensor(
        rs,
        profile,
        exposure=args.exposure,
        gain=args.gain,
        white_balance=args.white_balance,
    )

    stream_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intrinsics_yaml = intrinsics_to_yaml(stream_profile.get_intrinsics())
    camera_matrix = np.asarray(intrinsics_yaml["camera_matrix"], dtype=float)
    dist_coeffs = np.asarray(intrinsics_yaml["dist_coeffs"], dtype=float).reshape(-1, 1)

    samples_dir = Path(args.samples_dir)
    capture_key_char = (args.capture_key or "c")[0].lower()
    capture_key_code = ord(capture_key_char)

    session = None
    session_cm = None
    if not args.no_robot:
        RobotSession = import_robot_session()
        logger = make_logger()
        session_cm = RobotSession(
            args.robot_sn,
            logger=logger,
            operational_timeout_s=args.operational_timeout_s,
        )
        try:
            session = session_cm.__enter__()
            if args.hold_position:
                print(
                    f"Robot session open: {args.robot_sn} (holding position; "
                    "use the wrist Lead-Through button to move by hand)"
                )
            else:
                try:
                    dof = enable_joint_floating(session)
                    print(
                        f"Robot session open: {args.robot_sn} "
                        f"(FloatingJoint primitive active on {dof} joints; "
                        "arm is free to hand-guide)"
                    )
                except Exception as exc:
                    print(
                        f"Robot session open: {args.robot_sn} (could not start "
                        f"FloatingJoint: {exc}; arm will hold position. Use the "
                        "wrist Lead-Through button to move it.)"
                    )
        except Exception as exc:
            session_cm = None
            print(f"Robot session unavailable ({exc}); capture key disabled.")

    capture_help = (
        f"'{capture_key_char}'=capture sample"
        if session is not None
        else f"'{capture_key_char}'=capture (DISABLED, no robot)"
    )
    print(f"Live preview: 'q'/ESC=quit  's'=snapshot  {capture_help}")
    win = "RealSense preview"

    flash_text = ""
    flash_color = (0, 200, 0)
    flash_frames = 0
    try:
        while True:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            raw_image = np.asanyarray(color_frame.get_data()).copy()
            display = raw_image.copy()

            status = "detect disabled"
            status_color = (200, 200, 200)
            if board is not None:
                try:
                    gray = cv2.cvtColor(raw_image, cv2.COLOR_BGR2GRAY)
                    (
                        charuco_corners,
                        charuco_ids,
                        marker_corners,
                        marker_ids,
                    ) = detect_charuco_corners(
                        cv2, gray, board, dictionary, camera_matrix, dist_coeffs
                    )
                    marker_count = 0 if marker_ids is None else int(len(marker_ids))
                    charuco_count = 0 if charuco_ids is None else int(len(charuco_ids))
                    if marker_count > 0:
                        cv2.aruco.drawDetectedMarkers(
                            display, marker_corners, marker_ids
                        )
                    if charuco_count > 0:
                        cv2.aruco.drawDetectedCornersCharuco(
                            display, charuco_corners, charuco_ids
                        )
                    status = (
                        f"markers={marker_count}  "
                        f"charuco={charuco_count}/min{args.min_charuco_corners}"
                    )
                    status_color = (
                        (0, 200, 0)
                        if charuco_count >= args.min_charuco_corners
                        else (0, 0, 255)
                    )
                except Exception as exc:
                    status = f"detect error: {exc}"
                    status_color = (0, 0, 255)

            draw_text(cv2, display, status, (10, 25), status_color)
            if flash_frames > 0:
                draw_text(
                    cv2,
                    display,
                    flash_text,
                    (10, display.shape[0] - 15),
                    flash_color,
                )
                flash_frames -= 1

            cv2.imshow(win, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                snap_path = Path(args.snapshot_dir) / f"preview_{timestamp_slug()}.png"
                snap_path.parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(snap_path), display)
                print(f"saved snapshot -> {snap_path}")
                flash_text = f"SNAPSHOT: {snap_path.name}"
                flash_color = (0, 200, 200)
                flash_frames = 30
            elif key == capture_key_code:
                if session is None:
                    flash_text = "CAPTURE DISABLED (no robot session)"
                    flash_color = (0, 0, 255)
                    flash_frames = 30
                    continue
                try:
                    _, state = session.selected_arm_state()
                    tcp_pose = [float(v) for v in getattr(state, "tcp_pose", [])]
                    if len(tcp_pose) != 7:
                        raise RuntimeError(
                            f"tcp_pose has {len(tcp_pose)} values; expected 7"
                        )
                    sample_dir = save_calibration_sample(
                        cv2,
                        samples_dir=samples_dir,
                        sample_id=args.sample_id_prefix,
                        image=raw_image,
                        intrinsics=intrinsics_yaml,
                        camera=camera,
                        tcp_pose=tcp_pose,
                        board_cfg=board_cfg,
                        robot_sn=args.robot_sn,
                    )
                    msg = f"saved sample -> {sample_dir}"
                    print(msg)
                    flash_text = f"SAVED: {sample_dir.name}"
                    flash_color = (0, 200, 0)
                    flash_frames = 30
                except Exception as exc:
                    print(f"capture failed: {exc}")
                    flash_text = f"CAPTURE FAILED: {exc}"
                    flash_color = (0, 0, 255)
                    flash_frames = 30
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        if session_cm is not None:
            try:
                session_cm.__exit__(None, None, None)
            except Exception as exc:
                print(f"warning: robot session cleanup raised: {exc}")
    return 0


def draw_text(cv2, image, text: str, origin, color) -> None:
    cv2.putText(
        image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA
    )
    cv2.putText(
        image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 1, cv2.LINE_AA
    )


def parse_approach_axis(spec: str) -> np.ndarray:
    spec = spec.strip().lower().replace(" ", "")
    sign = 1.0
    if spec.startswith(("+", "-")):
        sign = -1.0 if spec[0] == "-" else 1.0
        spec = spec[1:]
    if spec not in ("x", "y", "z"):
        raise ValueError(
            f"approach axis must be one of +x,-x,+y,-y,+z,-z (got {spec!r})"
        )
    axis = {"x": np.array([1.0, 0.0, 0.0]),
            "y": np.array([0.0, 1.0, 0.0]),
            "z": np.array([0.0, 0.0, 1.0])}[spec]
    return sign * axis


def rotation_align_unit_vectors(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Shortest 3D rotation R such that R @ a = b (Rodrigues)."""
    a = np.asarray(a, dtype=float); a = a / np.linalg.norm(a)
    b = np.asarray(b, dtype=float); b = b / np.linalg.norm(b)
    c = float(np.dot(a, b))
    if c > 1.0 - 1e-9:
        return np.eye(3)
    if c < -1.0 + 1e-9:
        # 180 deg rotation around any axis perpendicular to a
        seed = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        perp = seed - np.dot(seed, a) * a
        perp = perp / np.linalg.norm(perp)
        K = np.array([[0.0, -perp[2], perp[1]],
                      [perp[2], 0.0, -perp[0]],
                      [-perp[1], perp[0], 0.0]])
        return np.eye(3) + 2.0 * (K @ K)
    v = np.cross(a, b)
    K = np.array([[0.0, -v[2], v[1]],
                  [v[2], 0.0, -v[0]],
                  [-v[1], v[0], 0.0]])
    return np.eye(3) + K + (K @ K) * ((1.0 - c) / float(np.dot(v, v)))


def compute_verify_target(
    T_world_board: np.ndarray,
    approach_tcp_unit: np.ndarray,
    distance_m: float,
    camera_pos_world: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return (target_pos_world, target_R_world).

    Standoff is placed on the camera-facing side of the board (the side the
    user can actually see), independent of which way OpenCV chose to point the
    board's local +Z axis for this board. ``camera_pos_world`` is the camera
    origin in world coordinates; its position relative to the board plane
    determines which sign of the normal to use.

    Convention 1 (match board axes): the TCP approach axis is rotated to point
    anti-parallel to the chosen standoff direction (i.e., it points from the
    target back at the board). When approach=+Z and the camera is on the
    +board_Z side, target_R = board_R @ Rx(180); when the camera is on the
    -board_Z side, target_R = board_R (identity local rotation).
    """
    board_pos = np.asarray(T_world_board[:3, 3], dtype=float)
    board_R = np.asarray(T_world_board[:3, :3], dtype=float)
    board_z_world = board_R[:, 2]

    facing = float(np.dot(np.asarray(camera_pos_world) - board_pos, board_z_world))
    facing_sign = 1.0 if facing >= 0.0 else -1.0

    target_pos = board_pos + facing_sign * float(distance_m) * board_z_world

    approach_target_board = np.array([0.0, 0.0, -facing_sign])
    R_local = rotation_align_unit_vectors(approach_tcp_unit, approach_target_board)
    target_R = board_R @ R_local
    return target_pos, target_R


def tcp_pose_from_pos_R(pos: np.ndarray, R: np.ndarray) -> list[float]:
    quat = matrix_to_quat_wxyz(R)
    return [float(pos[0]), float(pos[1]), float(pos[2]),
            float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])]


@dataclass(frozen=True)
class VerifyMotionStep:
    name: str
    primitive: str
    target_tcp: list[float]


def summarize_pose_delta(
    current_tcp: Iterable[float], target_tcp: Iterable[float]
) -> dict[str, Any]:
    current = [float(v) for v in current_tcp]
    target = [float(v) for v in target_tcp]
    if len(current) != 7 or len(target) != 7:
        raise ValueError("summarize_pose_delta expects two 7-element TCP poses")

    current_pos = np.asarray(current[:3], dtype=float)
    target_pos = np.asarray(target[:3], dtype=float)
    delta_world_m = target_pos - current_pos
    delta_world_mm = delta_world_m * 1000.0

    target_R = quat_wxyz_to_matrix(target[3:])
    current_R = quat_wxyz_to_matrix(current[3:])
    delta_target_mm = target_R.T @ delta_world_mm
    drot_deg = rotation_angle_deg(current_R.T @ target_R)

    return {
        "delta_world_mm": vector_to_list(delta_world_mm),
        "delta_target_mm": vector_to_list(delta_target_mm),
        "dpos_mm": float(np.linalg.norm(delta_world_mm)),
        "drot_deg": float(drot_deg),
    }


def build_verify_motion_plan(
    target_tcp: Iterable[float],
    *,
    via_tcp: Iterable[float] | None = None,
    final_primitive: str = "movel",
) -> list[VerifyMotionStep]:
    final_name = final_primitive.strip().lower()
    if final_name not in {"movel", "moveptp"}:
        raise ValueError(
            f"final_primitive must be 'movel' or 'moveptp' (got {final_primitive!r})"
        )

    target = [float(v) for v in target_tcp]
    steps: list[VerifyMotionStep] = []
    if via_tcp is not None:
        via = [float(v) for v in via_tcp]
        if not np.allclose(via, target, atol=1e-9, rtol=0.0):
            steps.append(
                VerifyMotionStep(
                    name="approach_via",
                    primitive="MovePTP",
                    target_tcp=via,
                )
            )
    steps.append(
        VerifyMotionStep(
            name="final_target",
            primitive="MoveL" if final_name == "movel" else "MovePTP",
            target_tcp=target,
        )
    )
    return steps


def execute_verify_motion_step(
    session,
    helpers,
    step: VerifyMotionStep,
    *,
    ref_joints_deg: list[float],
    jnt_vel_scale: int,
    linear_vel: float,
) -> None:
    coord_args = helpers.tcp_pose_to_coord_args(
        step.target_tcp,
        ref_joints_deg=ref_joints_deg,
    )
    target = session.flexivrdk.Coord(*coord_args)
    if step.primitive == "MoveL":
        session.execute_primitive(
            "MoveL",
            {
                "target": target,
                "vel": float(linear_vel),
                "zoneRadius": "ZFine",
            },
        )
    elif step.primitive == "MovePTP":
        session.execute_primitive(
            "MovePTP",
            {
                "target": target,
                "jntVelScale": int(jnt_vel_scale),
                "zoneRadius": "ZFine",
                "targetTolerLevel": 1,
                "jntAccMultiplier": 1.0,
                "enableFixRefJntPos": False,
                "refJntPos": session.flexivrdk.JPos(ref_joints_deg),
            },
        )
    else:
        raise ValueError(f"Unsupported primitive: {step.primitive}")
    session.wait_for_primitive("reachedTarget")


def command_verify(args: argparse.Namespace) -> int:
    cv2 = require_cv2()
    rs = require_realsense()

    calibration = read_yaml(Path(args.calibration))
    T_tcp_camera = np.asarray(calibration["T_tcp_camera"], dtype=float).reshape(4, 4)

    board_cfg = load_board_config(Path(args.board))
    board, dictionary = create_charuco_board(cv2, board_cfg)

    approach_tcp = parse_approach_axis(args.approach_axis)
    distance_m = float(args.distance_mm) / 1000.0

    devices = realsense_devices(rs)
    if not devices:
        raise SystemExit("No Intel RealSense camera detected.")
    camera = select_camera(devices, args.serial)
    if camera is None:
        raise SystemExit(f"No RealSense camera found with serial number: {args.serial}")
    print("Using camera: {name} | serial: {serial}".format(**camera))

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(camera["serial"])
    config.enable_stream(
        rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps
    )
    profile = pipeline.start(config)
    configure_color_sensor(
        rs, profile,
        exposure=args.exposure, gain=args.gain, white_balance=args.white_balance,
    )
    stream_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intrinsics_yaml = intrinsics_to_yaml(stream_profile.get_intrinsics())
    camera_matrix = np.asarray(intrinsics_yaml["camera_matrix"], dtype=float)
    dist_coeffs = np.asarray(intrinsics_yaml["dist_coeffs"], dtype=float).reshape(-1, 1)

    RobotSession = import_robot_session()
    helpers = import_flexiv_helpers()
    logger = make_logger()
    session_cm = RobotSession(
        args.robot_sn, logger=logger,
        operational_timeout_s=args.operational_timeout_s,
    )
    session = session_cm.__enter__()
    try:
        dof = enable_joint_floating(session)
        print(f"Robot session open: {args.robot_sn} (FloatingJoint on {dof} joints)")
    except Exception as exc:
        print(f"FloatingJoint failed ({exc}); arm will hold position.")

    print(
        "verify keys: 'c'=capture+lock target, hand-guide while watching error, "
        "'g'=go with staged motion, 'x'=cancel pending target, 'q'=quit"
    )
    print(
        f"approach axis (TCP frame): {args.approach_axis}, "
        f"standoff distance: {args.distance_mm:g} mm, jntVelScale: {args.jnt_vel_scale}"
        + ("  [DRY-RUN — no motion will be commanded]" if args.dry_run else "")
    )
    if args.approach_via_mm > args.distance_mm:
        final_leg = (
            f"{args.final_primitive.upper()} to final target "
            f"(vel={args.linear_vel:.3f} m/s)"
            if args.final_primitive == "movel"
            else f"{args.final_primitive.upper()} to final target"
        )
        print(
            f"auto-motion plan: MovePTP to {args.approach_via_mm:g} mm standoff, "
            f"then {final_leg}"
        )
    else:
        final_leg = (
            f"single {args.final_primitive.upper()} to final target "
            f"(vel={args.linear_vel:.3f} m/s)"
            if args.final_primitive == "movel"
            else f"single {args.final_primitive.upper()} to final target"
        )
        print(
            f"auto-motion plan: {final_leg} "
            f"(set --approach-via-mm > {args.distance_mm:g} to add a via pose)"
        )

    pending_target = None
    flash_text = ""
    flash_color = (0, 200, 0)
    flash_frames = 0
    last_guide_within_tol = False
    win = "Calibration verify"
    try:
        while True:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            raw_image = np.asanyarray(color_frame.get_data()).copy()
            display = raw_image.copy()

            status = ""
            status_color = (200, 200, 200)
            try:
                gray = cv2.cvtColor(raw_image, cv2.COLOR_BGR2GRAY)
                (charuco_corners, charuco_ids, marker_corners, marker_ids) = (
                    detect_charuco_corners(
                        cv2, gray, board, dictionary, camera_matrix, dist_coeffs
                    )
                )
                marker_count = 0 if marker_ids is None else int(len(marker_ids))
                charuco_count = 0 if charuco_ids is None else int(len(charuco_ids))
                if marker_count > 0:
                    cv2.aruco.drawDetectedMarkers(display, marker_corners, marker_ids)
                if charuco_count > 0:
                    cv2.aruco.drawDetectedCornersCharuco(
                        display, charuco_corners, charuco_ids
                    )
                status = (
                    f"markers={marker_count}  charuco={charuco_count}/min8"
                )
                status_color = (0, 200, 0) if charuco_count >= 8 else (0, 0, 255)
            except Exception as exc:
                status = f"detect error: {exc}"
                status_color = (0, 0, 255)

            draw_text(cv2, display, status, (10, 25), status_color)
            guide_summary = None
            guide_status = "TARGET PENDING"
            guide_color = (0, 200, 200)
            if pending_target is not None:
                try:
                    _, state = session.selected_arm_state()
                    current_tcp = [float(v) for v in getattr(state, "tcp_pose", [])]
                    if len(current_tcp) != 7:
                        raise RuntimeError(
                            f"tcp_pose has {len(current_tcp)} values; expected 7"
                        )
                    guide_summary = summarize_pose_delta(
                        current_tcp,
                        pending_target["target_tcp"],
                    )
                    within_tol = (
                        guide_summary["dpos_mm"] <= float(args.guide_pos_tol_mm)
                        and guide_summary["drot_deg"] <= float(args.guide_rot_tol_deg)
                    )
                    if within_tol:
                        guide_status = "GUIDED TARGET REACHED"
                        guide_color = (0, 200, 0)
                        if not last_guide_within_tol:
                            flash_text = (
                                f"TARGET REACHED "
                                f"({guide_summary['dpos_mm']:.1f} mm, "
                                f"{guide_summary['drot_deg']:.1f} deg)"
                            )
                            flash_color = (0, 200, 0)
                            flash_frames = 45
                    else:
                        guide_status = "TARGET PENDING"
                        guide_color = (0, 200, 200)
                    last_guide_within_tol = within_tol
                except Exception as exc:
                    guide_status = f"GUIDE ERROR: {exc}"
                    guide_color = (0, 0, 255)
                    last_guide_within_tol = False
                draw_text(
                    cv2, display,
                    f"{guide_status} — 'g'=auto move, 'x'=cancel",
                    (10, 50), guide_color,
                )
                if guide_summary is not None:
                    draw_text(
                        cv2,
                        display,
                        (
                            f"remaining: {guide_summary['dpos_mm']:.1f} mm, "
                            f"{guide_summary['drot_deg']:.1f} deg"
                        ),
                        (10, 75),
                        guide_color,
                    )
                    dtx, dty, dtz = guide_summary["delta_target_mm"]
                    draw_text(
                        cv2,
                        display,
                        (
                            f"target-frame xyz(mm): "
                            f"{dtx:+.1f} {dty:+.1f} {dtz:+.1f}"
                        ),
                        (10, 100),
                        guide_color,
                    )
            else:
                draw_text(cv2, display, "no pending target", (10, 50), (180, 180, 180))
                last_guide_within_tol = False
            if flash_frames > 0:
                draw_text(
                    cv2, display, flash_text,
                    (10, display.shape[0] - 15), flash_color,
                )
                flash_frames -= 1

            cv2.imshow(win, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord("x"):
                if pending_target is not None:
                    pending_target = None
                    print("cancelled pending target")
                    flash_text = "CANCELLED"
                    flash_color = (0, 200, 200)
                    flash_frames = 30
            elif key == ord("c"):
                try:
                    pose = estimate_charuco_pose(
                        cv2, raw_image, board, dictionary,
                        camera_matrix, dist_coeffs, min_charuco_corners=8,
                    )
                    _, state = session.selected_arm_state()
                    tcp_now = [float(v) for v in getattr(state, "tcp_pose", [])]
                    if len(tcp_now) != 7:
                        raise RuntimeError(
                            f"tcp_pose has {len(tcp_now)} values; expected 7"
                        )
                    T_world_tcp = pose_vec_to_transform(tcp_now)
                    T_world_camera = T_world_tcp @ T_tcp_camera
                    T_world_board = T_world_camera @ pose.T_camera_board
                    target_pos, target_R = compute_verify_target(
                        T_world_board, approach_tcp, distance_m,
                        camera_pos_world=T_world_camera[:3, 3],
                    )
                    target_tcp = tcp_pose_from_pos_R(target_pos, target_R)
                    guide_summary = summarize_pose_delta(tcp_now, target_tcp)

                    via_tcp = None
                    if float(args.approach_via_mm) > float(args.distance_mm):
                        via_pos, via_R = compute_verify_target(
                            T_world_board,
                            approach_tcp,
                            float(args.approach_via_mm) / 1000.0,
                            camera_pos_world=T_world_camera[:3, 3],
                        )
                        via_tcp = tcp_pose_from_pos_R(via_pos, via_R)

                    q_rad = list(getattr(state, "q", []))
                    if len(q_rad) < 7:
                        raise RuntimeError(
                            f"state.q has {len(q_rad)} values; expected at least 7"
                        )
                    ref_joints_deg = [math.degrees(v) for v in q_rad[:7]]

                    print("\n--- captured + target ---")
                    print(
                        f"current TCP  xyz=({tcp_now[0]:+.4f}, {tcp_now[1]:+.4f}, "
                        f"{tcp_now[2]:+.4f}) m"
                    )
                    print(
                        f"board pose   xyz=({T_world_board[0,3]:+.4f}, "
                        f"{T_world_board[1,3]:+.4f}, {T_world_board[2,3]:+.4f}) m  "
                        f"(reproj={pose.reprojection_error_px:.2f}px)"
                    )
                    print(
                        f"target TCP   xyz=({target_tcp[0]:+.4f}, {target_tcp[1]:+.4f}, "
                        f"{target_tcp[2]:+.4f}) m"
                    )
                    print(
                        f"             qwxyz=({target_tcp[3]:+.4f}, {target_tcp[4]:+.4f}, "
                        f"{target_tcp[5]:+.4f}, {target_tcp[6]:+.4f})"
                    )
                    if via_tcp is not None:
                        print(
                            f"via TCP      xyz=({via_tcp[0]:+.4f}, {via_tcp[1]:+.4f}, "
                            f"{via_tcp[2]:+.4f}) m"
                        )
                    print(
                        "delta from current: "
                        f"pos={guide_summary['dpos_mm']:.1f} mm, "
                        f"rot={guide_summary['drot_deg']:.1f} deg"
                    )
                    print(
                        "hand-guide in FloatingJoint while watching the live error, "
                        "or press 'g' for auto motion."
                    )
                    pending_target = {
                        "target_tcp": target_tcp,
                        "via_tcp": via_tcp,
                        "ref_joints_deg": ref_joints_deg,
                        "capture_summary": guide_summary,
                    }
                    last_guide_within_tol = False
                    flash_text = (
                        f"CAPTURED — target ready "
                        f"(dpos={guide_summary['dpos_mm']:.1f}mm)"
                    )
                    flash_color = (0, 200, 0)
                    flash_frames = 45
                except Exception as exc:
                    print(f"capture failed: {exc}")
                    flash_text = f"CAPTURE FAILED: {exc}"
                    flash_color = (0, 0, 255)
                    flash_frames = 45
            elif key == ord("g"):
                if pending_target is None:
                    flash_text = "NO PENDING TARGET — press 'c' first"
                    flash_color = (0, 0, 255)
                    flash_frames = 30
                    continue
                target_tcp = pending_target["target_tcp"]
                via_tcp = pending_target["via_tcp"]
                ref_joints_deg = pending_target["ref_joints_deg"]
                motion_plan = build_verify_motion_plan(
                    target_tcp,
                    via_tcp=via_tcp,
                    final_primitive=args.final_primitive,
                )
                if args.dry_run:
                    plan_desc = " -> ".join(
                        f"{step.primitive}:{step.name}" for step in motion_plan
                    )
                    print(f"dry-run: would execute {plan_desc}.")
                    flash_text = "DRY RUN — no motion"
                    flash_color = (0, 200, 200)
                    flash_frames = 45
                    pending_target = None
                    continue
                print(
                    "MOVING: staged verify motion "
                    f"({len(motion_plan)} step{'s' if len(motion_plan) != 1 else ''}) "
                    "— step away from the arm."
                )
                try:
                    try:
                        session.robot.Stop()
                    except Exception:
                        pass
                    session.switch_mode("NRT_PRIMITIVE_EXECUTION")
                    for index, step in enumerate(motion_plan, start=1):
                        print(
                            f"  step {index}/{len(motion_plan)}: {step.primitive} "
                            f"-> {step.name}"
                        )
                        execute_verify_motion_step(
                            session,
                            helpers,
                            step,
                            ref_joints_deg=ref_joints_deg,
                            jnt_vel_scale=args.jnt_vel_scale,
                            linear_vel=args.linear_vel,
                        )
                    _, state = session.selected_arm_state()
                    arrived = [float(v) for v in getattr(state, "tcp_pose", [])]
                    arrive_summary = summarize_pose_delta(arrived, target_tcp)
                    print(
                        f"ARRIVED at xyz=({arrived[0]:+.4f}, {arrived[1]:+.4f}, "
                        f"{arrived[2]:+.4f}) m  "
                        f"(residual={arrive_summary['dpos_mm']:.2f} mm, "
                        f"{arrive_summary['drot_deg']:.2f} deg)"
                    )
                    flash_text = (
                        f"ARRIVED ({arrive_summary['dpos_mm']:.2f}mm, "
                        f"{arrive_summary['drot_deg']:.2f}deg)"
                    )
                    flash_color = (0, 200, 0)
                except Exception as exc:
                    print(f"motion failed: {exc}")
                    flash_text = f"MOTION FAILED: {exc}"
                    flash_color = (0, 0, 255)
                finally:
                    try:
                        try:
                            session.robot.Stop()
                        except Exception:
                            pass
                        enable_joint_floating(session)
                    except Exception as exc:
                        print(f"warning: could not re-enter FloatingJoint: {exc}")
                    pending_target = None
                    flash_frames = 60
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        try:
            session_cm.__exit__(None, None, None)
        except Exception as exc:
            print(f"warning: robot session cleanup raised: {exc}")
    return 0


def command_solve(args: argparse.Namespace) -> int:
    cv2 = require_cv2()
    board_cfg = load_board_config(Path(args.board))
    board, dictionary = create_charuco_board(cv2, board_cfg)

    detections, rejected = collect_detections(
        cv2,
        Path(args.samples_dir),
        board,
        dictionary,
        min_charuco_corners=args.min_charuco_corners,
    )
    if len(detections) < args.min_samples:
        raise SystemExit(
            f"Only {len(detections)} accepted samples; need at least {args.min_samples}. "
            "Capture more varied views or lower --min-samples."
        )

    T_tcp_camera = solve_hand_eye_from_transforms(
        cv2,
        [det.T_world_tcp for det in detections],
        [det.T_camera_board for det in detections],
        method=args.method,
    )
    T_camera_tcp = invert_transform(T_tcp_camera)
    stats = validation_stats(detections, T_tcp_camera)
    axis = camera_axis_check(
        T_tcp_camera,
        args.expected_camera_z_tcp,
        args.max_axis_angle_deg,
    )

    output = {
        "schema_version": 1,
        "calibrated_at": utc_timestamp(),
        "calibration_type": "eye_in_hand",
        "method": args.method,
        "frame_convention": {
            "camera": "OpenCV/RealSense color optical: +x right, +y down, +z forward",
            "tcp_pose_world": "Flexiv tcp_pose [x, y, z, qw, qx, qy, qz]",
            "transform_matrix": "T_a_b maps coordinates from frame b into frame a",
        },
        "board": board_config_for_yaml(board_cfg),
        "camera": first_camera_record(detections),
        "T_tcp_camera": matrix_to_list(T_tcp_camera),
        "T_camera_tcp": matrix_to_list(T_camera_tcp),
        "transforms": {
            "T_tcp_camera": transform_record(T_tcp_camera, "tcp", "camera"),
            "T_camera_tcp": transform_record(T_camera_tcp, "camera", "tcp"),
        },
        "quality": stats,
        "axis_check": axis,
        "accepted_samples": accepted_sample_records(detections),
        "rejected_samples": rejected,
    }
    write_yaml(output, Path(args.out))

    print(f"\nSolved T_tcp_camera -> {args.out}")
    print("Translation TCP<-camera [m]:", vector_to_list(T_tcp_camera[:3, 3]))
    print(
        "Quaternion TCP<-camera [qw,qx,qy,qz]:",
        vector_to_list(matrix_to_quat_wxyz(T_tcp_camera[:3, :3])),
    )
    print_quality(stats)
    print_axis_check(axis)
    return 0


def command_validate(args: argparse.Namespace) -> int:
    cv2 = require_cv2()
    calibration = read_yaml(Path(args.calibration))
    board_path = Path(args.board) if args.board else DEFAULT_BOARD_PATH
    board_cfg = load_board_config(board_path)
    board, dictionary = create_charuco_board(cv2, board_cfg)

    T_tcp_camera = np.asarray(calibration["T_tcp_camera"], dtype=float).reshape(4, 4)
    detections, rejected = collect_detections(
        cv2,
        Path(args.samples_dir),
        board,
        dictionary,
        min_charuco_corners=args.min_charuco_corners,
    )
    if len(detections) < 3:
        raise SystemExit(
            f"Only {len(detections)} accepted samples; need at least 3 to validate."
        )

    stats = validation_stats(detections, T_tcp_camera)
    axis = camera_axis_check(
        T_tcp_camera,
        args.expected_camera_z_tcp,
        args.max_axis_angle_deg,
    )

    print_quality(stats)
    print_axis_check(axis)
    if rejected:
        print(f"\nRejected samples: {len(rejected)}")
        for item in rejected:
            print(f"  {item['sample']}: {item['reason']}")
    return 0


# ---------------------------------------------------------------------------
# CLI


def add_common_board_arg(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--board",
        default=str(DEFAULT_BOARD_PATH),
        help="Path to the ChArUco board YAML.",
    )


def add_axis_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--expected-camera-z-tcp",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        default=[0.0, 0.0, 1.0],
        help="Expected direction of camera optical +Z in TCP coordinates.",
    )
    parser.add_argument(
        "--max-axis-angle-deg",
        type=float,
        default=45.0,
        help="Warn if camera optical +Z differs from expected direction by more than this.",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Calibrate a gripper-mounted RealSense camera to the Flexiv TCP."
    )
    sub = parser.add_subparsers(dest="command", required=True)

    capture = sub.add_parser("capture", help="Capture one image/TCP-pose sample.")
    add_common_board_arg(capture)
    capture.add_argument("--samples-dir", default=str(DEFAULT_SAMPLES_DIR))
    capture.add_argument("--sample-id", help="Optional sample directory name.")
    capture.add_argument("--robot-sn", default=DEFAULT_ROBOT_SN)
    capture.add_argument("--operational-timeout-s", type=float, default=30.0)
    capture.add_argument("--prompt", action="store_true", help="Wait for Enter before capture.")
    capture.add_argument("--serial", help="Specific RealSense serial number to use.")
    capture.add_argument("--width", type=int, default=640)
    capture.add_argument("--height", type=int, default=480)
    capture.add_argument("--fps", type=int, default=30)
    capture.add_argument("--warmup-frames", type=int, default=30)
    capture.add_argument(
        "--exposure",
        type=float,
        default=19000,
        help="Manual color exposure. Setting this turns off auto exposure.",
    )
    capture.add_argument("--gain", type=float, help="Manual color gain.")
    capture.add_argument(
        "--white-balance",
        type=float,
        help="Manual white balance in Kelvin. Setting this turns off auto white balance.",
    )
    capture.set_defaults(func=command_capture)

    preview = sub.add_parser(
        "preview",
        help="Live RealSense window with optional ChArUco detection overlay.",
    )
    add_common_board_arg(preview)
    preview.add_argument("--serial", help="Specific RealSense serial number to use.")
    preview.add_argument("--width", type=int, default=640)
    preview.add_argument("--height", type=int, default=480)
    preview.add_argument("--fps", type=int, default=30)
    preview.add_argument(
        "--exposure",
        type=float,
        default=19000,
        help="Manual color exposure. Setting this turns off auto exposure.",
    )
    preview.add_argument("--gain", type=float, help="Manual color gain.")
    preview.add_argument(
        "--white-balance",
        type=float,
        help="Manual white balance in Kelvin. Setting this turns off auto white balance.",
    )
    preview.add_argument(
        "--no-detect",
        action="store_true",
        help="Skip ChArUco detection overlay (just show the raw stream).",
    )
    preview.add_argument(
        "--min-charuco-corners",
        type=int,
        default=8,
        help="Status overlay turns green when at least this many corners are detected.",
    )
    preview.add_argument(
        "--snapshot-dir",
        default=str(HERE / "preview_snapshots"),
        help="Directory the 's' key writes snapshots into.",
    )
    preview.add_argument(
        "--samples-dir",
        default=str(DEFAULT_SAMPLES_DIR),
        help="Directory the capture key writes calibration samples into.",
    )
    preview.add_argument(
        "--sample-id-prefix",
        default=None,
        help="Base name for captured sample directories. Defaults to a UTC timestamp.",
    )
    preview.add_argument("--robot-sn", default=DEFAULT_ROBOT_SN)
    preview.add_argument("--operational-timeout-s", type=float, default=30.0)
    preview.add_argument(
        "--no-robot",
        action="store_true",
        help="Skip opening a robot session (disables the capture key, snapshots still work).",
    )
    preview.add_argument(
        "--capture-key",
        default="c",
        help="Single character that triggers a calibration capture (default: 'c').",
    )
    preview.add_argument(
        "--hold-position",
        action="store_true",
        help=(
            "Keep the robot actively holding position. Default is to start the FloatingJoint "
            "primitive so the arm can be hand-guided between captures."
        ),
    )
    preview.set_defaults(func=command_preview)

    solve = sub.add_parser("solve", help="Solve T_tcp_camera from captured samples.")
    add_common_board_arg(solve)
    add_axis_args(solve)
    solve.add_argument("--samples-dir", default=str(DEFAULT_SAMPLES_DIR))
    solve.add_argument("--out", default=str(DEFAULT_CALIBRATION_PATH))
    solve.add_argument("--method", choices=sorted(HAND_EYE_METHODS), default="tsai")
    solve.add_argument("--min-charuco-corners", type=int, default=8)
    solve.add_argument("--min-samples", type=int, default=8)
    solve.set_defaults(func=command_solve)

    verify = sub.add_parser(
        "verify",
        help=(
            "Functional test: capture board pose, then move TCP to the board normal "
            "at a fixed standoff and visually check alignment."
        ),
    )
    add_common_board_arg(verify)
    verify.add_argument(
        "--calibration", default=str(DEFAULT_CALIBRATION_PATH),
        help="Path to the saved calibration YAML (T_tcp_camera).",
    )
    verify.add_argument("--serial", help="Specific RealSense serial number to use.")
    verify.add_argument("--width", type=int, default=640)
    verify.add_argument("--height", type=int, default=480)
    verify.add_argument("--fps", type=int, default=30)
    verify.add_argument(
        "--exposure", type=float, default=19000,
        help="Manual color exposure. Setting this turns off auto exposure.",
    )
    verify.add_argument("--gain", type=float, help="Manual color gain.")
    verify.add_argument(
        "--white-balance", type=float,
        help="Manual white balance in Kelvin. Setting this turns off auto white balance.",
    )
    verify.add_argument("--robot-sn", default=DEFAULT_ROBOT_SN)
    verify.add_argument("--operational-timeout-s", type=float, default=30.0)
    verify.add_argument(
        "--distance-mm", type=float, default=100.0,
        help="Standoff distance from the board along its +Z normal (default 100).",
    )
    verify.add_argument(
        "--approach-axis", default="+z",
        help="TCP axis that should point at the board (default +z; allowed: +x,-x,+y,-y,+z,-z).",
    )
    verify.add_argument(
        "--jnt-vel-scale", type=int, default=5,
        help="MovePTP joint velocity scale 1-100 (default 5 = slow).",
    )
    verify.add_argument(
        "--approach-via-mm",
        type=float,
        default=250.0,
        help=(
            "Optional larger standoff used as an intermediate auto-motion pose "
            "before the final target. Set <= --distance-mm to disable."
        ),
    )
    verify.add_argument(
        "--final-primitive",
        choices=("movel", "moveptp"),
        default="movel",
        help="Primitive used for the final auto-motion leg (default: movel).",
    )
    verify.add_argument(
        "--linear-vel",
        type=float,
        default=0.02,
        help="TCP velocity in m/s for the final MoveL leg (default: 0.02).",
    )
    verify.add_argument(
        "--guide-pos-tol-mm",
        type=float,
        default=5.0,
        help="Manual-guidance position tolerance in mm for the live target indicator.",
    )
    verify.add_argument(
        "--guide-rot-tol-deg",
        type=float,
        default=3.0,
        help="Manual-guidance rotation tolerance in deg for the live target indicator.",
    )
    verify.add_argument(
        "--dry-run", action="store_true",
        help="Compute and print the target but never command motion.",
    )
    verify.set_defaults(func=command_verify)

    validate = sub.add_parser("validate", help="Validate a saved calibration.")
    add_axis_args(validate)
    validate.add_argument("--samples-dir", default=str(DEFAULT_SAMPLES_DIR))
    validate.add_argument("--calibration", default=str(DEFAULT_CALIBRATION_PATH))
    validate.add_argument(
        "--board",
        help="Path to board YAML. Defaults to project/calibration/tag_01.yaml.",
    )
    validate.add_argument("--min-charuco-corners", type=int, default=8)
    validate.set_defaults(func=command_validate)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return int(args.func(args))
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130


if __name__ == "__main__":
    sys.exit(main())
