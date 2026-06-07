#!/usr/bin/env python3
"""GreenPink injectable teardown recipe.

This script implements the human pseudo-code in ``plan.md``. It coordinates
the Flexiv Rizon4 + GN01 gripper with the Arduino-controlled cutting machine.

Start with:

    python recipe.py --dry-run

Real hardware execution intentionally prompts before CUT_HEIGHT unless ``--yes``
is passed.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np


HERE = Path(__file__).resolve().parent
PROJECT_DIR = HERE.parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from helper.flexiv_helpers import (  # noqa: E402
    RobotSession,
    gripper_set as helper_gripper_set,
    joints_to_jpos_deg,
    move_ptp_joint,
    rpy_deg_to_quat,
    rotate_tcp_about_tool_axis,
    tcp_pose_to_coord_args,
)
from helper.arduino_client import (  # noqa: E402
    ArduinoClient,
    ArduinoCommandError,
    DEFAULT_BAUD,
    DEFAULT_READY_TIMEOUT_S,
)
from helper.injectable_camera_session import (  # noqa: E402
    SharedInjectableCameraSession,
)


PARAMS: dict[str, Any] = {
    "ROBOT_SN": "Rizon4-062930",
    "GRIPPER_NAME": "Flexiv-GN01",
    "KEY_POSITION_DIR": "key_positions",
    "SLOW_MODE_ENABLED": False,
    "MOVE_JNT_VEL_SCALE": 30,
    "MOVE_USE_REF_JOINTS": False,
    "MOVE_ZONE_RADIUS": "ZFine",
    "MOVE_TARGET_TOLER_LEVEL": 1,
    "MOVE_JNT_ACC_MULTIPLIER": 1.0,
    "CARTESIAN_INSERT_VEL_M_S": 0.06,
    "CARTESIAN_RETREAT_VEL_M_S": 0.06,
    "GRIPPER_OPEN_WIDTH_M": 0.06,
    "GRIPPER_RELEASE_WIDTH_M": 0.05,
    "GRIPPER_CLOSE_WIDTH_M": 0.0,
    "GRIPPER_VELOCITY_M_S": 0.05,
    "GRIPPER_FORCE_N": 80.0,
    "GRIPPER_SETTLE_S": 1.0,
    "GRIPPER_OPEN_SETTLE_S": 1.5,
    "INJECTABLE_ALIGN_TARGET_INDEX": 1,
    "INJECTABLE_CALIBRATION": str(PROJECT_DIR / "calibration" / "camera_tcp.yaml"),
    "INJECTABLE_CAPTURE_OUT": str(HERE / "injectable_capture.jpg"),
    "INJECTABLE_ALIGN_OUT": str(HERE / "injectable_align_detected.jpg"),
    "POV_RECORD_ENABLED": False,
    "POV_RECORD_PATH": str(HERE / "operation_pov.mp4"),
    "INJECTABLE_CAMERA_SERIAL": None,
    "INJECTABLE_WIDTH": 640,
    "INJECTABLE_HEIGHT": 480,
    "INJECTABLE_FPS": 30,
    "INJECTABLE_WARMUP_FRAMES": 30,
    "INJECTABLE_EXPOSURE": 20000,
    "INJECTABLE_GAIN": None,
    "INJECTABLE_WHITE_BALANCE": None,
    "INJECTABLE_TEAL_LOW_HSV": [84, 110, 70],
    "INJECTABLE_TEAL_HIGH_HSV": [100, 255, 255],
    "INJECTABLE_PINK_LOW_HSV": [125, 40, 40],
    "INJECTABLE_PINK_HIGH_HSV": [175, 255, 255],
    "INJECTABLE_MIN_PINK_AREA": 300,
    "INJECTABLE_MIN_TEAL_AREA": 150,
    "INJECTABLE_DIST_TOLERANCE": 0.30,
    "INJECTABLE_AXIS_COS_MIN": 0.70,
    "INJECTABLE_ALIGN_PRIMITIVE": "movel",
    "INJECTABLE_ALIGN_LINEAR_VEL_M_S": 0.02,
    "INJECTABLE_ALIGN_OFFSET_TOWARD_PINK_M": 0.02,
    "INJECTABLE_ALIGN_MAX_ATTEMPTS": 3,
    "INJECTABLE_ALIGN_RETRY_SETTLE_S": 0.75,
    "INJECTABLE_ALIGN_FRAME_TIMEOUT_S": 5.0,
    "GRASP_OPEN_WIDTH_M": 0.06,
    "GRASP_FORCE_N": 50.0,
    "GRASP_OPEN_FORCE_LIMIT_N": 10.0,
    "GRASP_CONTACT_FORCE_N": 8.0,
    "GRASP_PRECONTACT_MOVE_Z_M": 0.20,
    "GRASP_PRECONTACT_MOVE_VEL_M_S": 0.15,
    "GRASP_CONTACT_VEL_M_S": 0.05,
    "TAG_CALI_ENABLED": True,
    "TAG_1_STAGING_KEY_POSITION": "tag_1",
    "TAG_2_STAGING_KEY_POSITION": "tag_2",
    "TAG_3_STAGING_KEY_POSITION": "tag_3",
    "VISE_CALI_TARGET_KEY_POSITION": "Vise",
    "VISE_CALI_BOARD": str(PROJECT_DIR / "calibration" / "tag_01.yaml"),
    "VISE_CALI_CALIBRATION": str(PROJECT_DIR / "calibration" / "camera_tcp.yaml"),
    "VISE_CALI_REFERENCE": str(PROJECT_DIR / "calibration" / "tag_01_to_vise_tcp.json"),
    "VISE_CALI_CAPTURE_OUT": str(HERE / "cali_vise_capture.jpg"),
    "VISE_CALI_DETECT_OUT": str(HERE / "cali_vise_detected.jpg"),
    "VISE_CALI_MIN_CHARUCO_CORNERS": 8,
    "TAG_CALI_MAX_ATTEMPTS": 3,
    "TAG_CALI_RETRY_SETTLE_S": 0.75,
    "PICKUP_LIFT_Z_OFFSET_M": 0.20,
    "VISE_APPROACH_Z_OFFSET_M": 0.05,
    "VISE_RETREAT_Z_OFFSET_M": 0.15,
    "SPRING_REMOVE_APPROACH_Z_OFFSET_M": 0.10,
    "SPRING_REMOVE_GRIP_Z_OFFSET_M": 0.0,
    "SPRING_REMOVE_FORCE_N": 80.0,
    "SPRING_REMOVE_LIFT_Z_OFFSET_M": 0.15,
    "YELLOW_REMOVE_APPROACH_Z_OFFSET_M": 0.10,
    "YELLOW_REMOVE_GRIP_Z_OFFSET_M": -0.038,
    "YELLOW_REMOVE_FORCE_N": 80.0,
    "YELLOW_REMOVE_LIFT_Z_OFFSET_M": 0.10,
    "SHELL_REMOVE_APPROACH_Z_OFFSET_M": 0.10,
    "SHELL_REMOVE_GRIP_Z_OFFSET_M": -0.06,
    "SHELL_REMOVE_FORCE_N": 80.0,
    "SHELL_REMOVE_LIFT_Z_OFFSET_M": 0.20,
    "DUMP_TOOL_Z_DEG": 176.0,
    "DUMP_VP_OFFSET_X_M": 0.0,
    "DUMP_VP_OFFSET_Y_M": 0.02,
    "DUMP_VP_OFFSET_Z_M": 0.02,
    "DUMP_MOVEC_VEL_M_S": 0.03,
    "DUMP_MOVEC_ACC_M_S2": 0.10,
    "DUMP_MOVEC_JERK_M_S3": 100.0,
    "DUMP_MOVEC_EQUAL_RADIUS": 0.1,
    "VISE_OPEN_TARGET_FORCE_KG": 0.2,
    "INSERTCOMP_INSERT_AXIS": "AUTO_WORLD_NEG_Z",
    "INSERTCOMP_COMP_AXIS": [0, 1, 1, 0, 0, 0],
    "INSERTCOMP_MAX_CONTACT_FORCE_N": 8.0,
    "INSERTCOMP_DEADBAND_SCALE": 80.0,
    "INSERTCOMP_INSERT_VEL_M_S": 0.02,
    "INSERTCOMP_COMP_VEL_SCALE": 20.0,
    "INSERTCOMP_START_TIMEOUT_S": 3.0,
    "INSERTCOMP_TIMEOUT_S": 20.0,
    "VISE_TARGET_FORCE_KG": 4.0,
    "CUT_X_MM": 110.5,
    "CUT_Z_MM": 133.0,
    "CUT_DEG": 360.0,
    "ROT_SAFE_TOL_DEG": 0.2,
    "CAP_GRIP_Z_OFFSET_M": 0.0,
    "CAP_TWIST_DEG": 5.0,
    "CAP_TWIST_REPEAT_COUNT": 1,
    "CAP_LIFT_Z_OFFSET_M": 0.20,
    "ARDUINO_PORT": None,
    "ARDUINO_BAUD": DEFAULT_BAUD,
    "ARDUINO_DONE_TIMEOUT_S": 60.0,
}

SHARED_KEY_POSITION_DIR = PROJECT_DIR / "key_positions"
SLOW_PROFILE_KEY_MOVE_JNT_VEL_SCALE = 10
SLOW_PROFILE_TOOL_MOVE_LINEAR_VEL_M_S = 0.02
SLOW_PROFILE_DUMP_MOVEC_VEL_M_S = 0.03

REQUIRED_POSITIONS = ("Middle", "Plate", "Vise", "Spring", "Plastic", "Glass")
OPTIONAL_POSITIONS = ("tag_1", "tag_2", "tag_3")

TAG_CALIBRATION_SEQUENCE = (
    {
        "tag_id": 1,
        "staging_param": "TAG_1_STAGING_KEY_POSITION",
        "refresh_names": ("Vise",),
    },
    {
        "tag_id": 2,
        "staging_param": "TAG_2_STAGING_KEY_POSITION",
        "refresh_names": ("Plate",),
    },
    {
        "tag_id": 3,
        "staging_param": "TAG_3_STAGING_KEY_POSITION",
        "refresh_names": ("Spring", "Plastic", "Glass"),
    },
)

PLAN_STEPS = {
    1: "open gripper",
    2: "move to Middle",
    3: "move to Plate camera staging pose",
    4: "align injectable with camera",
    5: "adaptive grasp injectable",
    6: "lift injectable up in positive world Z",
    7: "move to Middle",
    8: "move to above Vise",
    9: "InsertComp down into vise",
    10: "close vise",
    11: "release injectable with wider gripper opening",
    12: "retreat up in world Z and wait clear of vise",
    13: "CUT_HEIGHT",
    14: "move down to Vise",
    15: "close gripper at 80 N on cap",
    16: "twist cap around TCP X and return",
    17: "lift cap up in positive world Z",
    18: "move to Middle",
    19: "move to Plastic",
    20: "open gripper",
}

REMOVAL_CYCLES = (
    {
        "name": "Spring",
        "steps": {
            "transit": 21,
            "above_vise": 22,
            "grip": 23,
            "close": 24,
            "lift": 25,
            "return_transit": 26,
            "drop": 27,
            "open": 28,
        },
        "approach_z_param": "SPRING_REMOVE_APPROACH_Z_OFFSET_M",
        "grip_z_param": "SPRING_REMOVE_GRIP_Z_OFFSET_M",
        "force_param": "SPRING_REMOVE_FORCE_N",
        "lift_z_param": "SPRING_REMOVE_LIFT_Z_OFFSET_M",
        "drop_target": "Spring",
    },
    {
        "name": "Yellow plastic",
        "steps": {
            "transit": 29,
            "above_vise": 30,
            "grip": 31,
            "close": 32,
            "lift": 33,
            "return_transit": 34,
            "drop": 35,
            "open": 36,
        },
        "approach_z_param": "YELLOW_REMOVE_APPROACH_Z_OFFSET_M",
        "grip_z_param": "YELLOW_REMOVE_GRIP_Z_OFFSET_M",
        "force_param": "YELLOW_REMOVE_FORCE_N",
        "lift_z_param": "YELLOW_REMOVE_LIFT_Z_OFFSET_M",
        "drop_target": "Plastic",
    },
)

for cycle in REMOVAL_CYCLES:
    name = str(cycle["name"])
    drop_target = str(cycle["drop_target"])
    steps = dict(cycle["steps"])
    PLAN_STEPS.update(
        {
            int(steps["transit"]): f"move to Middle for {name} removal",
            int(steps["above_vise"]): f"move to above Vise for {name} removal",
            int(steps["grip"]): f"move to Vise grasp for {name} removal",
            int(steps["close"]): f"close gripper for {name} removal",
            int(steps["lift"]): f"lift {name} up in positive world Z",
            int(steps["return_transit"]): f"move to Middle after {name} removal",
            int(steps["drop"]): f"move to {drop_target} drop after {name} removal",
            int(steps["open"]): f"open gripper after {name} removal",
        }
    )

SHELL_GLASS_STEP_SEQUENCE = (
    37,
    38,
    39,
    40,
    41,
    42,
    43,
    44,
    45,
    46,
    47,
    48,
)

UNCUT_STEP_SEQUENCE = (
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
    12,
    *SHELL_GLASS_STEP_SEQUENCE,
)
PLAN_STEPS.update(
    {
        37: "move to Middle for shell and glass removal",
        38: "move to above Vise for shell and glass removal",
        39: "move to Vise grasp for shell and glass removal",
        40: "close gripper for shell and glass removal",
        41: "open vise before lifting shell",
        42: "lift shell up in positive world Z",
        43: "move to Middle after shell removal",
        44: "move to Glass",
        45: "dump with MoveC arc around tool-frame virtual pivot and return",
        46: "move to Plastic",
        47: "open gripper",
        48: "move to Middle",
    }
)


class RecipeError(RuntimeError):
    """Raised for expected recipe validation / safety failures."""


GREENPINK_FAMILY_LOCK_LABEL = "GreenPink-family"


class SingleInstanceLock:
    """Best-effort process lock so only one hardware recipe run is active."""

    def __init__(self, path: Path) -> None:
        self._path = Path(path)
        self._handle = None

    def acquire(self) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        handle = open(self._path, "a+", encoding="utf-8")
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            handle.seek(0)
            owner = handle.read().strip()
            handle.close()
            detail = f": {owner}" if owner else ""
            raise RecipeError(
                f"another {GREENPINK_FAMILY_LOCK_LABEL} recipe process is already running"
                f"{detail}"
            )
        handle.seek(0)
        handle.truncate()
        handle.write(f"pid={os.getpid()} cmd={' '.join(sys.argv)}\n")
        handle.flush()
        self._handle = handle

    def release(self) -> None:
        if self._handle is None:
            return
        try:
            self._handle.seek(0)
            self._handle.truncate()
            fcntl.flock(self._handle.fileno(), fcntl.LOCK_UN)
        finally:
            self._handle.close()
            self._handle = None


class PrintLogger:
    def info(self, msg: str) -> None:
        print(f"[info] {msg}", flush=True)

    def warn(self, msg: str) -> None:
        print(f"[warn] {msg}", flush=True)

    def error(self, msg: str) -> None:
        print(f"[error] {msg}", file=sys.stderr, flush=True)


class DryRunArduino:
    """Small no-hardware stand-in for plan inspection."""

    def __init__(self, logger=None) -> None:
        self.logger = logger or PrintLogger()
        self.status: dict[str, Any] = {
            "homed": False,
            "busy": False,
            "faulted": False,
            "x_mm": 0.0,
            "z_mm": 0.0,
            "rot_deg": 0.0,
            "blade_on": False,
            "vise_state": "OPEN",
            "force_kg": 0.0,
            "active_command": "NONE",
        }

    def connect(self) -> None:
        self.logger.info("[dry-run arduino] connect")

    def close(self) -> None:
        self.logger.info("[dry-run arduino] close")

    def get_status(self) -> dict:
        self.logger.info(f"[dry-run arduino] GET_STATUS -> {self.status}")
        return dict(self.status)

    def home_all(self) -> dict:
        self.logger.info("[dry-run arduino] HOME_ALL")
        self.status.update(homed=True, x_mm=0.0, z_mm=0.0, rot_deg=0.0)
        return dict(self.status)

    def rotate_abs(self, deg: float, speed: float | None = None) -> dict:
        self.logger.info(f"[dry-run arduino] ROTATE_ABS deg={deg} speed={speed}")
        self.status["rot_deg"] = float(deg)
        return dict(self.status)

    def close_vise(self, target_force_kg: float = 4.0) -> dict:
        self.logger.info(f"[dry-run arduino] CLOSE_VISE target_force_kg={target_force_kg}")
        self.status.update(vise_state="CLOSED", force_kg=float(target_force_kg))
        return dict(self.status)

    def open_vise(self, target_force_kg: float = 0.2) -> dict:
        self.logger.info(f"[dry-run arduino] OPEN_VISE target_force_kg={target_force_kg}")
        self.status.update(vise_state="OPEN", force_kg=float(target_force_kg))
        return dict(self.status)

    def cut_height(self, z_mm: float, x_mm: float, deg: float) -> dict:
        self.logger.info(
            f"[dry-run arduino] CUT_HEIGHT z_mm={z_mm} x_mm={x_mm} deg={deg}"
        )
        self.status.update(
            x_mm=0.0,
            z_mm=0.0,
            rot_deg=0.0,
            blade_on=False,
            busy=False,
            faulted=False,
            homed=True,
        )
        return dict(self.status)

    def stop_all(self) -> dict:
        self.logger.info("[dry-run arduino] STOP_ALL")
        self.status.update(blade_on=False, busy=False)
        return dict(self.status)


def make_logger():
    try:
        import spdlog  # type: ignore

        return spdlog.ConsoleLogger("GreenPinkRecipe")
    except Exception:
        return PrintLogger()


def load_record_robot_waypoints_module():
    import record_robot_waypoints as record_robot_waypoints  # type: ignore

    return record_robot_waypoints


def _script_path(path: str) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return HERE / candidate


def _is_numeric_list(value, length: int) -> bool:
    return (
        isinstance(value, list)
        and len(value) == length
        and all(isinstance(v, (int, float)) for v in value)
    )


def _parse_hsv_triplet_arg(value, flag_name: str) -> list[int] | None:
    if value is None:
        return None
    if isinstance(value, (list, tuple)) and len(value) == 3:
        parts = list(value)
    else:
        parts = str(value).replace(",", " ").split()
    if len(parts) != 3:
        raise RecipeError(
            f"{flag_name} must be an HSV triplet like '75,70,60'; got {value!r}"
        )
    try:
        ints = [int(part) for part in parts]
    except ValueError as exc:
        raise RecipeError(
            f"{flag_name} must contain integers; got {value!r}"
        ) from exc
    h, s, v = ints
    return [
        max(0, min(179, h)),
        max(0, min(255, s)),
        max(0, min(255, v)),
    ]


def _position_file_in_dir(search_dir: Path, name: str) -> Path | None:
    exact = search_dir / f"{name}.json"
    if exact.exists():
        return exact
    lower = search_dir / f"{name.lower()}.json"
    if lower.exists():
        return lower
    wanted = f"{name.lower()}.json"
    for candidate in search_dir.glob("*.json"):
        if candidate.name.lower() == wanted:
            return candidate
    return None


def _position_file(key_dir: Path, name: str) -> Path:
    path = _position_file_in_dir(key_dir, name)
    if path is not None:
        return path

    shared_dir = SHARED_KEY_POSITION_DIR
    try:
        same_as_shared = key_dir.resolve() == shared_dir.resolve()
    except FileNotFoundError:
        same_as_shared = False
    if not same_as_shared:
        path = _position_file_in_dir(shared_dir, name)
        if path is not None:
            return path

    raise RecipeError(
        f"missing key position {name!r} in {key_dir} or fallback {shared_dir}"
    )


def load_key_position(key_dir: Path, name: str) -> dict:
    path = _position_file(key_dir, name)
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    tcp = (data.get("tcp_pose_world") or {}).get("values")
    q_rad = data.get("joint_angles_rad")
    if not _is_numeric_list(tcp, 7):
        raise RecipeError(f"{path}: tcp_pose_world.values must contain 7 numbers")
    if not _is_numeric_list(q_rad, 7):
        raise RecipeError(f"{path}: joint_angles_rad must contain 7 numbers")
    return data


def _local_position_file(key_dir: Path, name: str) -> Path:
    path = _position_file_in_dir(key_dir, name)
    if path is not None:
        return path
    return key_dir / f"{name}.json"


def write_key_position_record(path: Path, record: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(record, handle, indent=2)
        handle.write("\n")


def is_tag_calibration_vision_failure(exc: Exception) -> bool:
    message = str(exc).lower()
    return any(
        token in message
        for token in (
            "charuco",
            "aruco",
            "marker",
            "corner",
            "could not estimate the charuco board pose",
            "camera did not return frames",
            "camera did not return aligned color/depth frames",
            "no intel realsense camera detected",
            "no realsense camera found",
        )
    )


def to_pose_entry(key_position: dict) -> dict:
    return {
        "name": key_position.get("name"),
        "q_rad": [float(v) for v in key_position["joint_angles_rad"]],
        "tcp_pose_world": {
            "order": list(key_position["tcp_pose_world"].get("order") or []),
            "values": [float(v) for v in key_position["tcp_pose_world"]["values"]],
        },
    }


def load_positions(key_dir: Path, logger) -> dict[str, dict]:
    raw: dict[str, dict] = {}
    poses: dict[str, dict] = {}
    for name in REQUIRED_POSITIONS:
        raw[name] = load_key_position(key_dir, name)
        poses[name] = to_pose_entry(raw[name])
    for name in OPTIONAL_POSITIONS:
        try:
            raw[name] = load_key_position(key_dir, name)
            poses[name] = to_pose_entry(raw[name])
        except RecipeError:
            logger.warn(f"optional key position {name!r} not found")
    logger.info(f"loaded key positions: {', '.join(sorted(poses))}")
    return poses


def offset_pose_z(pose_entry: dict, dz_m: float) -> dict:
    """Return a pose copy offset along world Z."""
    tcp = list(pose_entry["tcp_pose_world"]["values"])
    tcp[2] += float(dz_m)
    return {
        "name": f"{pose_entry.get('name', 'pose')}_offset_z",
        "q_rad": list(pose_entry["q_rad"]),
        "tcp_pose_world": {
            "order": list(pose_entry["tcp_pose_world"].get("order") or []),
            "values": tcp,
        },
    }


def current_pose_offset_z(session: RobotSession, dz_m: float) -> dict:
    _, state = session.selected_arm_state()
    q_rad = [float(v) for v in getattr(state, "q", [])]
    tcp = [float(v) for v in getattr(state, "tcp_pose", [])]
    if len(q_rad) != 7 or len(tcp) != 7:
        raise RecipeError(
            f"current arm state has q={len(q_rad)}, tcp={len(tcp)}; expected 7 each"
        )
    return offset_pose_z(
        {
            "name": "current",
            "q_rad": q_rad,
            "tcp_pose_world": {
                "order": ["x", "y", "z", "qw", "qx", "qy", "qz"],
                "values": tcp,
            },
        },
        dz_m,
    )


def _fmt_pose_mm(pose_entry: dict) -> str:
    tcp = pose_entry["tcp_pose_world"]["values"]
    return f"x={tcp[0]*1000:.1f} y={tcp[1]*1000:.1f} z={tcp[2]*1000:.1f} mm"


def _quat_to_matrix(quat: list[float]) -> list[list[float]]:
    qw, qx, qy, qz = [float(v) for v in quat]
    norm = (qw * qw + qx * qx + qy * qy + qz * qz) ** 0.5
    if norm == 0.0:
        raise RecipeError("cannot resolve InsertComp axis from zero quaternion")
    qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm
    return [
        [
            1.0 - 2.0 * (qy * qy + qz * qz),
            2.0 * (qx * qy - qz * qw),
            2.0 * (qx * qz + qy * qw),
        ],
        [
            2.0 * (qx * qy + qz * qw),
            1.0 - 2.0 * (qx * qx + qz * qz),
            2.0 * (qy * qz - qx * qw),
        ],
        [
            2.0 * (qx * qz - qy * qw),
            2.0 * (qy * qz + qx * qw),
            1.0 - 2.0 * (qx * qx + qy * qy),
        ],
    ]


def _quat_normalize(quat: list[float]) -> list[float]:
    qw, qx, qy, qz = [float(v) for v in quat]
    norm = (qw * qw + qx * qx + qy * qy + qz * qz) ** 0.5
    if norm == 0.0:
        raise RecipeError("cannot normalize zero quaternion")
    return [qw / norm, qx / norm, qy / norm, qz / norm]


def _quat_multiply(a: list[float], b: list[float]) -> list[float]:
    aw, ax, ay, az = _quat_normalize(a)
    bw, bx, by, bz = _quat_normalize(b)
    return _quat_normalize(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def _mat_vec_mul(matrix: list[list[float]], vec: list[float]) -> list[float]:
    return [
        sum(matrix[row][col] * vec[col] for col in range(3)) for row in range(3)
    ]


def rotate_tcp_about_virtual_offset(
    tcp_pose: list[float],
    *,
    offset_m_xyz: list[float],
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
    yaw_deg: float = 0.0,
) -> list[float]:
    pose = [float(v) for v in tcp_pose]
    if len(pose) != 7:
        raise RecipeError(f"tcp_pose must have 7 values, got {len(pose)}")
    if len(offset_m_xyz) != 3:
        raise RecipeError(
            f"offset_m_xyz must have 3 values, got {len(offset_m_xyz)}"
        )
    position = pose[:3]
    current_quat = _quat_normalize(pose[3:])
    current_matrix = _quat_to_matrix(current_quat)
    pivot_world = [
        position[i] + _mat_vec_mul(current_matrix, offset_m_xyz)[i] for i in range(3)
    ]
    delta_quat = rpy_deg_to_quat(roll_deg, pitch_deg, yaw_deg)
    target_quat = _quat_multiply(current_quat, delta_quat)
    target_matrix = _quat_to_matrix(target_quat)
    rotated_offset = _mat_vec_mul(target_matrix, offset_m_xyz)
    target_position = [
        pivot_world[i] - rotated_offset[i] for i in range(3)
    ]
    return target_position + list(target_quat)


def _vec_sub(a: list[float], b: list[float]) -> list[float]:
    return [float(a[i]) - float(b[i]) for i in range(3)]


def _vec_cross(a: list[float], b: list[float]) -> list[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _vec_norm(values: list[float]) -> float:
    return math.sqrt(sum(float(v) * float(v) for v in values))


def movec_geometry_is_valid(
    start_tcp: list[float],
    middle_tcp: list[float],
    target_tcp: list[float],
) -> bool:
    start_pos = [float(v) for v in start_tcp[:3]]
    middle_pos = [float(v) for v in middle_tcp[:3]]
    target_pos = [float(v) for v in target_tcp[:3]]
    start_to_middle = _vec_sub(middle_pos, start_pos)
    start_to_target = _vec_sub(target_pos, start_pos)
    if _vec_norm(start_to_middle) < 1e-5 or _vec_norm(start_to_target) < 1e-5:
        return False
    return _vec_norm(_vec_cross(start_to_middle, start_to_target)) > 1e-8


def primitive_state_lookup(robot, state_key: str):
    states = robot.primitive_states()
    if isinstance(states, dict) and state_key in states:
        return states[state_key]
    if isinstance(states, dict):
        for state in states.values():
            values = getattr(state, "names_and_values", state)
            if isinstance(values, dict) and state_key in values:
                return values[state_key]
    return None


def tcp_axis_for_world_direction(pose_entry: dict, world_direction: list[float]) -> str:
    """Return the signed TCP axis whose world vector best matches world_direction."""
    tcp = pose_entry["tcp_pose_world"]["values"]
    matrix = _quat_to_matrix(tcp[3:])
    axes = {
        "X": [matrix[0][0], matrix[1][0], matrix[2][0]],
        "Y": [matrix[0][1], matrix[1][1], matrix[2][1]],
        "Z": [matrix[0][2], matrix[1][2], matrix[2][2]],
    }
    direction_norm = sum(float(v) * float(v) for v in world_direction) ** 0.5
    if direction_norm == 0.0:
        raise RecipeError("world_direction must be non-zero")
    desired = [float(v) / direction_norm for v in world_direction]
    best_axis = "X"
    best_dot = -2.0
    for axis, vector in axes.items():
        dot = sum(vector[i] * desired[i] for i in range(3))
        if abs(dot) > best_dot:
            best_axis = axis if dot >= 0.0 else f"-{axis}"
            best_dot = abs(dot)
    return best_axis


def insert_comp_params(params: dict, reference_pose: dict | None = None) -> dict:
    insert_axis = str(params["INSERTCOMP_INSERT_AXIS"])
    if insert_axis == "AUTO_WORLD_NEG_Z":
        if reference_pose is None:
            raise RecipeError("AUTO_WORLD_NEG_Z requires a reference pose")
        insert_axis = tcp_axis_for_world_direction(reference_pose, [0.0, 0.0, -1.0])
    if insert_axis not in {"X", "-X", "Y", "-Y", "Z", "-Z"}:
        raise RecipeError(f"invalid INSERTCOMP_INSERT_AXIS={insert_axis!r}")
    comp_axis = list(params["INSERTCOMP_COMP_AXIS"])
    if len(comp_axis) != 6 or any(int(value) not in (0, 1) for value in comp_axis):
        raise RecipeError("INSERTCOMP_COMP_AXIS must contain six 0/1 values")
    return {
        "insertAxis": insert_axis,
        "compAxis": [int(value) for value in comp_axis],
        "maxContactForce": float(params["INSERTCOMP_MAX_CONTACT_FORCE_N"]),
        "deadbandScale": float(params["INSERTCOMP_DEADBAND_SCALE"]),
        "insertVel": float(params["INSERTCOMP_INSERT_VEL_M_S"]),
        "compVelScale": float(params["INSERTCOMP_COMP_VEL_SCALE"]),
    }


class RecipeContext:
    def __init__(
        self,
        *,
        params: dict,
        poses: dict[str, dict],
        logger,
        dry_run: bool,
        session: RobotSession | None = None,
        arduino=None,
        camera_session: SharedInjectableCameraSession | None = None,
    ) -> None:
        self.params = params
        self.poses = poses
        self.logger = logger
        self.dry_run = dry_run
        self.session = session
        self.arduino = arduino
        self.camera_session = camera_session
        self.arduino_status_query_supported: bool | None = None

    def effective_gripper_velocity_m_s(self, *, for_graspcomp: bool = False) -> float:
        requested = float(self.params["GRIPPER_VELOCITY_M_S"])
        max_vel = requested
        min_vel = 0.0
        if self.session is not None and self.session.gripper is not None:
            try:
                gripper_params = self.session.gripper.params()
                min_vel = float(gripper_params.min_vel)
                max_vel = float(gripper_params.max_vel)
            except Exception:
                pass
        if for_graspcomp:
            max_vel = min(max_vel, 0.05)
        velocity = max(min_vel, min(requested, max_vel))
        if velocity != requested:
            label = "GraspComp" if for_graspcomp else "gripper"
            self.logger.warn(
                f"Requested {label} velocity {requested:.3f} m/s exceeds limit; "
                f"using {velocity:.3f} m/s"
            )
        return velocity

    def move_ptp(
        self,
        label: str,
        pose_entry: dict,
        *,
        joint_locked: bool = False,
        use_ref_joints: bool | None = None,
    ) -> None:
        if use_ref_joints is None:
            use_ref_joints = bool(self.params["MOVE_USE_REF_JOINTS"])
        ref_mode = (
            "locked joints"
            if joint_locked
            else "task-space only"
            if not use_ref_joints
            else "task-space + ref joints"
        )
        self.logger.info(f"MovePTP -> {label} ({_fmt_pose_mm(pose_entry)}; {ref_mode})")
        if self.dry_run:
            return
        if self.session is None:
            raise RecipeError("robot session is not available")
        move_ptp_joint(
            self.session,
            pose_entry,
            vel_scale=int(self.params["MOVE_JNT_VEL_SCALE"]),
            zone_radius=str(self.params["MOVE_ZONE_RADIUS"]),
            target_toler_level=int(self.params["MOVE_TARGET_TOLER_LEVEL"]),
            jnt_acc_multiplier=float(self.params["MOVE_JNT_ACC_MULTIPLIER"]),
            joint_locked=joint_locked,
            use_ref_joints=use_ref_joints,
        )

    def move_l(self, label: str, pose_entry: dict, *, vel_m_s: float) -> None:
        self.logger.info(f"MoveL -> {label} ({_fmt_pose_mm(pose_entry)})")
        if self.dry_run:
            return
        if self.session is None:
            raise RecipeError("robot session is not available")
        flexivrdk = self.session.flexivrdk
        joints_deg = joints_to_jpos_deg(pose_entry["q_rad"])
        tcp = pose_entry["tcp_pose_world"]["values"]
        coord_args = tcp_pose_to_coord_args(tcp, ref_joints_deg=joints_deg)
        self.session.execute_primitive(
            "MoveL",
            {
                "target": flexivrdk.Coord(*coord_args),
                "vel": float(vel_m_s),
                "zoneRadius": str(self.params["MOVE_ZONE_RADIUS"]),
            },
        )
        self.session.wait_for_primitive("reachedTarget")

    def move_c(
        self,
        label: str,
        middle_pose_entry: dict,
        target_pose_entry: dict,
        *,
        vel_m_s: float,
        acc_m_s2: float,
        jerk_m_s3: float,
        equal_radius: float,
    ) -> None:
        self.logger.info(
            f"MoveC -> {label} (mid {_fmt_pose_mm(middle_pose_entry)} -> "
            f"target {_fmt_pose_mm(target_pose_entry)})"
        )
        if self.dry_run:
            return
        if self.session is None:
            raise RecipeError("robot session is not available")
        flexivrdk = self.session.flexivrdk
        _, state = self.session.selected_arm_state()
        ref_joints_deg = joints_to_jpos_deg(getattr(state, "q", []))
        middle_coord = flexivrdk.Coord(
            *tcp_pose_to_coord_args(
                middle_pose_entry["tcp_pose_world"]["values"],
                ref_joints_deg=ref_joints_deg,
            )
        )
        target_coord = flexivrdk.Coord(
            *tcp_pose_to_coord_args(
                target_pose_entry["tcp_pose_world"]["values"],
                ref_joints_deg=ref_joints_deg,
            )
        )
        self.session.execute_primitive(
            "MoveC",
            {
                "middlePose": middle_coord,
                "target": target_coord,
                "vel": float(vel_m_s),
                "targetTolerLevel": int(self.params["MOVE_TARGET_TOLER_LEVEL"]),
                "acc": float(acc_m_s2),
                "jerk": float(jerk_m_s3),
                "equalRadius": float(equal_radius),
            },
        )
        self.session.wait_for_primitive("reachedTarget")

    def move_current_z_offset(self, label: str, dz_m: float) -> None:
        if self.dry_run:
            self.logger.info(
                f"MoveL -> {label}: current world Z offset {dz_m*1000:.1f} mm"
            )
            return
        if self.session is None:
            raise RecipeError("robot session is not available")
        pose = current_pose_offset_z(self.session, dz_m)
        self.move_l(label, pose, vel_m_s=float(self.params["CARTESIAN_RETREAT_VEL_M_S"]))

    def insert_comp(self, label: str, *, reference_pose: dict | None = None) -> None:
        params = insert_comp_params(self.params, reference_pose=reference_pose)
        configured_axis = str(self.params["INSERTCOMP_INSERT_AXIS"])
        if configured_axis == "AUTO_WORLD_NEG_Z":
            self.logger.info(
                f"InsertComp axis: world -Z resolved to TCP {params['insertAxis']}"
            )
        self.logger.info(f"InsertComp -> {label}: {params}")
        if self.dry_run:
            self.logger.info("InsertComp wait: isMoving == 1, then isMoving == 0")
            return
        if self.session is None:
            raise RecipeError("robot session is not available")
        self.session.execute_primitive("InsertComp", params)
        try:
            self.session.wait_for_primitive_state(
                "isMoving",
                True,
                timeout_s=float(self.params["INSERTCOMP_START_TIMEOUT_S"]),
            )
        except TimeoutError as exc:
            current_is_moving = primitive_state_lookup(self.session.robot, "isMoving")
            current_terminated = primitive_state_lookup(self.session.robot, "terminated")
            current_insert_dis = primitive_state_lookup(self.session.robot, "insertDis")
            extra = []
            extra.append(f"isMoving={current_is_moving}")
            extra.append(f"terminated={current_terminated}")
            if current_insert_dis is not None:
                try:
                    extra.append(f"insertDis={float(current_insert_dis):.6f} m")
                except Exception:
                    extra.append(f"insertDis={current_insert_dis}")
            raise RecipeError(
                "InsertComp did not appear to start motion before timeout; "
                + ", ".join(extra)
                + ". If the part is preloaded against the vise already, try "
                "increasing INSERTCOMP_MAX_CONTACT_FORCE_N and/or "
                "INSERTCOMP_DEADBAND_SCALE, or back off the pre-insert pose slightly."
            ) from exc
        self.session.wait_for_primitive_state(
            "isMoving",
            False,
            timeout_s=float(self.params["INSERTCOMP_TIMEOUT_S"]),
        )

    def gripper(
        self,
        action: str,
        *,
        force_n: float | None = None,
        width_m: float | None = None,
    ) -> None:
        if action == "open":
            width = float(self.params["GRIPPER_OPEN_WIDTH_M"])
            force = 10.0
        elif action == "release":
            width = float(self.params["GRIPPER_RELEASE_WIDTH_M"])
            force = 10.0
        elif action == "close":
            width = float(self.params["GRIPPER_CLOSE_WIDTH_M"])
            force = float(self.params["GRIPPER_FORCE_N"])
        else:
            raise RecipeError(f"unknown gripper action {action!r}")
        if width_m is not None:
            width = float(width_m)
        if force_n is not None:
            force = float(force_n)
        self.logger.info(f"Gripper {action}: width={width:.3f}m force={force:.1f}N")
        settle_s = 0.0
        if action == "close":
            settle_s = float(self.params["GRIPPER_SETTLE_S"])
        elif action == "open":
            settle_s = float(self.params["GRIPPER_OPEN_SETTLE_S"])
        if action == "close":
            self.logger.info(
                f"Gripper close hold: wait until stopped, then settle {settle_s:.1f}s"
            )
        elif action == "open" and settle_s > 0.0:
            self.logger.info(
                f"Gripper open hold: wait until stopped, then settle {settle_s:.1f}s"
            )
        if self.dry_run:
            return
        if self.session is None or self.session.gripper is None:
            raise RecipeError("gripper is not initialized")
        velocity_m_s = self.effective_gripper_velocity_m_s(for_graspcomp=False)
        helper_gripper_set(
            self.session.gripper,
            width,
            vel_m_s=velocity_m_s,
            force_n=force,
            settle_after_s=settle_s,
        )
        if action == "close":
            self.logger.info("Gripper close settled; continuing robot motion")
        elif action == "open" and settle_s > 0.0:
            self.logger.info("Gripper open settled; continuing robot motion")

    def hold_current_joints(self, reason: str) -> None:
        self.logger.info(f"Joint hold -> {reason}")
        if self.dry_run:
            return
        if self.session is None:
            raise RecipeError("robot session is not available")
        self.session.hold_current_joints()

    def switch_to_primitive_mode(self, reason: str) -> None:
        self.logger.info(f"Primitive mode -> {reason}")
        if self.dry_run:
            return
        if self.session is None:
            raise RecipeError("robot session is not available")
        self.session.switch_mode("NRT_PRIMITIVE_EXECUTION")

    def refresh_pose(self, name: str) -> None:
        key_dir = _script_path(str(self.params["KEY_POSITION_DIR"]))
        self.poses[name] = to_pose_entry(load_key_position(key_dir, name))

    def tag_cali_args(self):
        return SimpleNamespace(
            robot_sn=str(self.params["ROBOT_SN"]),
            injectable_camera_serial=self.params["INJECTABLE_CAMERA_SERIAL"],
            injectable_width=int(self.params["INJECTABLE_WIDTH"]),
            injectable_height=int(self.params["INJECTABLE_HEIGHT"]),
            injectable_fps=int(self.params["INJECTABLE_FPS"]),
            injectable_warmup_frames=int(self.params["INJECTABLE_WARMUP_FRAMES"]),
            injectable_exposure=self.params["INJECTABLE_EXPOSURE"],
            injectable_gain=self.params["INJECTABLE_GAIN"],
            injectable_white_balance=self.params["INJECTABLE_WHITE_BALANCE"],
            cali_vise_key_position=str(self.params["VISE_CALI_TARGET_KEY_POSITION"]),
            cali_vise_board=str(self.params["VISE_CALI_BOARD"]),
            cali_vise_calibration=str(self.params["VISE_CALI_CALIBRATION"]),
            cali_vise_reference=str(self.params["VISE_CALI_REFERENCE"]),
            cali_vise_capture_out=str(self.params["VISE_CALI_CAPTURE_OUT"]),
            cali_vise_detect_out=str(self.params["VISE_CALI_DETECT_OUT"]),
            cali_vise_min_charuco_corners=int(
                self.params["VISE_CALI_MIN_CHARUCO_CORNERS"]
            ),
        )

    def calibrate_tag(self, tag_id: int, *, refresh_names: tuple[str, ...]) -> bool:
        key_dir = _script_path(str(self.params["KEY_POSITION_DIR"]))
        max_attempts = max(1, int(self.params["TAG_CALI_MAX_ATTEMPTS"]))
        retry_settle_s = max(0.0, float(self.params["TAG_CALI_RETRY_SETTLE_S"]))
        self.logger.info(
            f"Cali-tag{tag_id} -> "
            f"refresh={', '.join(refresh_names)} keyDir="
            f"{key_dir} attempts={max_attempts}"
        )
        if self.dry_run:
            return True
        if self.session is None:
            raise RecipeError("robot session is not available")
        backups = {
            name: load_key_position(key_dir, name)
            for name in refresh_names
        }
        record_robot_waypoints = load_record_robot_waypoints_module()
        last_error: Exception | None = None
        for attempt in range(1, max_attempts + 1):
            self.logger.info(f"Cali-tag{tag_id} attempt {attempt}/{max_attempts}")
            try:
                record_robot_waypoints.cali_tag(
                    self.session.robot,
                    key_dir,
                    tag_id,
                    self.tag_cali_args(),
                    self.logger,
                )
                for name in refresh_names:
                    self.refresh_pose(name)
                    self.logger.info(f"Reloaded relocalized {name} pose from disk")
                return True
            except Exception as exc:  # noqa: BLE001 - retry transient tag-calibration failures
                last_error = exc
                self.logger.warn(
                    f"Cali-tag{tag_id} attempt {attempt}/{max_attempts} failed: {exc}"
                )
                for name, backup_record in backups.items():
                    restore_path = _local_position_file(key_dir, name)
                    write_key_position_record(restore_path, backup_record)
                    self.poses[name] = to_pose_entry(backup_record)
                if attempt < max_attempts and retry_settle_s > 0.0:
                    self.logger.info(
                        f"Waiting {retry_settle_s:.2f}s before retrying tag calibration"
                    )
                    time.sleep(retry_settle_s)
        if last_error is not None and is_tag_calibration_vision_failure(last_error):
            self.logger.warn(
                f"Cali-tag{tag_id} failed after {max_attempts} attempts; "
                f"keeping saved poses for {', '.join(refresh_names)}"
            )
            return False
        if last_error is not None:
            raise RecipeError(
                f"Cali-tag{tag_id} failed after {max_attempts} attempts: {last_error}"
            ) from last_error
        raise RecipeError(f"Cali-tag{tag_id} failed without a reported error")

    def record_robot_waypoints_args(self):
        return SimpleNamespace(
            injectable_camera_serial=self.params["INJECTABLE_CAMERA_SERIAL"],
            injectable_width=int(self.params["INJECTABLE_WIDTH"]),
            injectable_height=int(self.params["INJECTABLE_HEIGHT"]),
            injectable_fps=int(self.params["INJECTABLE_FPS"]),
            injectable_warmup_frames=int(self.params["INJECTABLE_WARMUP_FRAMES"]),
            injectable_exposure=self.params["INJECTABLE_EXPOSURE"],
            injectable_gain=self.params["INJECTABLE_GAIN"],
            injectable_white_balance=self.params["INJECTABLE_WHITE_BALANCE"],
            injectable_capture_out=str(self.params["INJECTABLE_CAPTURE_OUT"]),
            injectable_align_out=str(self.params["INJECTABLE_ALIGN_OUT"]),
            injectable_calibration=str(self.params["INJECTABLE_CALIBRATION"]),
            injectable_align_primitive=str(self.params["INJECTABLE_ALIGN_PRIMITIVE"]),
            injectable_align_linear_vel=float(
                self.params["INJECTABLE_ALIGN_LINEAR_VEL_M_S"]
            ),
            tool_move_zone_radius=str(self.params["MOVE_ZONE_RADIUS"]),
            tool_move_jnt_vel_scale=int(self.params["MOVE_JNT_VEL_SCALE"]),
            tool_move_target_toler_level=int(self.params["MOVE_TARGET_TOLER_LEVEL"]),
            tool_move_jnt_acc_multiplier=float(
                self.params["MOVE_JNT_ACC_MULTIPLIER"]
            ),
            ortho_jnt_vel_scale=int(self.params["MOVE_JNT_VEL_SCALE"]),
            ortho_zone_radius=str(self.params["MOVE_ZONE_RADIUS"]),
            ortho_target_toler_level=int(self.params["MOVE_TARGET_TOLER_LEVEL"]),
            ortho_jnt_acc_multiplier=float(
                self.params["MOVE_JNT_ACC_MULTIPLIER"]
            ),
            gripper_velocity=self.effective_gripper_velocity_m_s(for_graspcomp=True),
            open_force_limit=float(self.params["GRASP_OPEN_FORCE_LIMIT_N"]),
            gripper_name=str(self.params["GRIPPER_NAME"]),
            poll_sec=0.2,
        )

    def configured_record_robot_waypoints(self):
        record_robot_waypoints = load_record_robot_waypoints_module()
        record_robot_waypoints.DEFAULT_VELOCITY = self.effective_gripper_velocity_m_s(
            for_graspcomp=False
        )
        record_robot_waypoints.MAX_GRASPCOMP_GRIP_VEL = 0.05
        record_robot_waypoints.DEFAULT_INJECTABLE_ALIGN_OFFSET_TOWARD_PINK_M = float(
            self.params["INJECTABLE_ALIGN_OFFSET_TOWARD_PINK_M"]
        )
        record_robot_waypoints.DEFAULT_GRASP_CONTACT_FORCE = float(
            self.params["GRASP_CONTACT_FORCE_N"]
        )
        record_robot_waypoints.DEFAULT_GRASP_PRECONTACT_MOVE_Z_M = float(
            self.params["GRASP_PRECONTACT_MOVE_Z_M"]
        )
        record_robot_waypoints.DEFAULT_GRASP_PRECONTACT_MOVE_VEL = float(
            self.params["GRASP_PRECONTACT_MOVE_VEL_M_S"]
        )
        record_robot_waypoints.DEFAULT_GRASP_CONTACT_VEL = float(
            self.params["GRASP_CONTACT_VEL_M_S"]
        )
        detector = record_robot_waypoints.load_injectable_detector_module()
        detector.apply_detector_config(
            teal_low=self.params["INJECTABLE_TEAL_LOW_HSV"],
            teal_high=self.params["INJECTABLE_TEAL_HIGH_HSV"],
            pink_low=self.params["INJECTABLE_PINK_LOW_HSV"],
            pink_high=self.params["INJECTABLE_PINK_HIGH_HSV"],
            min_pink_area=int(self.params["INJECTABLE_MIN_PINK_AREA"]),
            min_teal_area=int(self.params["INJECTABLE_MIN_TEAL_AREA"]),
            dist_tolerance=float(self.params["INJECTABLE_DIST_TOLERANCE"]),
            axis_cos_min=float(self.params["INJECTABLE_AXIS_COS_MIN"]),
        )
        return record_robot_waypoints

    def align_injectable(self) -> None:
        target_index = int(self.params["INJECTABLE_ALIGN_TARGET_INDEX"])
        max_attempts = max(1, int(self.params["INJECTABLE_ALIGN_MAX_ATTEMPTS"]))
        retry_settle_s = max(
            0.0, float(self.params["INJECTABLE_ALIGN_RETRY_SETTLE_S"])
        )
        frame_timeout_s = max(
            0.1, float(self.params["INJECTABLE_ALIGN_FRAME_TIMEOUT_S"])
        )
        if self.dry_run:
            self.logger.info(
                "Align Injectable -> "
                f"index={target_index} "
                f"offsetTowardPink={float(self.params['INJECTABLE_ALIGN_OFFSET_TOWARD_PINK_M']) * 1000.0:.1f} mm "
                f"primitive={str(self.params['INJECTABLE_ALIGN_PRIMITIVE']).upper()} "
                f"vel={float(self.params['INJECTABLE_ALIGN_LINEAR_VEL_M_S']):.3f} m/s "
                f"attempts={max_attempts}"
            )
            return
        if self.session is None:
            raise RecipeError("robot session is not available")
        record_robot_waypoints = self.configured_record_robot_waypoints()
        if self.camera_session is None:
            last_error: Exception | None = None
            for attempt in range(1, max_attempts + 1):
                self.logger.info(
                    f"Align Injectable attempt {attempt}/{max_attempts} via fresh camera capture"
                )
                try:
                    record_robot_waypoints.align_injectable(
                        self.session.robot,
                        _script_path(str(self.params["KEY_POSITION_DIR"])),
                        self.record_robot_waypoints_args(),
                        self.logger,
                        target_index=target_index,
                    )
                    return
                except Exception as exc:  # noqa: BLE001 - retry on transient vision failures
                    last_error = exc
                    is_last_attempt = attempt >= max_attempts
                    self.logger.warn(
                        f"Align Injectable attempt {attempt}/{max_attempts} failed: {exc}"
                    )
                    if is_last_attempt:
                        break
                    if retry_settle_s > 0.0:
                        self.logger.info(
                            f"Waiting {retry_settle_s:.2f}s before retrying alignment capture"
                        )
                        time.sleep(retry_settle_s)
            if last_error is not None:
                raise RecipeError(
                    f"Align Injectable failed after {max_attempts} attempts: {last_error}"
                ) from last_error
            raise RecipeError("Align Injectable failed without a reported error")
        detector = record_robot_waypoints.load_injectable_detector_module()
        handeye = record_robot_waypoints.load_handeye_module()
        calibration_tcp_camera = np.asarray(
            handeye.read_yaml(Path(self.params["INJECTABLE_CALIBRATION"]))[
                "T_tcp_camera"
            ],
            dtype=float,
        ).reshape(4, 4)
        last_error: Exception | None = None
        for attempt in range(1, max_attempts + 1):
            self.logger.info(
                f"Align Injectable attempt {attempt}/{max_attempts} via shared camera session"
            )
            capture_not_before = time.time() + retry_settle_s
            try:
                color_image, depth_image, intrinsics, depth_scale = (
                    self.camera_session.get_latest_aligned_rgbd(
                        timeout_s=retry_settle_s + frame_timeout_s,
                        min_timestamp=capture_not_before,
                    )
                )
                capture_path = record_robot_waypoints.save_injectable_image(
                    str(self.params["INJECTABLE_CAPTURE_OUT"]),
                    color_image,
                )
                self.logger.info(f"Captured -> {capture_path}")
                result, detections = detector.detect(color_image)
                detections = record_robot_waypoints.sort_injectable_detections(detections)
                record_robot_waypoints.print_injectable_detections(
                    detections, self.logger
                )
                if not detections:
                    detect_path = record_robot_waypoints.save_injectable_image(
                        str(self.params["INJECTABLE_ALIGN_OUT"]),
                        result,
                    )
                    raise RecipeError(
                        f"No injectable detections found. Saved annotation -> {detect_path}"
                    )
                if target_index < 1 or target_index > len(detections):
                    raise RecipeError(
                        f"Injectable index {target_index} is out of range 1..{len(detections)}"
                    )
                selected = detections[target_index - 1]
                annotated = record_robot_waypoints.annotate_selected_injectable(
                    result,
                    selected,
                    target_index,
                )
                detect_path = record_robot_waypoints.save_injectable_image(
                    str(self.params["INJECTABLE_ALIGN_OUT"]),
                    annotated,
                )
                self.logger.info(f"Saved -> {detect_path}")
                _, state = self.session.selected_arm_state()
                current_tcp = [float(value) for value in getattr(state, "tcp_pose", [])]
                target = record_robot_waypoints.compute_planar_injectable_target(
                    selected,
                    depth_image,
                    depth_scale,
                    intrinsics,
                    current_tcp,
                    calibration_tcp_camera,
                )
                target_tcp = target["target_tcp"]
                self.logger.info(
                    "Planar alignment target: "
                    f"dX={target['delta_xy_mm'][0]:+.1f} mm, "
                    f"dY={target['delta_xy_mm'][1]:+.1f} mm, "
                    f"dRot={target['rotation_deg']:.1f} deg, "
                    f"center depth={target['center_depth_m'] * 1000.0:.1f} mm, "
                    f"offsetTowardPink={target['offset_toward_pink_mm']:.1f} mm, "
                    "tcp+X->green/teal"
                )
                self.logger.info(
                    "Target TCP [x y z qw qx qy qz]: "
                    f"{[round(value, 6) for value in target_tcp]}"
                )
                record_robot_waypoints.execute_injectable_alignment_move(
                    self.session.robot,
                    target_tcp,
                    self.record_robot_waypoints_args(),
                    self.logger,
                )
                return
            except Exception as exc:  # noqa: BLE001 - retry on transient vision failures
                last_error = exc
                self.logger.warn(
                    f"Align Injectable attempt {attempt}/{max_attempts} failed: {exc}"
                )
                if attempt >= max_attempts:
                    break
        if last_error is not None:
            raise RecipeError(
                f"Align Injectable failed after {max_attempts} attempts: {last_error}"
            ) from last_error
        raise RecipeError("Align Injectable failed without a reported error")

    def grasp_injectable(self) -> None:
        width_m = float(self.params["GRASP_OPEN_WIDTH_M"])
        force_n = float(self.params["GRASP_FORCE_N"])
        if self.dry_run:
            self.logger.info(
                "Grasp Injectable -> "
                f"openWidth={width_m:.3f} m force={force_n:.1f} N "
                f"precontact={float(self.params['GRASP_PRECONTACT_MOVE_Z_M']):.3f} m @ "
                f"{float(self.params['GRASP_PRECONTACT_MOVE_VEL_M_S']):.3f} m/s "
                f"contact={float(self.params['GRASP_CONTACT_VEL_M_S']):.3f} m/s "
                f"maxForce={float(self.params['GRASP_CONTACT_FORCE_N']):.1f} N"
            )
            return
        if self.session is None or self.session.gripper is None:
            raise RecipeError("gripper is not initialized")
        record_robot_waypoints = self.configured_record_robot_waypoints()
        record_robot_waypoints.execute_adaptive_grasp(
            self.session.robot,
            self.session.gripper,
            width_m,
            force_n,
            self.record_robot_waypoints_args(),
            self.logger,
            ensure_gripper_initialized=False,
        )

    def twist_cap(self) -> None:
        twist_deg = float(self.params["CAP_TWIST_DEG"])
        repeat_count = int(self.params["CAP_TWIST_REPEAT_COUNT"])
        self.logger.info(
            f"Twist cap: TCP X {twist_deg:+.1f} deg then back to 0 repeat={repeat_count}"
        )
        if self.dry_run:
            return
        if self.session is None:
            raise RecipeError("robot session is not available")
        _, state = self.session.selected_arm_state()
        start_q_rad = [float(v) for v in getattr(state, "q", [])]
        start_tcp = [float(v) for v in getattr(state, "tcp_pose", [])]
        if len(start_q_rad) != 7 or len(start_tcp) != 7:
            raise RecipeError(
                f"current arm state has q={len(start_q_rad)}, tcp={len(start_tcp)}; "
                "expected 7 each"
            )
        start_pose = {
            "name": "twist_start",
            "q_rad": start_q_rad,
            "tcp_pose_world": {
                "order": ["x", "y", "z", "qw", "qx", "qy", "qz"],
                "values": start_tcp,
            },
        }
        for index in range(repeat_count):
            label = f"twist_{index + 1}"
            twist_pose = {
                "name": label,
                "q_rad": start_q_rad,
                "tcp_pose_world": {
                    "order": ["x", "y", "z", "qw", "qx", "qy", "qz"],
                    "values": rotate_tcp_about_tool_axis(
                        start_tcp,
                        roll_deg=twist_deg,
                    ),
                },
            }
            self.move_ptp(label, twist_pose, use_ref_joints=True)
            self.move_ptp(f"twist_{index + 1}_return", start_pose, use_ref_joints=True)

    def dump_tool_z(self) -> None:
        dump_deg = float(self.params["DUMP_TOOL_Z_DEG"])
        pivot_offset = [
            float(self.params["DUMP_VP_OFFSET_X_M"]),
            float(self.params["DUMP_VP_OFFSET_Y_M"]),
            float(self.params["DUMP_VP_OFFSET_Z_M"]),
        ]
        vel_m_s = float(self.params["DUMP_MOVEC_VEL_M_S"])
        acc_m_s2 = float(self.params["DUMP_MOVEC_ACC_M_S2"])
        jerk_m_s3 = float(self.params["DUMP_MOVEC_JERK_M_S3"])
        equal_radius = float(self.params["DUMP_MOVEC_EQUAL_RADIUS"])
        self.logger.info(
            "Dump motion: MoveC around tool-frame virtual pivot "
            f"[x={pivot_offset[0]*1000:.1f}, y={pivot_offset[1]*1000:.1f}, "
            f"z={pivot_offset[2]*1000:.1f}] mm with tool Z {dump_deg:+.1f} deg "
            "then return"
        )
        if self.dry_run:
            start_q_rad = list(self.poses["Glass"]["q_rad"])
            start_tcp = list(self.poses["Glass"]["tcp_pose_world"]["values"])
        else:
            if self.session is None:
                raise RecipeError("robot session is not available")
            _, state = self.session.selected_arm_state()
            start_q_rad = [float(v) for v in getattr(state, "q", [])]
            start_tcp = [float(v) for v in getattr(state, "tcp_pose", [])]
            if len(start_q_rad) != 7 or len(start_tcp) != 7:
                raise RecipeError(
                    f"current arm state has q={len(start_q_rad)}, tcp={len(start_tcp)}; "
                    "expected 7 each"
                )
        start_pose = {
            "name": "dump_start",
            "q_rad": start_q_rad,
            "tcp_pose_world": {
                "order": ["x", "y", "z", "qw", "qx", "qy", "qz"],
                "values": start_tcp,
            },
        }
        middle_tcp = rotate_tcp_about_virtual_offset(
            start_tcp,
            offset_m_xyz=pivot_offset,
            yaw_deg=dump_deg / 2.0,
        )
        target_tcp = rotate_tcp_about_virtual_offset(
            start_tcp,
            offset_m_xyz=pivot_offset,
            yaw_deg=dump_deg,
        )
        if not movec_geometry_is_valid(start_tcp, middle_tcp, target_tcp):
            raise RecipeError(
                "Dump MoveC geometry is invalid; adjust DUMP_VP_OFFSET_* params"
            )
        middle_pose = {
            "name": "dump_tool_z_mid",
            "q_rad": list(start_q_rad),
            "tcp_pose_world": {
                "order": ["x", "y", "z", "qw", "qx", "qy", "qz"],
                "values": middle_tcp,
            },
        }
        target_pose = {
            "name": "dump_tool_z_target",
            "q_rad": list(start_q_rad),
            "tcp_pose_world": {
                "order": ["x", "y", "z", "qw", "qx", "qy", "qz"],
                "values": target_tcp,
            },
        }
        if self.dry_run:
            self.move_c(
                "dump_tool_z",
                middle_pose,
                target_pose,
                vel_m_s=vel_m_s,
                acc_m_s2=acc_m_s2,
                jerk_m_s3=jerk_m_s3,
                equal_radius=equal_radius,
            )
            self.move_c(
                "dump_tool_z_return",
                middle_pose,
                start_pose,
                vel_m_s=vel_m_s,
                acc_m_s2=acc_m_s2,
                jerk_m_s3=jerk_m_s3,
                equal_radius=equal_radius,
            )
            return
        self.move_c(
            "dump_tool_z",
            middle_pose,
            target_pose,
            vel_m_s=vel_m_s,
            acc_m_s2=acc_m_s2,
            jerk_m_s3=jerk_m_s3,
            equal_radius=equal_radius,
        )
        _, state = self.session.selected_arm_state()
        return_seed_q_rad = [float(v) for v in getattr(state, "q", [])]
        return_pose = {
            "name": "dump_tool_z_return",
            "q_rad": list(return_seed_q_rad),
            "tcp_pose_world": {
                "order": ["x", "y", "z", "qw", "qx", "qy", "qz"],
                "values": start_tcp,
            },
        }
        self.move_c(
            "dump_tool_z_return",
            middle_pose,
            return_pose,
            vel_m_s=vel_m_s,
            acc_m_s2=acc_m_s2,
            jerk_m_s3=jerk_m_s3,
            equal_radius=equal_radius,
        )


def require_arduino(ctx: RecipeContext):
    if ctx.arduino is None:
        raise RecipeError("Arduino is required for this phase")
    return ctx.arduino


def is_unsupported_get_status_error(exc: Exception) -> bool:
    if not isinstance(exc, ArduinoCommandError):
        return False
    return (
        exc.command == "GET_STATUS"
        and str(exc.code).upper() == "INVALID_ARG"
        and "unknown command" in str(exc.message).lower()
    )


def get_arduino_status_or_none(ctx: RecipeContext) -> dict | None:
    arduino = require_arduino(ctx)
    if ctx.arduino_status_query_supported is False:
        return None
    try:
        status = arduino.get_status()
        ctx.arduino_status_query_supported = True
        return status
    except Exception as exc:  # noqa: BLE001 - firmware compatibility handling
        if is_unsupported_get_status_error(exc):
            ctx.arduino_status_query_supported = False
            ctx.logger.warn(
                "Arduino firmware does not support GET_STATUS; "
                "skipping machine status prechecks and assuming the machine is "
                "already prepared. HOME_ALL cannot be auto-issued in this mode."
            )
            return None
        raise


def ensure_arduino_ready(ctx: RecipeContext) -> dict:
    arduino = require_arduino(ctx)
    status = get_arduino_status_or_none(ctx)
    if status is None:
        return {"status_query_unavailable": True}
    if status.get("faulted"):
        raise RecipeError(f"Arduino is faulted: {status}")
    if status.get("busy"):
        raise RecipeError(f"Arduino is busy: {status}")
    if status.get("blade_on"):
        raise RecipeError(f"Arduino blade is on before motion: {status}")
    if not status.get("homed"):
        ctx.logger.info("Arduino not homed; running HOME_ALL")
        status = arduino.home_all()
    return status


def prepare_arduino_for_vise_action(ctx: RecipeContext, *, action_label: str) -> dict:
    arduino = require_arduino(ctx)
    status = get_arduino_status_or_none(ctx)
    if status is not None:
        if status.get("faulted"):
            raise RecipeError(f"Arduino is faulted before {action_label}: {status}")
        if status.get("busy"):
            raise RecipeError(f"Arduino is busy before {action_label}: {status}")
        if status.get("blade_on"):
            raise RecipeError(f"Arduino blade is on before {action_label}: {status}")
    ctx.logger.info(f"Arduino vise prep before {action_label}: running HOME_ALL")
    home_result = arduino.home_all()
    if home_result.get("faulted"):
        raise RecipeError(f"HOME_ALL faulted before {action_label}: {home_result}")
    if home_result.get("busy"):
        raise RecipeError(f"HOME_ALL left Arduino busy before {action_label}: {home_result}")
    if home_result.get("blade_on"):
        raise RecipeError(
            f"HOME_ALL left Arduino blade_on before {action_label}: {home_result}"
        )
    return home_result


def validate_vise_closed(result: dict, target_force_kg: float) -> None:
    if result.get("faulted"):
        raise RecipeError(f"CLOSE_VISE returned faulted state: {result}")
    if result.get("vise_state") != "CLOSED":
        raise RecipeError(f"expected vise_state=CLOSED, got {result}")
    force = float(result.get("force_kg", 0.0))
    if force < float(target_force_kg) * 0.75:
        raise RecipeError(
            f"vise force {force:.2f}kg is unexpectedly below target {target_force_kg:.2f}kg"
        )


def validate_vise_open(result: dict, target_force_kg: float) -> None:
    if result.get("faulted"):
        raise RecipeError(f"OPEN_VISE returned faulted state: {result}")
    if result.get("vise_state") != "OPEN":
        raise RecipeError(f"expected vise_state=OPEN, got {result}")
    force = float(result.get("force_kg", target_force_kg))
    if force > max(float(target_force_kg) * 2.0, float(target_force_kg) + 0.5):
        raise RecipeError(
            f"vise force {force:.2f}kg is unexpectedly above open target {target_force_kg:.2f}kg"
        )


def validate_cut_done(result: dict, *, rot_safe_tol_deg: float) -> None:
    problems: list[str] = []
    if result.get("blade_on"):
        problems.append("blade_on is true")
    if result.get("busy"):
        problems.append("busy is true")
    if result.get("faulted"):
        problems.append("faulted is true")
    for field in ("x_mm", "z_mm"):
        if abs(float(result.get(field, 0.0))) > 1e-3:
            problems.append(f"{field}={result.get(field)}")
    rot_deg = float(result.get("rot_deg", 0.0))
    if abs(rot_deg) > float(rot_safe_tol_deg):
        problems.append(f"rot_deg={result.get('rot_deg')} > tol={rot_safe_tol_deg}")
    if problems:
        raise RecipeError("CUT_HEIGHT did not finish in safe state: " + ", ".join(problems))


def transit_pose(ctx: RecipeContext) -> tuple[str, dict]:
    return "Middle", ctx.poses["Middle"]


def pickup_stage_pose(ctx: RecipeContext) -> tuple[str, dict]:
    return "Plate", ctx.poses["Plate"]


def above_vise_pose(ctx: RecipeContext) -> dict:
    return offset_pose_z(ctx.poses["Vise"], float(ctx.params["VISE_APPROACH_Z_OFFSET_M"]))


def vise_offset_pose(ctx: RecipeContext, dz_m: float) -> dict:
    return ctx.poses["Vise"] if float(dz_m) == 0.0 else offset_pose_z(ctx.poses["Vise"], dz_m)


def cap_grip_pose(ctx: RecipeContext) -> dict:
    return vise_offset_pose(ctx, float(ctx.params["CAP_GRIP_Z_OFFSET_M"]))


def resolve_drop_pose(ctx: RecipeContext, target_name: str) -> tuple[str, dict, bool]:
    if target_name in ctx.poses:
        return target_name, ctx.poses[target_name], False
    raise RecipeError(f"drop target {target_name!r} is not available in loaded key positions")


def removal_cycle_for_step(step_number: int) -> tuple[dict, str] | None:
    for cycle in REMOVAL_CYCLES:
        for action, cycle_step in dict(cycle["steps"]).items():
            if int(cycle_step) == step_number:
                return cycle, action
    return None


def shell_glass_step_offset(step_number: int) -> int | None:
    if step_number in SHELL_GLASS_STEP_SEQUENCE:
        return SHELL_GLASS_STEP_SEQUENCE.index(step_number)
    return None


def step_requires_arduino(step_number: int) -> bool:
    return step_number in {10, 13, 41}


def step_uses_gripper(step_number: int) -> bool:
    return step_number in {
        1,
        5,
        11,
        12,
        15,
        20,
        24,
        28,
        32,
        36,
        40,
        47,
    }


def validate_plan_step(step_number: int) -> int:
    if step_number not in PLAN_STEPS:
        valid_steps = ", ".join(str(step) for step in sorted(PLAN_STEPS))
        raise RecipeError(
            f"unknown plan step {step_number}; valid steps are {valid_steps}"
        )
    return step_number


def parse_step_selection(selection) -> tuple[int, ...]:
    text = str(selection).strip()
    if not text:
        raise RecipeError("--step requires a value like '15' or '1-5'")

    if "-" not in text:
        try:
            return (validate_plan_step(int(text)),)
        except ValueError as exc:
            raise RecipeError(
                f"invalid --step value {text!r}; expected a step like '15' or range like '1-5'"
            ) from exc

    start_text, end_text = (part.strip() for part in text.split("-", 1))
    if not start_text or not end_text:
        raise RecipeError(
            f"invalid --step range {text!r}; expected a range like '1-5'"
        )
    try:
        start_step = validate_plan_step(int(start_text))
        end_step = validate_plan_step(int(end_text))
    except ValueError as exc:
        raise RecipeError(
            f"invalid --step range {text!r}; expected integers like '1-5'"
        ) from exc
    if start_step > end_step:
        raise RecipeError(
            f"invalid --step range {text!r}; start step must be <= end step"
        )
    selected = tuple(
        step for step in sorted(PLAN_STEPS) if start_step <= step <= end_step
    )
    if not selected:
        raise RecipeError(
            f"step range {text!r} did not include any valid plan steps"
        )
    return selected


def step_selection_label(step_numbers: tuple[int, ...]) -> str:
    if not step_numbers:
        return "(none)"
    if len(step_numbers) == 1:
        return str(step_numbers[0])
    return f"{step_numbers[0]}-{step_numbers[-1]}"


def execute_plan_step(
    ctx: RecipeContext,
    step_number: int,
    *,
    yes: bool = False,
    skip_cut: bool = False,
) -> None:
    step_number = validate_plan_step(step_number)
    ctx.logger.info(f"=== step {step_number}: {PLAN_STEPS[step_number]} ===")

    removal_step = removal_cycle_for_step(step_number)
    if removal_step is not None:
        cycle, action = removal_step
        name = str(cycle["name"])
        approach_z_m = float(ctx.params[str(cycle["approach_z_param"])])
        grip_z_m = float(ctx.params[str(cycle["grip_z_param"])])
        force_n = float(ctx.params[str(cycle["force_param"])])
        lift_z_m = float(ctx.params[str(cycle["lift_z_param"])])
        if action == "transit":
            transit_label, transit_entry = transit_pose(ctx)
            ctx.move_ptp(transit_label, transit_entry)
        elif action == "above_vise":
            ctx.move_ptp(
                f"{name}_above_vise",
                vise_offset_pose(ctx, approach_z_m),
            )
        elif action == "grip":
            ctx.move_l(
                f"{name}_grip",
                vise_offset_pose(ctx, grip_z_m),
                vel_m_s=float(ctx.params["CARTESIAN_INSERT_VEL_M_S"]),
            )
        elif action == "close":
            ctx.gripper("close", force_n=force_n)
        elif action == "lift":
            ctx.move_current_z_offset(f"{name}_lift", lift_z_m)
        elif action == "return_transit":
            transit_label, transit_entry = transit_pose(ctx)
            ctx.move_ptp(transit_label, transit_entry)
        elif action == "drop":
            drop_label, drop_pose, _ = resolve_drop_pose(ctx, str(cycle["drop_target"]))
            ctx.move_ptp(drop_label, drop_pose)
        elif action == "open":
            ctx.gripper("open")
        return

    shell_offset = shell_glass_step_offset(step_number)
    if shell_offset is not None:
        if shell_offset == 0:
            transit_label, transit_entry = transit_pose(ctx)
            ctx.move_ptp(transit_label, transit_entry)
        elif shell_offset == 1:
            ctx.move_ptp(
                "shell_above_vise",
                vise_offset_pose(
                    ctx, float(ctx.params["SHELL_REMOVE_APPROACH_Z_OFFSET_M"])
                ),
            )
        elif shell_offset == 2:
            ctx.move_l(
                "shell_grip",
                vise_offset_pose(
                    ctx, float(ctx.params["SHELL_REMOVE_GRIP_Z_OFFSET_M"])
                ),
                vel_m_s=float(ctx.params["CARTESIAN_INSERT_VEL_M_S"]),
            )
        elif shell_offset == 3:
            ctx.gripper("close", force_n=float(ctx.params["SHELL_REMOVE_FORCE_N"]))
        elif shell_offset == 4:
            prepare_arduino_for_vise_action(ctx, action_label="OPEN_VISE")
            result = require_arduino(ctx).open_vise(
                target_force_kg=float(ctx.params["VISE_OPEN_TARGET_FORCE_KG"])
            )
            validate_vise_open(result, float(ctx.params["VISE_OPEN_TARGET_FORCE_KG"]))
        elif shell_offset == 5:
            ctx.move_current_z_offset(
                "shell_lift",
                float(ctx.params["SHELL_REMOVE_LIFT_Z_OFFSET_M"]),
            )
        elif shell_offset == 6:
            transit_label, transit_entry = transit_pose(ctx)
            ctx.move_ptp(transit_label, transit_entry)
        elif shell_offset == 7:
            ctx.move_ptp("Glass", ctx.poses["Glass"])
        elif shell_offset == 8:
            ctx.dump_tool_z()
        elif shell_offset == 9:
            ctx.move_ptp("Plastic", ctx.poses["Plastic"])
        elif shell_offset == 10:
            ctx.gripper("open")
        elif shell_offset == 11:
            transit_label, transit_entry = transit_pose(ctx)
            ctx.move_ptp(transit_label, transit_entry)
        return

    if step_number == 1:
        ctx.gripper("open")
    elif step_number == 2:
        transit_label, transit_entry = transit_pose(ctx)
        ctx.move_ptp(transit_label, transit_entry)
    elif step_number == 3:
        stage_label, stage_pose = pickup_stage_pose(ctx)
        ctx.move_ptp(stage_label, stage_pose)
    elif step_number == 4:
        ctx.align_injectable()
    elif step_number == 5:
        ctx.grasp_injectable()
    elif step_number == 6:
        ctx.move_current_z_offset(
            "pickup_lift",
            float(ctx.params["PICKUP_LIFT_Z_OFFSET_M"]),
        )
    elif step_number == 7:
        transit_label, transit_entry = transit_pose(ctx)
        ctx.move_ptp(transit_label, transit_entry)
    elif step_number == 8:
        ctx.move_ptp(
            "above_vise",
            above_vise_pose(ctx),
        )
    elif step_number == 9:
        above_vise = above_vise_pose(ctx)
        ctx.insert_comp("Vise insert", reference_pose=above_vise)
    elif step_number == 10:
        prepare_arduino_for_vise_action(ctx, action_label="CLOSE_VISE")
        result = require_arduino(ctx).close_vise(
            target_force_kg=float(ctx.params["VISE_TARGET_FORCE_KG"])
        )
        validate_vise_closed(result, float(ctx.params["VISE_TARGET_FORCE_KG"]))
    elif step_number == 11:
        ctx.hold_current_joints("hold current pose after InsertComp before gripper release")
        ctx.gripper("release")
    elif step_number == 12:
        ctx.hold_current_joints("keep robot static while opening gripper fully")
        ctx.gripper("open")
        ctx.switch_to_primitive_mode("resume motion control for vise retreat")
        ctx.move_current_z_offset(
            "vise_retreat",
            float(ctx.params["VISE_RETREAT_Z_OFFSET_M"]),
        )
    elif step_number == 13:
        arduino = require_arduino(ctx)
        status = ensure_arduino_ready(ctx)
        if status.get("status_query_unavailable"):
            ctx.logger.warn(
                "CUT_HEIGHT pre-status unavailable because GET_STATUS is unsupported; "
                "skipping vise-state precheck and assuming the machine is ready."
            )
        else:
            ctx.logger.info(f"CUT_HEIGHT pre-status: {status}")
            if status.get("vise_state") != "CLOSED":
                message = f"cannot cut unless vise is CLOSED: {status}"
                if ctx.dry_run:
                    ctx.logger.warn(f"{message}; dry-run will continue to show CUT_HEIGHT")
                else:
                    raise RecipeError(message)
        ctx.logger.info("CUT_HEIGHT prep: ROTATE_ABS deg=0.0")
        arduino.rotate_abs(0.0)
        if skip_cut:
            ctx.logger.warn("Skipping CUT_HEIGHT because --skip-cut was supplied")
            return
        confirm_cut(ctx, yes=yes)
        ctx.logger.info(
            "CUT_HEIGHT fire: "
            f"z_mm={float(ctx.params['CUT_Z_MM'])} "
            f"x_mm={float(ctx.params['CUT_X_MM'])} "
            f"deg={float(ctx.params['CUT_DEG'])}"
        )
        result = arduino.cut_height(
            z_mm=float(ctx.params["CUT_Z_MM"]),
            x_mm=float(ctx.params["CUT_X_MM"]),
            deg=float(ctx.params["CUT_DEG"]),
        )
        ctx.logger.info(f"CUT_HEIGHT result: {result}")
        validate_cut_done(result, rot_safe_tol_deg=float(ctx.params["ROT_SAFE_TOL_DEG"]))
    elif step_number == 14:
        ctx.move_l(
            "Vise",
            cap_grip_pose(ctx),
            vel_m_s=float(ctx.params["CARTESIAN_INSERT_VEL_M_S"]),
        )
    elif step_number == 15:
        ctx.gripper("close")
    elif step_number == 16:
        ctx.twist_cap()
    elif step_number == 17:
        ctx.move_current_z_offset(
            "cap_lift",
            float(ctx.params["CAP_LIFT_Z_OFFSET_M"]),
        )
    elif step_number == 18:
        transit_label, transit_entry = transit_pose(ctx)
        ctx.move_ptp(transit_label, transit_entry)
    elif step_number == 19:
        ctx.move_ptp("Plastic", ctx.poses["Plastic"])
    elif step_number == 20:
        ctx.gripper("open")


def run_plan_steps(
    ctx: RecipeContext,
    step_numbers,
    *,
    yes: bool = False,
    skip_cut: bool = False,
) -> None:
    for step_number in step_numbers:
        execute_plan_step(ctx, step_number, yes=yes, skip_cut=skip_cut)


def maybe_start_pov_camera_session(ctx: RecipeContext) -> None:
    if ctx.camera_session is not None:
        return
    if ctx.dry_run:
        return
    if not bool(ctx.params["POV_RECORD_ENABLED"]):
        return
    ctx.logger.info("Starting shared RealSense session for POV recording")
    ctx.camera_session = build_camera_session(
        ctx.params,
        ctx.logger,
        enable_recording=True,
    )


def phase_startup_transfer(ctx: RecipeContext) -> None:
    ctx.logger.info("=== phase_startup_transfer ===")
    run_plan_steps(ctx, (1, 2))


def phase_calibrate_tags(ctx: RecipeContext) -> None:
    relocalize_tags = bool(ctx.params["TAG_CALI_ENABLED"])
    if relocalize_tags:
        ctx.logger.info("=== phase_calibrate_tags ===")
    else:
        ctx.logger.info("=== phase_calibrate_tags (staging-only; relocalization disabled) ===")
    for entry in TAG_CALIBRATION_SEQUENCE:
        staging_name = str(ctx.params[str(entry["staging_param"])])
        if staging_name not in ctx.poses:
            raise RecipeError(
                f"calibration staging pose {staging_name!r} is required for tag preflight"
            )
        ctx.move_ptp(staging_name, ctx.poses[staging_name], use_ref_joints=True)
        if relocalize_tags:
            success = ctx.calibrate_tag(
                int(entry["tag_id"]),
                refresh_names=tuple(str(name) for name in entry["refresh_names"]),
            )
            if not success:
                ctx.logger.warn(
                    f"Cali-tag{int(entry['tag_id'])} continuing with saved key positions "
                    f"for {', '.join(str(name) for name in entry['refresh_names'])}"
                )
        else:
            ctx.logger.info(
                f"Cali-tag{int(entry['tag_id'])} skipped; keeping saved key positions "
                f"for {', '.join(str(name) for name in entry['refresh_names'])}"
            )
    transit_label, transit_entry = transit_pose(ctx)
    ctx.move_ptp(transit_label, transit_entry)
    maybe_start_pov_camera_session(ctx)


def phase_pick_from_camera_align_grasp(ctx: RecipeContext) -> None:
    ctx.logger.info("=== phase_pick_from_camera_align_grasp ===")
    run_plan_steps(ctx, (3, 4, 5, 6, 7))


def phase_load_vise(ctx: RecipeContext) -> None:
    ctx.logger.info("=== phase_load_vise ===")
    run_plan_steps(ctx, (8, 9, 10, 11, 12))


def confirm_cut(ctx: RecipeContext, yes: bool) -> None:
    if ctx.dry_run or yes:
        return
    print(
        "\nCUT_HEIGHT will fire the ultrasonic blade. "
        "Type CUT to continue, anything else to abort: ",
        end="",
        flush=True,
    )
    if input().strip() != "CUT":
        raise RecipeError("operator aborted before CUT_HEIGHT")


def confirm_next_frame(
    ctx: RecipeContext,
    *,
    completed_frame: int,
    next_frame: int,
) -> None:
    if ctx.dry_run:
        ctx.logger.info(
            f"[dry-run] frame {completed_frame} complete; auto-advancing to frame {next_frame}"
        )
        return
    print(
        f"\nFrame {completed_frame} complete. "
        f"Move to frame {next_frame}? Type N to continue, anything else to abort: ",
        end="",
        flush=True,
    )
    if input().strip().upper() != "N":
        raise RecipeError(
            f"operator aborted before advancing from frame {completed_frame} to frame {next_frame}"
        )


def phase_cut(ctx: RecipeContext, *, yes: bool, skip_cut: bool = False) -> None:
    ctx.logger.info("=== phase_cut ===")
    execute_plan_step(ctx, 13, yes=yes, skip_cut=skip_cut)


def phase_twist_and_drop_cap(ctx: RecipeContext) -> None:
    ctx.logger.info("=== phase_twist_and_drop_cap ===")
    run_plan_steps(ctx, (14, 15, 16, 17, 18, 19, 20))


def phase_remove_spring(ctx: RecipeContext) -> None:
    ctx.logger.info("=== phase_remove_spring ===")
    run_plan_steps(ctx, (21, 22, 23, 24, 25, 26, 27, 28))


def phase_remove_yellow_plastic(ctx: RecipeContext) -> None:
    ctx.logger.info("=== phase_remove_yellow_plastic ===")
    run_plan_steps(ctx, (29, 30, 31, 32, 33, 34, 35, 36))


def phase_remove_shell(ctx: RecipeContext) -> None:
    ctx.logger.info("=== phase_remove_shell_and_glass ===")
    run_plan_steps(ctx, SHELL_GLASS_STEP_SEQUENCE)


def phase_return_middle(ctx: RecipeContext) -> None:
    ctx.logger.info("=== phase_return_middle ===")
    run_plan_steps(ctx, ())


def phase_shutdown(ctx: RecipeContext) -> None:
    ctx.logger.info("=== phase_shutdown ===")
    try:
        ctx.gripper("open")
    except Exception as exc:  # noqa: BLE001 - shutdown is best effort
        ctx.logger.warn(f"gripper open during shutdown failed: {exc}")
    if ctx.arduino is not None:
        try:
            status = get_arduino_status_or_none(ctx)
            if status is None:
                ctx.logger.warn(
                    "Arduino shutdown status unavailable because GET_STATUS is unsupported; "
                    "skipping idle/blade verification."
                )
            elif status.get("busy") or status.get("blade_on"):
                ctx.logger.warn(f"machine not idle at shutdown; STOP_ALL: {status}")
                ctx.arduino.stop_all()
        except Exception as exc:  # noqa: BLE001
            ctx.logger.warn(f"Arduino shutdown status/stop failed: {exc}")


def run_full_recipe(
    ctx: RecipeContext,
    *,
    yes: bool,
    skip_cut: bool = False,
    uncut: bool = False,
    frame_gated: bool = False,
) -> None:
    phase_startup_transfer(ctx)
    phase_calibrate_tags(ctx)
    if frame_gated:
        confirm_next_frame(ctx, completed_frame=1, next_frame=2)
    phase_pick_from_camera_align_grasp(ctx)
    if frame_gated:
        confirm_next_frame(ctx, completed_frame=2, next_frame=3)
    phase_load_vise(ctx)
    if uncut:
        ctx.logger.info(
            "=== uncut shortcut enabled: after step 12 jump directly to step 37 ==="
        )
        if frame_gated:
            confirm_next_frame(ctx, completed_frame=3, next_frame=5)
        phase_remove_shell(ctx)
    else:
        if frame_gated:
            confirm_next_frame(ctx, completed_frame=3, next_frame=4)
        phase_cut(ctx, yes=yes, skip_cut=skip_cut)
        phase_twist_and_drop_cap(ctx)
        if frame_gated:
            confirm_next_frame(ctx, completed_frame=4, next_frame=5)
        phase_remove_spring(ctx)
        phase_remove_yellow_plastic(ctx)
        phase_remove_shell(ctx)
    phase_return_middle(ctx)
    phase_shutdown(ctx)


def run_robot_only_smoke(ctx: RecipeContext, *, uncut: bool = False) -> None:
    ctx.logger.info("=== robot_only_smoke ===")
    ctx.logger.warn(
        "Robot-only smoke skips Arduino CLOSE_VISE and CUT_HEIGHT; "
        "use a safe test setup before running with a loose part."
    )
    if bool(ctx.params["TAG_CALI_ENABLED"]):
        ctx.logger.warn(
            "Robot-only smoke does not run the automatic tag-calibration preflight; "
            "it uses the saved Plate/Vise/Spring/Plastic/Glass key positions as-is."
        )
    step_numbers = UNCUT_STEP_SEQUENCE if uncut else tuple(sorted(PLAN_STEPS))
    if uncut:
        ctx.logger.info(
            "Robot-only smoke uncut path: after step 12 jump directly to step 37"
        )
    for step_number in step_numbers:
        if step_requires_arduino(step_number):
            ctx.logger.info(
                f"=== step {step_number}: {PLAN_STEPS[step_number]} "
                "SKIPPED for robot-only smoke ==="
            )
            continue
        execute_plan_step(ctx, step_number)
    phase_shutdown(ctx)


def run_tag_calibration_only(ctx: RecipeContext) -> None:
    ctx.logger.info("=== tag_calibration_only ===")
    phase_startup_transfer(ctx)
    phase_calibrate_tags(ctx)
    phase_shutdown(ctx)


def run_selected_steps(
    ctx: RecipeContext,
    step_numbers: tuple[int, ...],
    *,
    yes: bool = False,
    skip_cut: bool = False,
) -> None:
    if not step_numbers:
        raise RecipeError("selected-step run requires at least one step")
    for step_number in step_numbers:
        if int(step_number) == 2:
            execute_plan_step(ctx, 2, yes=yes, skip_cut=skip_cut)
            phase_calibrate_tags(ctx)
            continue
        execute_plan_step(ctx, step_number, yes=yes, skip_cut=skip_cut)


def build_arduino(args, params: dict, logger, dry_run: bool):
    if dry_run:
        arduino = DryRunArduino(logger=logger)
        arduino.connect()
        return arduino
    arduino = ArduinoClient(
        port=params["ARDUINO_PORT"],
        baud=int(params["ARDUINO_BAUD"]),
        done_timeout_s=float(params["ARDUINO_DONE_TIMEOUT_S"]),
        logger=logger,
        ready_timeout_s=DEFAULT_READY_TIMEOUT_S,
    )
    arduino.connect()
    return arduino


def step_uses_camera(step_number: int) -> bool:
    return step_number == 4


def build_camera_session(params: dict, logger, *, enable_recording: bool):
    record_robot_waypoints = load_record_robot_waypoints_module()
    session = SharedInjectableCameraSession(
        detector_module=record_robot_waypoints.load_injectable_detector_module(),
        serial=params["INJECTABLE_CAMERA_SERIAL"],
        width=int(params["INJECTABLE_WIDTH"]),
        height=int(params["INJECTABLE_HEIGHT"]),
        fps=int(params["INJECTABLE_FPS"]),
        warmup_frames=int(params["INJECTABLE_WARMUP_FRAMES"]),
        exposure=params["INJECTABLE_EXPOSURE"],
        gain=params["INJECTABLE_GAIN"],
        white_balance=params["INJECTABLE_WHITE_BALANCE"],
        logger=logger,
        record_path=str(params["POV_RECORD_PATH"]) if enable_recording else None,
    )
    session.start()
    return session


def best_effort_stop_all(arduino, logger) -> None:
    if arduino is None:
        return
    try:
        arduino.stop_all()
    except Exception as exc:  # noqa: BLE001 - shutdown/fault path must not raise
        logger.warn(f"STOP_ALL failed during fault handling: {exc}")


def best_effort_close_camera(camera_session, logger) -> None:
    if camera_session is None:
        return
    try:
        camera_session.close()
    except Exception as exc:  # noqa: BLE001 - shutdown/fault path must not raise
        logger.warn(f"Shared camera session close failed: {exc}")


def print_dry_run_summary(
    params: dict,
    key_dir: Path,
    poses: dict[str, dict],
    logger,
    *,
    uncut: bool = False,
) -> None:
    logger.info("--- dry-run summary ---")
    logger.info(f"key_position_dir={key_dir}")
    logger.info(f"required_positions={', '.join(REQUIRED_POSITIONS)}")
    logger.info(f"loaded_positions={', '.join(sorted(poses))}")
    if uncut:
        logger.info(
            "phase_order=startup_transfer -> calibrate_tags -> "
            "pick_from_camera_align_grasp -> load_vise -> "
            "jump_to_remove_shell_and_glass -> shutdown"
        )
    else:
        logger.info(
            "phase_order=startup_transfer -> calibrate_tags -> "
            "pick_from_camera_align_grasp -> load_vise -> cut -> "
            "twist_and_drop_cap -> remove_spring -> "
            "remove_yellow_plastic -> remove_shell_and_glass -> shutdown"
        )
    logger.info(f"SLOW_MODE_ENABLED={params['SLOW_MODE_ENABLED']}")
    logger.info(f"UNCUT_MODE_ENABLED={uncut}")
    for key in (
        "TAG_CALI_ENABLED",
        "TAG_1_STAGING_KEY_POSITION",
        "TAG_2_STAGING_KEY_POSITION",
        "TAG_3_STAGING_KEY_POSITION",
        "VISE_CALI_TARGET_KEY_POSITION",
        "VISE_CALI_BOARD",
        "VISE_CALI_CALIBRATION",
        "VISE_CALI_REFERENCE",
        "VISE_CALI_MIN_CHARUCO_CORNERS",
        "TAG_CALI_MAX_ATTEMPTS",
        "TAG_CALI_RETRY_SETTLE_S",
        "INJECTABLE_ALIGN_TARGET_INDEX",
        "INJECTABLE_ALIGN_OFFSET_TOWARD_PINK_M",
        "INJECTABLE_CAMERA_SERIAL",
        "INJECTABLE_WIDTH",
        "INJECTABLE_HEIGHT",
        "INJECTABLE_FPS",
        "INJECTABLE_WARMUP_FRAMES",
        "INJECTABLE_EXPOSURE",
        "INJECTABLE_GAIN",
        "INJECTABLE_WHITE_BALANCE",
        "INJECTABLE_TEAL_LOW_HSV",
        "INJECTABLE_TEAL_HIGH_HSV",
        "INJECTABLE_PINK_LOW_HSV",
        "INJECTABLE_PINK_HIGH_HSV",
        "INJECTABLE_MIN_PINK_AREA",
        "INJECTABLE_MIN_TEAL_AREA",
        "INJECTABLE_DIST_TOLERANCE",
        "INJECTABLE_AXIS_COS_MIN",
        "INJECTABLE_ALIGN_PRIMITIVE",
        "INJECTABLE_ALIGN_LINEAR_VEL_M_S",
        "INJECTABLE_ALIGN_MAX_ATTEMPTS",
        "INJECTABLE_ALIGN_RETRY_SETTLE_S",
        "INJECTABLE_ALIGN_FRAME_TIMEOUT_S",
        "POV_RECORD_ENABLED",
        "POV_RECORD_PATH",
        "GRIPPER_VELOCITY_M_S",
        "GRASP_FORCE_N",
        "GRASP_OPEN_WIDTH_M",
        "GRASP_OPEN_FORCE_LIMIT_N",
        "GRASP_CONTACT_FORCE_N",
        "GRASP_PRECONTACT_MOVE_Z_M",
        "GRASP_PRECONTACT_MOVE_VEL_M_S",
        "GRASP_CONTACT_VEL_M_S",
        "PICKUP_LIFT_Z_OFFSET_M",
        "VISE_APPROACH_Z_OFFSET_M",
        "VISE_RETREAT_Z_OFFSET_M",
        "SPRING_REMOVE_APPROACH_Z_OFFSET_M",
        "SPRING_REMOVE_GRIP_Z_OFFSET_M",
        "SPRING_REMOVE_FORCE_N",
        "SPRING_REMOVE_LIFT_Z_OFFSET_M",
        "YELLOW_REMOVE_APPROACH_Z_OFFSET_M",
        "YELLOW_REMOVE_GRIP_Z_OFFSET_M",
        "YELLOW_REMOVE_FORCE_N",
        "YELLOW_REMOVE_LIFT_Z_OFFSET_M",
        "SHELL_REMOVE_APPROACH_Z_OFFSET_M",
        "SHELL_REMOVE_GRIP_Z_OFFSET_M",
        "SHELL_REMOVE_FORCE_N",
        "SHELL_REMOVE_LIFT_Z_OFFSET_M",
        "DUMP_TOOL_Z_DEG",
        "VISE_OPEN_TARGET_FORCE_KG",
        "MOVE_JNT_VEL_SCALE",
        "MOVE_USE_REF_JOINTS",
        "INSERTCOMP_INSERT_AXIS",
        "INSERTCOMP_COMP_AXIS",
        "INSERTCOMP_MAX_CONTACT_FORCE_N",
        "INSERTCOMP_DEADBAND_SCALE",
        "INSERTCOMP_INSERT_VEL_M_S",
        "INSERTCOMP_COMP_VEL_SCALE",
        "INSERTCOMP_START_TIMEOUT_S",
        "CUT_X_MM",
        "CUT_Z_MM",
        "CUT_DEG",
        "ROT_SAFE_TOL_DEG",
        "CAP_GRIP_Z_OFFSET_M",
        "CAP_TWIST_DEG",
        "CAP_TWIST_REPEAT_COUNT",
        "CAP_LIFT_Z_OFFSET_M",
        "GRIPPER_FORCE_N",
        "GRIPPER_RELEASE_WIDTH_M",
        "GRIPPER_SETTLE_S",
        "GRIPPER_OPEN_SETTLE_S",
    ):
        logger.info(f"{key}={params[key]}")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="recipe.py",
        description="Run the GreenPink injectable robot + cutting-machine recipe.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print the plan without hardware")
    parser.add_argument(
        "--step",
        default=None,
        help="Run one step or inclusive step range, for example: --step 15 or --step 1-5",
    )
    parser.add_argument(
        "--robot-only-smoke",
        action="store_true",
        help="Exercise only the robot/gripper pose path; no Arduino vise/cut actions",
    )
    parser.add_argument("--yes", action="store_true", help="Skip CUT_HEIGHT confirmation prompt")
    parser.add_argument(
        "--frame-gated",
        action="store_true",
        help="Pause after each human-code frame and wait for 'N' before continuing to the next frame.",
    )
    parser.add_argument("--skip-cut", action="store_true", help="Run all phases except CUT_HEIGHT")
    parser.add_argument(
        "--uncut",
        action="store_true",
        help="After step 12, jump directly to step 37 and skip steps 13 through 36.",
    )
    parser.add_argument(
        "--slow",
        action="store_true",
        help="Use the slower record_robot_waypoints motion profile for key-position transfers and straight-line moves.",
    )
    parser.add_argument(
        "--skip-tag-calibration",
        "--skip-vise-calibration",
        dest="skip_tag_calibration",
        action="store_true",
        help="Still visit tag 1/2/3 staging poses, but skip relocalization and keep the saved key positions as-is.",
    )
    parser.add_argument(
        "--tag-calibration-only",
        "--vise-calibration-only",
        dest="tag_calibration_only",
        action="store_true",
        help="Run only the startup transfer plus tag 1/2/3 preflight, then stop.",
    )
    parser.add_argument("--skip-gripper-init", action="store_true")
    parser.add_argument("--key-position-dir", default=None)
    parser.add_argument("--robot-sn", default=None)
    parser.add_argument("--arduino-port", default=None)
    parser.add_argument(
        "--record-pov",
        action="store_true",
        help="Record a color POV video from the shared RealSense session.",
    )
    parser.add_argument(
        "--pov-record-path",
        default=None,
        help="Path for POV MP4 output when --record-pov is enabled.",
    )
    parser.add_argument("--jnt-vel-scale", type=int, default=None)
    parser.add_argument("--injectable-align-target-index", type=int, default=None)
    parser.add_argument(
        "--injectable-align-offset-toward-pink-m", type=float, default=None
    )
    parser.add_argument("--injectable-camera-serial", default=None)
    parser.add_argument("--injectable-width", type=int, default=None)
    parser.add_argument("--injectable-height", type=int, default=None)
    parser.add_argument("--injectable-fps", type=int, default=None)
    parser.add_argument("--injectable-warmup-frames", type=int, default=None)
    parser.add_argument("--injectable-exposure", type=float, default=None)
    parser.add_argument("--injectable-gain", type=float, default=None)
    parser.add_argument("--injectable-white-balance", type=float, default=None)
    parser.add_argument("--injectable-teal-low-hsv", default=None)
    parser.add_argument("--injectable-teal-high-hsv", default=None)
    parser.add_argument("--injectable-pink-low-hsv", default=None)
    parser.add_argument("--injectable-pink-high-hsv", default=None)
    parser.add_argument("--injectable-min-pink-area", type=int, default=None)
    parser.add_argument("--injectable-min-teal-area", type=int, default=None)
    parser.add_argument("--injectable-dist-tolerance", type=float, default=None)
    parser.add_argument("--injectable-axis-cos-min", type=float, default=None)
    parser.add_argument(
        "--injectable-align-primitive",
        choices=("movel", "moveptp"),
        default=None,
    )
    parser.add_argument("--injectable-align-linear-vel-m-s", type=float, default=None)
    parser.add_argument("--injectable-align-max-attempts", type=int, default=None)
    parser.add_argument("--injectable-align-retry-settle-s", type=float, default=None)
    parser.add_argument("--injectable-align-frame-timeout-s", type=float, default=None)
    parser.add_argument("--tag1-staging-key-position", "--vise-cali-staging-key-position", dest="tag1_staging_key_position", default=None)
    parser.add_argument("--tag2-staging-key-position", dest="tag2_staging_key_position", default=None)
    parser.add_argument("--tag3-staging-key-position", dest="tag3_staging_key_position", default=None)
    parser.add_argument("--cali-vise-key-position", default=None)
    parser.add_argument("--cali-vise-board", default=None)
    parser.add_argument("--cali-vise-calibration", default=None)
    parser.add_argument("--cali-vise-reference", default=None)
    parser.add_argument("--cali-vise-capture-out", default=None)
    parser.add_argument("--cali-vise-detect-out", default=None)
    parser.add_argument("--cali-vise-min-charuco-corners", type=int, default=None)
    parser.add_argument("--tag-cali-max-attempts", type=int, default=None)
    parser.add_argument("--tag-cali-retry-settle-s", type=float, default=None)
    parser.add_argument("--pickup-lift-z-m", type=float, default=None)
    parser.add_argument("--gripper-velocity-m-s", type=float, default=None)
    parser.add_argument("--grasp-force-n", type=float, default=None)
    parser.add_argument("--grasp-open-width-m", type=float, default=None)
    parser.add_argument("--grasp-open-force-limit-n", type=float, default=None)
    parser.add_argument("--grasp-contact-force-n", type=float, default=None)
    parser.add_argument("--grasp-precontact-z-m", type=float, default=None)
    parser.add_argument("--grasp-precontact-vel-m-s", type=float, default=None)
    parser.add_argument("--grasp-contact-vel-m-s", type=float, default=None)
    parser.add_argument("--vise-approach-z-m", type=float, default=None)
    parser.add_argument("--vise-retreat-z-m", type=float, default=None)
    parser.add_argument("--spring-approach-z-m", type=float, default=None)
    parser.add_argument("--spring-grip-z-m", type=float, default=None)
    parser.add_argument("--spring-force-n", type=float, default=None)
    parser.add_argument("--yellow-approach-z-m", type=float, default=None)
    parser.add_argument("--yellow-grip-z-m", type=float, default=None)
    parser.add_argument("--yellow-force-n", type=float, default=None)
    parser.add_argument("--shell-approach-z-m", type=float, default=None)
    parser.add_argument("--shell-grip-z-m", type=float, default=None)
    parser.add_argument("--shell-force-n", type=float, default=None)
    parser.add_argument("--vise-open-target-force-kg", type=float, default=None)
    parser.add_argument(
        "--insertcomp-insert-axis",
        choices=("AUTO_WORLD_NEG_Z", "X", "-X", "Y", "-Y", "Z", "-Z"),
        default=None,
    )
    parser.add_argument("--insertcomp-max-contact-force-n", type=float, default=None)
    parser.add_argument("--insertcomp-deadband-scale", type=float, default=None)
    parser.add_argument("--insertcomp-insert-vel-m-s", type=float, default=None)
    parser.add_argument("--insertcomp-comp-vel-scale", type=float, default=None)
    parser.add_argument("--insertcomp-start-timeout-s", type=float, default=None)
    parser.add_argument("--insertcomp-timeout-s", type=float, default=None)
    parser.add_argument("--cut-x-mm", type=float, default=None)
    parser.add_argument("--cut-z-mm", type=float, default=None)
    parser.add_argument("--cut-deg", type=float, default=None)
    parser.add_argument("--rot-safe-tol-deg", type=float, default=None)
    parser.add_argument("--cap-grip-z-m", type=float, default=None)
    parser.add_argument("--cap-twist-deg", type=float, default=None)
    parser.add_argument("--cap-twist-repeat-count", type=int, default=None)
    parser.add_argument("--cap-lift-z-m", type=float, default=None)
    parser.add_argument("--gripper-force-n", type=float, default=None)
    parser.add_argument("--gripper-close-width-m", type=float, default=None)
    parser.add_argument("--gripper-release-width-m", type=float, default=None)
    parser.add_argument("--gripper-settle-s", type=float, default=None)
    parser.add_argument("--gripper-open-settle-s", type=float, default=None)
    return parser.parse_args(argv)


def resolve_params(args: argparse.Namespace) -> dict:
    params = dict(PARAMS)
    if args.slow:
        params["SLOW_MODE_ENABLED"] = True
        params["MOVE_JNT_VEL_SCALE"] = SLOW_PROFILE_KEY_MOVE_JNT_VEL_SCALE
        params["CARTESIAN_INSERT_VEL_M_S"] = SLOW_PROFILE_TOOL_MOVE_LINEAR_VEL_M_S
        params["CARTESIAN_RETREAT_VEL_M_S"] = SLOW_PROFILE_TOOL_MOVE_LINEAR_VEL_M_S
        params["INJECTABLE_ALIGN_LINEAR_VEL_M_S"] = (
            SLOW_PROFILE_TOOL_MOVE_LINEAR_VEL_M_S
        )
        params["DUMP_MOVEC_VEL_M_S"] = SLOW_PROFILE_DUMP_MOVEC_VEL_M_S
    overrides = {
        "ROBOT_SN": args.robot_sn,
        "ARDUINO_PORT": args.arduino_port,
        "POV_RECORD_ENABLED": True if args.record_pov else None,
        "POV_RECORD_PATH": args.pov_record_path,
        "TAG_CALI_ENABLED": False if args.skip_tag_calibration else None,
        "TAG_1_STAGING_KEY_POSITION": args.tag1_staging_key_position,
        "TAG_2_STAGING_KEY_POSITION": args.tag2_staging_key_position,
        "TAG_3_STAGING_KEY_POSITION": args.tag3_staging_key_position,
        "VISE_CALI_TARGET_KEY_POSITION": args.cali_vise_key_position,
        "VISE_CALI_BOARD": args.cali_vise_board,
        "VISE_CALI_CALIBRATION": args.cali_vise_calibration,
        "VISE_CALI_REFERENCE": args.cali_vise_reference,
        "VISE_CALI_CAPTURE_OUT": args.cali_vise_capture_out,
        "VISE_CALI_DETECT_OUT": args.cali_vise_detect_out,
        "VISE_CALI_MIN_CHARUCO_CORNERS": args.cali_vise_min_charuco_corners,
        "TAG_CALI_MAX_ATTEMPTS": args.tag_cali_max_attempts,
        "TAG_CALI_RETRY_SETTLE_S": args.tag_cali_retry_settle_s,
        "MOVE_JNT_VEL_SCALE": args.jnt_vel_scale,
        "INJECTABLE_ALIGN_TARGET_INDEX": args.injectable_align_target_index,
        "INJECTABLE_ALIGN_OFFSET_TOWARD_PINK_M": args.injectable_align_offset_toward_pink_m,
        "INJECTABLE_CAMERA_SERIAL": args.injectable_camera_serial,
        "INJECTABLE_WIDTH": args.injectable_width,
        "INJECTABLE_HEIGHT": args.injectable_height,
        "INJECTABLE_FPS": args.injectable_fps,
        "INJECTABLE_WARMUP_FRAMES": args.injectable_warmup_frames,
        "INJECTABLE_EXPOSURE": args.injectable_exposure,
        "INJECTABLE_GAIN": args.injectable_gain,
        "INJECTABLE_WHITE_BALANCE": args.injectable_white_balance,
        "INJECTABLE_TEAL_LOW_HSV": _parse_hsv_triplet_arg(
            args.injectable_teal_low_hsv, "--injectable-teal-low-hsv"
        ),
        "INJECTABLE_TEAL_HIGH_HSV": _parse_hsv_triplet_arg(
            args.injectable_teal_high_hsv, "--injectable-teal-high-hsv"
        ),
        "INJECTABLE_PINK_LOW_HSV": _parse_hsv_triplet_arg(
            args.injectable_pink_low_hsv, "--injectable-pink-low-hsv"
        ),
        "INJECTABLE_PINK_HIGH_HSV": _parse_hsv_triplet_arg(
            args.injectable_pink_high_hsv, "--injectable-pink-high-hsv"
        ),
        "INJECTABLE_MIN_PINK_AREA": args.injectable_min_pink_area,
        "INJECTABLE_MIN_TEAL_AREA": args.injectable_min_teal_area,
        "INJECTABLE_DIST_TOLERANCE": args.injectable_dist_tolerance,
        "INJECTABLE_AXIS_COS_MIN": args.injectable_axis_cos_min,
        "INJECTABLE_ALIGN_PRIMITIVE": args.injectable_align_primitive,
        "INJECTABLE_ALIGN_LINEAR_VEL_M_S": args.injectable_align_linear_vel_m_s,
        "INJECTABLE_ALIGN_MAX_ATTEMPTS": args.injectable_align_max_attempts,
        "INJECTABLE_ALIGN_RETRY_SETTLE_S": args.injectable_align_retry_settle_s,
        "INJECTABLE_ALIGN_FRAME_TIMEOUT_S": args.injectable_align_frame_timeout_s,
        "PICKUP_LIFT_Z_OFFSET_M": args.pickup_lift_z_m,
        "GRIPPER_VELOCITY_M_S": args.gripper_velocity_m_s,
        "GRASP_FORCE_N": args.grasp_force_n,
        "GRASP_OPEN_WIDTH_M": args.grasp_open_width_m,
        "GRASP_OPEN_FORCE_LIMIT_N": args.grasp_open_force_limit_n,
        "GRASP_CONTACT_FORCE_N": args.grasp_contact_force_n,
        "GRASP_PRECONTACT_MOVE_Z_M": args.grasp_precontact_z_m,
        "GRASP_PRECONTACT_MOVE_VEL_M_S": args.grasp_precontact_vel_m_s,
        "GRASP_CONTACT_VEL_M_S": args.grasp_contact_vel_m_s,
        "VISE_APPROACH_Z_OFFSET_M": args.vise_approach_z_m,
        "VISE_RETREAT_Z_OFFSET_M": args.vise_retreat_z_m,
        "SPRING_REMOVE_APPROACH_Z_OFFSET_M": args.spring_approach_z_m,
        "SPRING_REMOVE_GRIP_Z_OFFSET_M": args.spring_grip_z_m,
        "SPRING_REMOVE_FORCE_N": args.spring_force_n,
        "YELLOW_REMOVE_APPROACH_Z_OFFSET_M": args.yellow_approach_z_m,
        "YELLOW_REMOVE_GRIP_Z_OFFSET_M": args.yellow_grip_z_m,
        "YELLOW_REMOVE_FORCE_N": args.yellow_force_n,
        "SHELL_REMOVE_APPROACH_Z_OFFSET_M": args.shell_approach_z_m,
        "SHELL_REMOVE_GRIP_Z_OFFSET_M": args.shell_grip_z_m,
        "SHELL_REMOVE_FORCE_N": args.shell_force_n,
        "VISE_OPEN_TARGET_FORCE_KG": args.vise_open_target_force_kg,
        "INSERTCOMP_INSERT_AXIS": args.insertcomp_insert_axis,
        "INSERTCOMP_MAX_CONTACT_FORCE_N": args.insertcomp_max_contact_force_n,
        "INSERTCOMP_DEADBAND_SCALE": args.insertcomp_deadband_scale,
        "INSERTCOMP_INSERT_VEL_M_S": args.insertcomp_insert_vel_m_s,
        "INSERTCOMP_COMP_VEL_SCALE": args.insertcomp_comp_vel_scale,
        "INSERTCOMP_START_TIMEOUT_S": args.insertcomp_start_timeout_s,
        "INSERTCOMP_TIMEOUT_S": args.insertcomp_timeout_s,
        "CUT_X_MM": args.cut_x_mm,
        "CUT_Z_MM": args.cut_z_mm,
        "CUT_DEG": args.cut_deg,
        "ROT_SAFE_TOL_DEG": args.rot_safe_tol_deg,
        "CAP_GRIP_Z_OFFSET_M": args.cap_grip_z_m,
        "CAP_TWIST_DEG": args.cap_twist_deg,
        "CAP_TWIST_REPEAT_COUNT": args.cap_twist_repeat_count,
        "CAP_LIFT_Z_OFFSET_M": args.cap_lift_z_m,
        "GRIPPER_FORCE_N": args.gripper_force_n,
        "GRIPPER_CLOSE_WIDTH_M": args.gripper_close_width_m,
        "GRIPPER_RELEASE_WIDTH_M": args.gripper_release_width_m,
        "GRIPPER_SETTLE_S": args.gripper_settle_s,
        "GRIPPER_OPEN_SETTLE_S": args.gripper_open_settle_s,
    }
    for key, value in overrides.items():
        if value is not None:
            params[key] = value
    if args.key_position_dir is not None:
        params["KEY_POSITION_DIR"] = args.key_position_dir
    return params


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logger = make_logger()
    params = resolve_params(args)
    key_dir = _script_path(str(params["KEY_POSITION_DIR"]))
    arduino = None
    camera_session = None
    run_lock = None
    selected_steps = None
    ctx = None

    try:
        if args.tag_calibration_only and args.step is not None:
            raise RecipeError("--tag-calibration-only cannot be combined with --step")
        if args.tag_calibration_only and args.robot_only_smoke:
            raise RecipeError(
                "--tag-calibration-only cannot be combined with --robot-only-smoke"
            )
        if not args.dry_run:
            # Serialize all GreenPink recipe variants that target the same hardware.
            run_lock = SingleInstanceLock(HERE / ".greenpinkcamera.lock")
            run_lock.acquire()
        if args.step is not None:
            selected_steps = parse_step_selection(args.step)
        if args.tag_calibration_only:
            needs_arduino = False
        else:
            needs_arduino = not args.robot_only_smoke and (
                selected_steps is None
                or any(step_requires_arduino(step) for step in selected_steps)
            )
        needs_camera_session = False
        if args.tag_calibration_only:
            needs_camera_session = False
        elif selected_steps is not None:
            needs_camera_session = any(
                step_uses_camera(step) for step in selected_steps
            ) or bool(
                params["POV_RECORD_ENABLED"]
            )
        elif args.robot_only_smoke:
            needs_camera_session = bool(params["POV_RECORD_ENABLED"])
        elif not bool(params["TAG_CALI_ENABLED"]):
            needs_camera_session = True
        poses = load_positions(key_dir, logger)
        if args.dry_run:
            print_dry_run_summary(params, key_dir, poses, logger, uncut=args.uncut)
            arduino = build_arduino(args, params, logger, True) if needs_arduino else None
            ctx = RecipeContext(
                params=params,
                poses=poses,
                logger=logger,
                dry_run=True,
                session=None,
                arduino=arduino,
                camera_session=None,
            )
            if args.tag_calibration_only:
                run_tag_calibration_only(ctx)
            elif selected_steps is not None:
                run_selected_steps(ctx, selected_steps, yes=True, skip_cut=args.skip_cut)
            elif args.robot_only_smoke:
                run_robot_only_smoke(ctx, uncut=args.uncut)
            else:
                run_full_recipe(
                    ctx,
                    yes=True,
                    skip_cut=args.skip_cut,
                    uncut=args.uncut,
                    frame_gated=args.frame_gated,
                )
            if arduino is not None:
                arduino.close()
                arduino = None
            return 0

        arduino = build_arduino(args, params, logger, False) if needs_arduino else None
        try:
            if needs_camera_session:
                camera_session = build_camera_session(
                    params,
                    logger,
                    enable_recording=bool(params["POV_RECORD_ENABLED"]),
                )
            with RobotSession(params["ROBOT_SN"], logger=logger) as session:
                gripper_init = not args.skip_gripper_init
                if selected_steps is not None:
                    selected_label = step_selection_label(selected_steps)
                    if args.skip_gripper_init:
                        logger.info(
                            "Selected-step mode: skipping gripper initialization "
                            "because --skip-gripper-init was requested"
                        )
                    elif 1 in selected_steps:
                        gripper_init = True
                        logger.info(
                            "Selected-step mode: initializing gripper because "
                            f"step 1 is included in requested steps {selected_label}"
                        )
                    else:
                        gripper_init = False
                        logger.info(
                            "Selected-step mode: skipping gripper initialization "
                            f"because step 1 is not included in requested steps {selected_label}"
                        )
                session.setup_gripper(
                    str(params["GRIPPER_NAME"]),
                    init=gripper_init,
                )
                session.switch_mode("NRT_PRIMITIVE_EXECUTION")
                ctx = RecipeContext(
                    params=params,
                    poses=poses,
                    logger=logger,
                    dry_run=False,
                    session=session,
                    arduino=arduino,
                    camera_session=camera_session,
                )
                if args.tag_calibration_only:
                    run_tag_calibration_only(ctx)
                elif selected_steps is not None:
                    run_selected_steps(ctx, selected_steps, yes=args.yes, skip_cut=args.skip_cut)
                elif args.robot_only_smoke:
                    run_robot_only_smoke(ctx, uncut=args.uncut)
                else:
                    run_full_recipe(
                        ctx,
                        yes=args.yes,
                        skip_cut=args.skip_cut,
                        uncut=args.uncut,
                        frame_gated=args.frame_gated,
                    )
            return 0
        except Exception:
            best_effort_stop_all(arduino, logger)
            raise
        finally:
            best_effort_close_camera(
                camera_session
                if camera_session is not None
                else getattr(ctx, "camera_session", None),
                logger,
            )
            camera_session = None
            if arduino is not None:
                try:
                    arduino.close()
                except Exception:
                    pass
                arduino = None
            if run_lock is not None:
                run_lock.release()
                run_lock = None
    except RecipeError as exc:
        best_effort_stop_all(arduino, logger)
        logger.error(str(exc))
        return 2
    except KeyboardInterrupt:
        best_effort_stop_all(arduino, logger)
        logger.warn("interrupted by user")
        return 130
    except Exception as exc:  # noqa: BLE001 - top-level safety/reporting
        best_effort_stop_all(arduino, logger)
        logger.error(f"unexpected error: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
