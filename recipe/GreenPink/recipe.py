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
import json
import math
import sys
from pathlib import Path
from typing import Any


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
    DEFAULT_BAUD,
    DEFAULT_READY_TIMEOUT_S,
)


PARAMS: dict[str, Any] = {
    "ROBOT_SN": "Rizon4-062930",
    "GRIPPER_NAME": "Flexiv-GN01",
    "KEY_POSITION_DIR": "key_positions",
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
    "TRAY_APPROACH_Z_OFFSET_M": 0.05,
    "TRAY_GRIP_Z_OFFSET_M": 0.0,
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
    "INSERTCOMP_MAX_CONTACT_FORCE_N": 5.0,
    "INSERTCOMP_DEADBAND_SCALE": 50.0,
    "INSERTCOMP_INSERT_VEL_M_S": 0.01,
    "INSERTCOMP_COMP_VEL_SCALE": 20.0,
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

REQUIRED_POSITIONS = ("Inter", "Home", "Tray", "Vise", "plastic", "glass")
OPTIONAL_POSITIONS = ("spring", "metal")

PLAN_STEPS = {
    1: "open gripper and move to Inter",
    2: "move to Home",
    3: "move to above Tray",
    4: "move vertically down to Tray",
    5: "close gripper at 80 N",
    6: "move vertically up to above Tray",
    7: "move to Home",
    8: "move to Inter",
    9: "move to above Vise",
    10: "InsertComp down into vise",
    11: "close vise",
    12: "release injectable with wider gripper opening",
    13: "retreat up in world Z and wait clear of vise",
    14: "CUT_HEIGHT",
    15: "move down to Vise",
    16: "close gripper at 80 N on cap",
    17: "twist cap around TCP X and return",
    18: "lift cap up in positive world Z",
    19: "move to Inter",
    20: "move to Home",
    21: "move to plastic",
    22: "open gripper",
}

REMOVAL_CYCLES = (
    {
        "name": "spring",
        "base_step": 23,
        "approach_z_param": "SPRING_REMOVE_APPROACH_Z_OFFSET_M",
        "grip_z_param": "SPRING_REMOVE_GRIP_Z_OFFSET_M",
        "force_param": "SPRING_REMOVE_FORCE_N",
        "lift_z_param": "SPRING_REMOVE_LIFT_Z_OFFSET_M",
        "drop_target": "metal",
    },
    {
        "name": "yellow plastic",
        "base_step": 33,
        "approach_z_param": "YELLOW_REMOVE_APPROACH_Z_OFFSET_M",
        "grip_z_param": "YELLOW_REMOVE_GRIP_Z_OFFSET_M",
        "force_param": "YELLOW_REMOVE_FORCE_N",
        "lift_z_param": "YELLOW_REMOVE_LIFT_Z_OFFSET_M",
        "drop_target": "plastic",
    },
)

for cycle in REMOVAL_CYCLES:
    base = int(cycle["base_step"])
    name = str(cycle["name"])
    drop_target = str(cycle["drop_target"])
    PLAN_STEPS.update(
        {
            base + 0: f"move to Home for {name} removal",
            base + 1: f"move to Inter for {name} removal",
            base + 2: f"move to above Vise for {name} removal",
            base + 3: f"move to Vise grasp for {name} removal",
            base + 4: f"close gripper for {name} removal",
            base + 5: f"lift {name} up in positive world Z",
            base + 6: f"move to Inter after {name} removal",
            base + 7: f"move to Home after {name} removal",
            base + 8: f"move to {drop_target} drop after {name} removal",
            base + 9: f"open gripper after {name} removal",
        }
    )
PLAN_STEPS.update(
    {
        43: "move to Home for shell and glass removal",
        44: "move to Inter for shell and glass removal",
        45: "move to above Vise for shell and glass removal",
        46: "move to Vise grasp for shell and glass removal",
        47: "close gripper for shell and glass removal",
        48: "open vise before lifting shell",
        49: "lift shell up in positive world Z",
        50: "move to Inter after shell removal",
        51: "move to Home after shell removal",
        55: "move to glass",
        56: "dump with MoveC arc around tool-frame virtual pivot and return",
        57: "move to plastic",
        58: "open gripper",
        59: "move to Inter",
    }
)


class RecipeError(RuntimeError):
    """Raised for expected recipe validation / safety failures."""


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


def _position_file(key_dir: Path, name: str) -> Path:
    exact = key_dir / f"{name}.json"
    if exact.exists():
        return exact
    lower = key_dir / f"{name.lower()}.json"
    if lower.exists():
        return lower
    wanted = f"{name.lower()}.json"
    for candidate in key_dir.glob("*.json"):
        if candidate.name.lower() == wanted:
            return candidate
    raise RecipeError(f"missing key position {name!r} in {key_dir}")


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
    ) -> None:
        self.params = params
        self.poses = poses
        self.logger = logger
        self.dry_run = dry_run
        self.session = session
        self.arduino = arduino

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
            self.logger.info("InsertComp wait: isMoving == 0")
            return
        if self.session is None:
            raise RecipeError("robot session is not available")
        self.session.execute_primitive("InsertComp", params)
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
        helper_gripper_set(
            self.session.gripper,
            width,
            vel_m_s=float(self.params["GRIPPER_VELOCITY_M_S"]),
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
            start_q_rad = list(self.poses["glass"]["q_rad"])
            start_tcp = list(self.poses["glass"]["tcp_pose_world"]["values"])
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


def ensure_arduino_ready(ctx: RecipeContext) -> dict:
    arduino = require_arduino(ctx)
    status = arduino.get_status()
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


def above_tray_pose(ctx: RecipeContext) -> dict:
    return offset_pose_z(
        ctx.poses["Tray"],
        float(ctx.params["TRAY_APPROACH_Z_OFFSET_M"]),
    )


def tray_grip_pose(ctx: RecipeContext) -> tuple[str, dict]:
    tray_grip_offset = float(ctx.params["TRAY_GRIP_Z_OFFSET_M"])
    tray_grip = (
        ctx.poses["Tray"]
        if tray_grip_offset == 0.0
        else offset_pose_z(ctx.poses["Tray"], tray_grip_offset)
    )
    tray_label = "Tray" if tray_grip_offset == 0.0 else "tray_grip"
    return tray_label, tray_grip


def above_vise_pose(ctx: RecipeContext) -> dict:
    return offset_pose_z(ctx.poses["Vise"], float(ctx.params["VISE_APPROACH_Z_OFFSET_M"]))


def vise_offset_pose(ctx: RecipeContext, dz_m: float) -> dict:
    return ctx.poses["Vise"] if float(dz_m) == 0.0 else offset_pose_z(ctx.poses["Vise"], dz_m)


def cap_grip_pose(ctx: RecipeContext) -> dict:
    return vise_offset_pose(ctx, float(ctx.params["CAP_GRIP_Z_OFFSET_M"]))


def resolve_drop_pose(ctx: RecipeContext, target_name: str) -> tuple[str, dict, bool]:
    if target_name in ctx.poses:
        use_linear = target_name in {"spring", "glass", "plastic"}
        return target_name, ctx.poses[target_name], use_linear
    if target_name == "metal" and "spring" in ctx.poses:
        ctx.logger.warn("metal pose not found; using spring pose as metal drop target")
        return "metal(spring)", ctx.poses["spring"], True
    raise RecipeError(f"drop target {target_name!r} is not available in loaded key positions")


def removal_cycle_for_step(step_number: int) -> tuple[dict, int] | None:
    for cycle in REMOVAL_CYCLES:
        base = int(cycle["base_step"])
        if base <= step_number <= base + 9:
            return cycle, step_number - base
    return None


def shell_glass_step_offset(step_number: int) -> int | None:
    shell_steps = (43, 44, 45, 46, 47, 48, 49, 50, 51, 55, 56, 57, 58, 59)
    if step_number in shell_steps:
        return shell_steps.index(step_number)
    return None


def step_requires_arduino(step_number: int) -> bool:
    return step_number in {11, 14, 48}


def step_uses_gripper(step_number: int) -> bool:
    return step_number in {
        1,
        5,
        12,
        13,
        16,
        22,
        27,
        32,
        37,
        42,
        47,
        58,
    }


def validate_plan_step(step_number: int) -> int:
    if step_number not in PLAN_STEPS:
        raise RecipeError(
            f"unknown plan step {step_number}; valid steps are 1-{max(PLAN_STEPS)}"
        )
    return step_number


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
        cycle, offset = removal_step
        name = str(cycle["name"])
        approach_z_m = float(ctx.params[str(cycle["approach_z_param"])])
        grip_z_m = float(ctx.params[str(cycle["grip_z_param"])])
        force_n = float(ctx.params[str(cycle["force_param"])])
        lift_z_m = float(ctx.params[str(cycle["lift_z_param"])])
        if offset == 0:
            ctx.move_ptp("Home", ctx.poses["Home"])
        elif offset == 1:
            ctx.move_l(
                "Inter",
                ctx.poses["Inter"],
                vel_m_s=float(ctx.params["CARTESIAN_RETREAT_VEL_M_S"]),
            )
        elif offset == 2:
            ctx.move_l(
                f"{name}_above_vise",
                vise_offset_pose(ctx, approach_z_m),
                vel_m_s=float(ctx.params["CARTESIAN_RETREAT_VEL_M_S"]),
            )
        elif offset == 3:
            ctx.move_l(
                f"{name}_grip",
                vise_offset_pose(ctx, grip_z_m),
                vel_m_s=float(ctx.params["CARTESIAN_INSERT_VEL_M_S"]),
            )
        elif offset == 4:
            ctx.gripper("close", force_n=force_n)
        elif offset == 5:
            ctx.move_current_z_offset(f"{name}_lift", lift_z_m)
        elif offset == 6:
            ctx.move_l(
                "Inter",
                ctx.poses["Inter"],
                vel_m_s=float(ctx.params["CARTESIAN_RETREAT_VEL_M_S"]),
            )
        elif offset == 7:
            ctx.move_l(
                "Home",
                ctx.poses["Home"],
                vel_m_s=float(ctx.params["CARTESIAN_RETREAT_VEL_M_S"]),
            )
        elif offset == 8:
            drop_label, drop_pose, use_linear = resolve_drop_pose(
                ctx, str(cycle["drop_target"])
            )
            if use_linear:
                ctx.move_l(
                    drop_label,
                    drop_pose,
                    vel_m_s=float(ctx.params["CARTESIAN_RETREAT_VEL_M_S"]),
                )
            else:
                ctx.move_ptp(drop_label, drop_pose)
        elif offset == 9:
            ctx.gripper("open")
        return

    shell_offset = shell_glass_step_offset(step_number)
    if shell_offset is not None:
        if shell_offset == 0:
            ctx.move_ptp("Home", ctx.poses["Home"])
        elif shell_offset == 1:
            ctx.move_l(
                "Inter",
                ctx.poses["Inter"],
                vel_m_s=float(ctx.params["CARTESIAN_RETREAT_VEL_M_S"]),
            )
        elif shell_offset == 2:
            ctx.move_l(
                "shell_above_vise",
                vise_offset_pose(ctx, float(ctx.params["SHELL_REMOVE_APPROACH_Z_OFFSET_M"])),
                vel_m_s=float(ctx.params["CARTESIAN_RETREAT_VEL_M_S"]),
            )
        elif shell_offset == 3:
            ctx.move_l(
                "shell_grip",
                vise_offset_pose(ctx, float(ctx.params["SHELL_REMOVE_GRIP_Z_OFFSET_M"])),
                vel_m_s=float(ctx.params["CARTESIAN_INSERT_VEL_M_S"]),
            )
        elif shell_offset == 4:
            ctx.gripper("close", force_n=float(ctx.params["SHELL_REMOVE_FORCE_N"]))
        elif shell_offset == 5:
            ensure_arduino_ready(ctx)
            result = require_arduino(ctx).open_vise(
                target_force_kg=float(ctx.params["VISE_OPEN_TARGET_FORCE_KG"])
            )
            validate_vise_open(result, float(ctx.params["VISE_OPEN_TARGET_FORCE_KG"]))
        elif shell_offset == 6:
            ctx.move_current_z_offset(
                "shell_lift",
                float(ctx.params["SHELL_REMOVE_LIFT_Z_OFFSET_M"]),
            )
        elif shell_offset == 7:
            ctx.move_l(
                "Inter",
                ctx.poses["Inter"],
                vel_m_s=float(ctx.params["CARTESIAN_RETREAT_VEL_M_S"]),
            )
        elif shell_offset == 8:
            ctx.move_l(
                "Home",
                ctx.poses["Home"],
                vel_m_s=float(ctx.params["CARTESIAN_RETREAT_VEL_M_S"]),
            )
        elif shell_offset == 9:
            ctx.move_l(
                "glass",
                ctx.poses["glass"],
                vel_m_s=float(ctx.params["CARTESIAN_RETREAT_VEL_M_S"]),
            )
        elif shell_offset == 10:
            ctx.dump_tool_z()
        elif shell_offset == 11:
            ctx.move_l(
                "plastic",
                ctx.poses["plastic"],
                vel_m_s=float(ctx.params["CARTESIAN_RETREAT_VEL_M_S"]),
            )
        elif shell_offset == 12:
            ctx.gripper("open")
        elif shell_offset == 13:
            ctx.move_l(
                "Inter",
                ctx.poses["Inter"],
                vel_m_s=float(ctx.params["CARTESIAN_RETREAT_VEL_M_S"]),
            )
        return

    if step_number == 1:
        ctx.gripper("open")
        ctx.move_ptp("Inter", ctx.poses["Inter"])
    elif step_number == 2:
        ctx.move_l(
            "Home",
            ctx.poses["Home"],
            vel_m_s=float(ctx.params["CARTESIAN_RETREAT_VEL_M_S"]),
        )
    elif step_number == 3:
        ctx.move_ptp("above_tray", above_tray_pose(ctx))
    elif step_number == 4:
        label, pose = tray_grip_pose(ctx)
        ctx.move_l(
            label,
            pose,
            vel_m_s=float(ctx.params["CARTESIAN_INSERT_VEL_M_S"]),
        )
    elif step_number == 5:
        ctx.gripper("close")
    elif step_number == 6:
        ctx.move_l(
            "above_tray_retract",
            above_tray_pose(ctx),
            vel_m_s=float(ctx.params["CARTESIAN_RETREAT_VEL_M_S"]),
        )
    elif step_number == 7:
        ctx.move_ptp("Home", ctx.poses["Home"])
    elif step_number == 8:
        ctx.move_l(
            "Inter",
            ctx.poses["Inter"],
            vel_m_s=float(ctx.params["CARTESIAN_RETREAT_VEL_M_S"]),
        )
    elif step_number == 9:
        ctx.move_l(
            "above_vise",
            above_vise_pose(ctx),
            vel_m_s=float(ctx.params["CARTESIAN_RETREAT_VEL_M_S"]),
        )
    elif step_number == 10:
        above_vise = above_vise_pose(ctx)
        ctx.insert_comp("Vise insert", reference_pose=above_vise)
    elif step_number == 11:
        ensure_arduino_ready(ctx)
        result = require_arduino(ctx).close_vise(
            target_force_kg=float(ctx.params["VISE_TARGET_FORCE_KG"])
        )
        validate_vise_closed(result, float(ctx.params["VISE_TARGET_FORCE_KG"]))
    elif step_number == 12:
        ctx.hold_current_joints("hold current pose after InsertComp before gripper release")
        ctx.gripper("release")
    elif step_number == 13:
        ctx.hold_current_joints("keep robot static while opening gripper fully")
        ctx.gripper("open")
        ctx.switch_to_primitive_mode("resume motion control for vise retreat")
        ctx.move_current_z_offset(
            "vise_retreat",
            float(ctx.params["VISE_RETREAT_Z_OFFSET_M"]),
        )
    elif step_number == 14:
        arduino = require_arduino(ctx)
        status = ensure_arduino_ready(ctx)
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
    elif step_number == 15:
        ctx.move_l(
            "Vise",
            cap_grip_pose(ctx),
            vel_m_s=float(ctx.params["CARTESIAN_INSERT_VEL_M_S"]),
        )
    elif step_number == 16:
        ctx.gripper("close")
    elif step_number == 17:
        ctx.twist_cap()
    elif step_number == 18:
        ctx.move_current_z_offset(
            "cap_lift",
            float(ctx.params["CAP_LIFT_Z_OFFSET_M"]),
        )
    elif step_number == 19:
        ctx.move_l(
            "Inter",
            ctx.poses["Inter"],
            vel_m_s=float(ctx.params["CARTESIAN_RETREAT_VEL_M_S"]),
        )
    elif step_number == 20:
        ctx.move_l(
            "Home",
            ctx.poses["Home"],
            vel_m_s=float(ctx.params["CARTESIAN_RETREAT_VEL_M_S"]),
        )
    elif step_number == 21:
        ctx.move_l(
            "plastic",
            ctx.poses["plastic"],
            vel_m_s=float(ctx.params["CARTESIAN_RETREAT_VEL_M_S"]),
        )
    elif step_number == 22:
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


def phase_pick_from_tray(ctx: RecipeContext) -> None:
    ctx.logger.info("=== phase_pick_from_tray ===")
    run_plan_steps(ctx, range(1, 9))


def phase_load_vise(ctx: RecipeContext) -> None:
    ctx.logger.info("=== phase_load_vise ===")
    run_plan_steps(ctx, range(9, 14))


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


def phase_cut(ctx: RecipeContext, *, yes: bool, skip_cut: bool = False) -> None:
    ctx.logger.info("=== phase_cut ===")
    execute_plan_step(ctx, 14, yes=yes, skip_cut=skip_cut)


def phase_twist_and_drop_cap(ctx: RecipeContext) -> None:
    ctx.logger.info("=== phase_twist_and_drop_cap ===")
    run_plan_steps(ctx, range(15, 23))


def phase_remove_spring(ctx: RecipeContext) -> None:
    ctx.logger.info("=== phase_remove_spring ===")
    run_plan_steps(ctx, range(23, 33))


def phase_remove_yellow_plastic(ctx: RecipeContext) -> None:
    ctx.logger.info("=== phase_remove_yellow_plastic ===")
    run_plan_steps(ctx, range(33, 43))


def phase_remove_shell(ctx: RecipeContext) -> None:
    ctx.logger.info("=== phase_remove_shell_and_glass ===")
    run_plan_steps(ctx, (43, 44, 45, 46, 47, 48, 49, 50, 51, 55, 56, 57, 58, 59))


def phase_return_inter(ctx: RecipeContext) -> None:
    ctx.logger.info("=== phase_return_inter ===")
    run_plan_steps(ctx, ())


def phase_shutdown(ctx: RecipeContext) -> None:
    ctx.logger.info("=== phase_shutdown ===")
    try:
        ctx.gripper("open")
    except Exception as exc:  # noqa: BLE001 - shutdown is best effort
        ctx.logger.warn(f"gripper open during shutdown failed: {exc}")
    if ctx.arduino is not None:
        try:
            status = ctx.arduino.get_status()
            if status.get("busy") or status.get("blade_on"):
                ctx.logger.warn(f"machine not idle at shutdown; STOP_ALL: {status}")
                ctx.arduino.stop_all()
        except Exception as exc:  # noqa: BLE001
            ctx.logger.warn(f"Arduino shutdown status/stop failed: {exc}")


def run_full_recipe(ctx: RecipeContext, *, yes: bool, skip_cut: bool = False) -> None:
    phase_pick_from_tray(ctx)
    phase_load_vise(ctx)
    phase_cut(ctx, yes=yes, skip_cut=skip_cut)
    phase_twist_and_drop_cap(ctx)
    phase_remove_spring(ctx)
    phase_remove_yellow_plastic(ctx)
    phase_remove_shell(ctx)
    phase_return_inter(ctx)
    phase_shutdown(ctx)


def run_robot_only_smoke(ctx: RecipeContext) -> None:
    ctx.logger.info("=== robot_only_smoke ===")
    ctx.logger.warn(
        "Robot-only smoke skips Arduino CLOSE_VISE and CUT_HEIGHT; "
        "use a safe test setup before running with a loose part."
    )
    for step_number in sorted(PLAN_STEPS):
        if step_requires_arduino(step_number):
            ctx.logger.info(
                f"=== step {step_number}: {PLAN_STEPS[step_number]} "
                "SKIPPED for robot-only smoke ==="
            )
            continue
        execute_plan_step(ctx, step_number)
    phase_shutdown(ctx)


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


def best_effort_stop_all(arduino, logger) -> None:
    if arduino is None:
        return
    try:
        arduino.stop_all()
    except Exception as exc:  # noqa: BLE001 - shutdown/fault path must not raise
        logger.warn(f"STOP_ALL failed during fault handling: {exc}")


def print_dry_run_summary(params: dict, key_dir: Path, poses: dict[str, dict], logger) -> None:
    logger.info("--- dry-run summary ---")
    logger.info(f"key_position_dir={key_dir}")
    logger.info(f"required_positions={', '.join(REQUIRED_POSITIONS)}")
    logger.info(f"loaded_positions={', '.join(sorted(poses))}")
    logger.info(
        "phase_order=pick_from_tray -> load_vise -> cut -> twist_and_drop_cap "
        "-> remove_spring -> remove_yellow_plastic -> remove_shell_and_glass "
        "-> shutdown"
    )
    for key in (
        "TRAY_APPROACH_Z_OFFSET_M",
        "TRAY_GRIP_Z_OFFSET_M",
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
        "INSERTCOMP_INSERT_VEL_M_S",
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
        type=int,
        default=None,
        help="Run one numbered plan step only, for example: --step 15",
    )
    parser.add_argument(
        "--robot-only-smoke",
        action="store_true",
        help="Exercise only the robot/gripper pose path; no Arduino vise/cut actions",
    )
    parser.add_argument("--yes", action="store_true", help="Skip CUT_HEIGHT confirmation prompt")
    parser.add_argument("--skip-cut", action="store_true", help="Run all phases except CUT_HEIGHT")
    parser.add_argument("--skip-gripper-init", action="store_true")
    parser.add_argument("--key-position-dir", default=None)
    parser.add_argument("--robot-sn", default=None)
    parser.add_argument("--arduino-port", default=None)
    parser.add_argument("--jnt-vel-scale", type=int, default=None)
    parser.add_argument("--tray-approach-z-m", type=float, default=None)
    parser.add_argument("--tray-grip-z-m", type=float, default=None)
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
    parser.add_argument("--insertcomp-insert-vel-m-s", type=float, default=None)
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
    overrides = {
        "ROBOT_SN": args.robot_sn,
        "ARDUINO_PORT": args.arduino_port,
        "MOVE_JNT_VEL_SCALE": args.jnt_vel_scale,
        "TRAY_APPROACH_Z_OFFSET_M": args.tray_approach_z_m,
        "TRAY_GRIP_Z_OFFSET_M": args.tray_grip_z_m,
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
        "INSERTCOMP_INSERT_VEL_M_S": args.insertcomp_insert_vel_m_s,
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
    selected_step = args.step

    try:
        if selected_step is not None:
            selected_step = validate_plan_step(int(selected_step))
        needs_arduino = not args.robot_only_smoke and (
            selected_step is None or step_requires_arduino(selected_step)
        )
        poses = load_positions(key_dir, logger)
        if args.dry_run:
            print_dry_run_summary(params, key_dir, poses, logger)
            arduino = build_arduino(args, params, logger, True) if needs_arduino else None
            ctx = RecipeContext(
                params=params,
                poses=poses,
                logger=logger,
                dry_run=True,
                session=None,
                arduino=arduino,
            )
            if selected_step is not None:
                execute_plan_step(ctx, selected_step, yes=True, skip_cut=args.skip_cut)
            elif args.robot_only_smoke:
                run_robot_only_smoke(ctx)
            else:
                run_full_recipe(ctx, yes=True, skip_cut=args.skip_cut)
            if arduino is not None:
                arduino.close()
                arduino = None
            return 0

        arduino = build_arduino(args, params, logger, False) if needs_arduino else None
        try:
            with RobotSession(params["ROBOT_SN"], logger=logger) as session:
                gripper_init = not args.skip_gripper_init
                if selected_step is not None:
                    if args.skip_gripper_init:
                        logger.info(
                            "Single-step mode: skipping gripper initialization "
                            "because --skip-gripper-init was requested"
                        )
                    elif step_uses_gripper(selected_step):
                        gripper_init = True
                        logger.info(
                            "Single-step mode: initializing gripper for direct "
                            f"gripper step {selected_step}"
                        )
                    else:
                        gripper_init = False
                        logger.info(
                            "Single-step mode: skipping gripper initialization "
                            f"for motion-only step {selected_step}"
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
                )
                if selected_step is not None:
                    execute_plan_step(ctx, selected_step, yes=args.yes, skip_cut=args.skip_cut)
                elif args.robot_only_smoke:
                    run_robot_only_smoke(ctx)
                else:
                    run_full_recipe(ctx, yes=args.yes, skip_cut=args.skip_cut)
            return 0
        except Exception:
            best_effort_stop_all(arduino, logger)
            raise
        finally:
            if arduino is not None:
                try:
                    arduino.close()
                except Exception:
                    pass
                arduino = None
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
