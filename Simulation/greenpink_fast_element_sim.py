#!/usr/bin/env python3
"""Simulate selected GreenPinkCameraFast robot motions in Flexiv Element.

This script is intentionally robot-only: it replays the recipe's arm motions
for dump, twist, and wiggle without touching the Arduino or camera stack.

Examples:

    python project/Simulation/greenpink_fast_element_sim.py
    python project/Simulation/greenpink_fast_element_sim.py dump
    python project/Simulation/greenpink_fast_element_sim.py twist wiggle
    python project/Simulation/greenpink_fast_element_sim.py --dry-run
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
from pathlib import Path
from typing import Any, Iterable


HERE = Path(__file__).resolve().parent
PROJECT_DIR = HERE.parent
REPO_DIR = PROJECT_DIR.parent
RECIPE_DIR = PROJECT_DIR / "recipe" / "GreenPinkCameraFast"
DEFAULT_KEY_POSITION_DIR = RECIPE_DIR / "key_positions"

if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from helper.flexiv_helpers import (  # noqa: E402
    RobotSession,
    joints_to_jpos_deg,
    move_ptp_joint,
    rotate_tcp_about_tool_axis,
    rpy_deg_to_quat,
    tcp_pose_to_coord_args,
    wiggle_about_virtual_point,
)


PARAMS: dict[str, Any] = {
    "ROBOT_SN": "Rizon4-062930",
    "KEY_POSITION_DIR": str(DEFAULT_KEY_POSITION_DIR),
    "MOVE_ZONE_RADIUS": "ZFine",
    "MOVE_TARGET_TOLER_LEVEL": 1,
    "MOVE_JNT_ACC_MULTIPLIER": 1.0,
    "MOVE_JNT_VEL_SCALE": 80,
    "FRAME5_MOVE_JNT_VEL_SCALE": 80,
    "FRAME5_CARTESIAN_VEL_M_S": 0.5,
    "CARTESIAN_INSERT_VEL_M_S": 0.06,
    "VISE_APPROACH_Z_OFFSET_M": 0.05,
    "CAP_GRIP_WORLD_Z_OFFSET_M": -0.02,
    "CAP_GRIP_TCP_Z_OFFSET_M": 0.01,
    "CAP_LIFT_Z_OFFSET_M": 0.20,
    "DUMP_TOOL_Z_DEG": 176.0,
    "DUMP_VP_OFFSET_X_M": 0.0,
    "DUMP_VP_OFFSET_Y_M": 0.02,
    "DUMP_VP_OFFSET_Z_M": 0.02,
    "DUMP_MOVEC_VEL_M_S": 0.03,
    "DUMP_MOVEC_ACC_M_S2": 0.10,
    "DUMP_MOVEC_JERK_M_S3": 100.0,
    "DUMP_MOVEC_EQUAL_RADIUS": 0.1,
    "CAP_TWIST_DEG": 7.0,
    "CAP_TWIST_NEG_DEG": -10.0,
    "CAP_TWIST_MOVE_JNT_VEL_SCALE": 20,
    "CAP_TWIST_REPEAT_COUNT": 1,
    # Wiggle defaults come from earlier break-off experiments.
    "WIGGLE_PIVOT_OFFSET_CM": 6.0,
    "WIGGLE_YAW_DEG": 15.0,
    "WIGGLE_REPEAT_COUNT": 2,
    "WIGGLE_VEL_SCALE": 10,
}

REQUIRED_POSITIONS = ("Middle", "Vise", "Glass", "Plastic")


class SimulationError(RuntimeError):
    """Raised when the simulator cannot load poses or execute a motion."""


def build_logger(verbose: bool) -> logging.Logger:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(level=level, format="%(asctime)s %(levelname)s %(message)s")
    return logging.getLogger("greenpink_fast_element_sim")


def normalize_robot_sn(robot_sn: str) -> str:
    text = str(robot_sn).strip()
    if not text:
        return str(PARAMS["ROBOT_SN"])
    if text.startswith("Rizon4-"):
        return text
    if text.isdigit():
        return f"Rizon4-{text}"
    return text


def robot_sn_candidates(robot_sn: str) -> list[str]:
    text = str(robot_sn).strip()
    if not text:
        text = str(PARAMS["ROBOT_SN"])
    candidates: list[str] = []
    candidate_values = [text, normalize_robot_sn(text)]
    if not text.startswith("Rizon4-"):
        candidate_values.append(text.removeprefix("Rizon4-"))
    for candidate in candidate_values:
        normalized = str(candidate).strip()
        if normalized and normalized not in candidates:
            candidates.append(normalized)
    return candidates


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
    logger.info("loaded key positions: %s", ", ".join(sorted(poses)))
    return poses


def load_key_position(key_dir: Path, name: str) -> dict:
    path = key_dir / f"{name}.json"
    if not path.is_file():
        raise SimulationError(f"missing key position: {path}")
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _fmt_pose_mm(pose_entry: dict) -> str:
    tcp = pose_entry["tcp_pose_world"]["values"]
    return f"x={tcp[0]*1000:.1f} y={tcp[1]*1000:.1f} z={tcp[2]*1000:.1f} mm"


def offset_pose_z(pose_entry: dict, dz_m: float) -> dict:
    tcp = list(pose_entry["tcp_pose_world"]["values"])
    tcp[2] += float(dz_m)
    return {
        "name": f"{pose_entry.get('name', 'pose')}_offset_z",
        "q_rad": list(pose_entry["q_rad"]),
        "tcp_pose_world": {
            "order": list(pose_entry["tcp_pose_world"]["order"]),
            "values": tcp,
        },
    }


def _quat_normalize(quat: Iterable[float]) -> list[float]:
    qw, qx, qy, qz = [float(v) for v in quat]
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm == 0.0:
        raise SimulationError("cannot normalize zero quaternion")
    return [qw / norm, qx / norm, qy / norm, qz / norm]


def _quat_multiply(a: Iterable[float], b: Iterable[float]) -> list[float]:
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


def _quat_to_matrix(quat: Iterable[float]) -> list[list[float]]:
    qw, qx, qy, qz = _quat_normalize(quat)
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


def _mat_vec_mul(matrix: list[list[float]], vec: Iterable[float]) -> list[float]:
    vec_values = [float(v) for v in vec]
    return [
        sum(matrix[row][col] * vec_values[col] for col in range(3)) for row in range(3)
    ]


def offset_pose_tcp_axis(pose_entry: dict, axis: str, distance_m: float) -> dict:
    tcp = list(pose_entry["tcp_pose_world"]["values"])
    axis_text = str(axis).strip().lower()
    sign = 1.0
    if axis_text.startswith("-"):
        sign = -1.0
        axis_text = axis_text[1:]
    axis_to_col = {"x": 0, "y": 1, "z": 2}
    if axis_text not in axis_to_col:
        raise SimulationError(f"unsupported TCP axis {axis!r}")
    rotation = _quat_to_matrix(tcp[3:])
    column_index = axis_to_col[axis_text]
    distance = sign * float(distance_m)
    for world_index in range(3):
        tcp[world_index] += rotation[world_index][column_index] * distance
    return {
        "name": f"{pose_entry.get('name', 'pose')}_offset_tcp_{axis_text}",
        "q_rad": list(pose_entry["q_rad"]),
        "tcp_pose_world": {
            "order": list(pose_entry["tcp_pose_world"]["order"]),
            "values": tcp,
        },
    }


def rotate_tcp_about_virtual_offset(
    tcp_pose: Iterable[float],
    *,
    offset_m_xyz: Iterable[float],
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
    yaw_deg: float = 0.0,
) -> list[float]:
    pose = [float(v) for v in tcp_pose]
    offset = [float(v) for v in offset_m_xyz]
    if len(pose) != 7:
        raise SimulationError(f"tcp_pose must have 7 values, got {len(pose)}")
    if len(offset) != 3:
        raise SimulationError(f"offset_m_xyz must have 3 values, got {len(offset)}")
    position = pose[:3]
    current_quat = _quat_normalize(pose[3:])
    current_matrix = _quat_to_matrix(current_quat)
    world_offset = _mat_vec_mul(current_matrix, offset)
    pivot_world = [position[index] + world_offset[index] for index in range(3)]
    delta_quat = rpy_deg_to_quat(roll_deg, pitch_deg, yaw_deg)
    target_quat = _quat_multiply(current_quat, delta_quat)
    target_matrix = _quat_to_matrix(target_quat)
    rotated_offset = _mat_vec_mul(target_matrix, offset)
    target_position = [
        pivot_world[index] - rotated_offset[index] for index in range(3)
    ]
    return target_position + list(target_quat)


def movec_geometry_is_valid(
    start_tcp: Iterable[float],
    middle_tcp: Iterable[float],
    target_tcp: Iterable[float],
) -> bool:
    start_pos = [float(v) for v in list(start_tcp)[:3]]
    middle_pos = [float(v) for v in list(middle_tcp)[:3]]
    target_pos = [float(v) for v in list(target_tcp)[:3]]
    start_to_middle = [middle_pos[i] - start_pos[i] for i in range(3)]
    start_to_target = [target_pos[i] - start_pos[i] for i in range(3)]
    norm_middle = math.sqrt(sum(value * value for value in start_to_middle))
    norm_target = math.sqrt(sum(value * value for value in start_to_target))
    if norm_middle < 1e-5 or norm_target < 1e-5:
        return False
    cross = [
        start_to_middle[1] * start_to_target[2]
        - start_to_middle[2] * start_to_target[1],
        start_to_middle[2] * start_to_target[0]
        - start_to_middle[0] * start_to_target[2],
        start_to_middle[0] * start_to_target[1]
        - start_to_middle[1] * start_to_target[0],
    ]
    return math.sqrt(sum(value * value for value in cross)) > 1e-8


def current_pose_offset_z(session: RobotSession, dz_m: float) -> dict:
    _, state = session.selected_arm_state()
    q_rad = [float(v) for v in getattr(state, "q", [])]
    tcp = [float(v) for v in getattr(state, "tcp_pose", [])]
    if len(q_rad) != 7 or len(tcp) != 7:
        raise SimulationError(
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


class SimulationContext:
    def __init__(self, args: argparse.Namespace, logger: logging.Logger) -> None:
        self.params = build_params(args)
        self.logger = logger
        self.dry_run = bool(args.dry_run)
        self.session: RobotSession | None = None
        self.key_position_dir = Path(str(self.params["KEY_POSITION_DIR"])).resolve()
        self.poses = load_positions(self.key_position_dir, logger)

    def is_dry_run(self) -> bool:
        return bool(self.dry_run)

    def move_ptp(
        self,
        label: str,
        pose_entry: dict,
        *,
        vel_scale: int,
        use_ref_joints: bool = False,
    ) -> None:
        self.logger.info(
            "MovePTP -> %s (%s; jntVelScale=%d; useRefJoints=%s)",
            label,
            _fmt_pose_mm(pose_entry),
            int(vel_scale),
            bool(use_ref_joints),
        )
        if self.is_dry_run():
            return
        if self.session is None:
            raise SimulationError("robot session is required for live motion")
        move_ptp_joint(
            self.session,
            pose_entry,
            vel_scale=int(vel_scale),
            zone_radius=str(self.params["MOVE_ZONE_RADIUS"]),
            target_toler_level=int(self.params["MOVE_TARGET_TOLER_LEVEL"]),
            jnt_acc_multiplier=float(self.params["MOVE_JNT_ACC_MULTIPLIER"]),
            joint_locked=False,
            use_ref_joints=bool(use_ref_joints),
        )

    def move_l(
        self,
        label: str,
        pose_entry: dict,
        *,
        vel_m_s: float,
    ) -> None:
        self.logger.info(
            "MoveL -> %s (%s; vel=%.3f m/s)",
            label,
            _fmt_pose_mm(pose_entry),
            float(vel_m_s),
        )
        if self.is_dry_run():
            return
        if self.session is None:
            raise SimulationError("robot session is required for live motion")
        flexivrdk = self.session.flexivrdk
        joints_deg = joints_to_jpos_deg(pose_entry["q_rad"])
        coord = flexivrdk.Coord(
            *tcp_pose_to_coord_args(
                pose_entry["tcp_pose_world"]["values"],
                ref_joints_deg=joints_deg,
            )
        )
        self.session.execute_primitive(
            "MoveL",
            {
                "target": coord,
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
            "MoveC -> %s (mid %s -> target %s; vel=%.3f m/s)",
            label,
            _fmt_pose_mm(middle_pose_entry),
            _fmt_pose_mm(target_pose_entry),
            float(vel_m_s),
        )
        if self.is_dry_run():
            return
        if self.session is None:
            raise SimulationError("robot session is required for live motion")
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

    def above_vise_pose(self) -> dict:
        return offset_pose_z(self.poses["Vise"], float(self.params["VISE_APPROACH_Z_OFFSET_M"]))

    def cap_grip_world_pose(self) -> dict:
        return offset_pose_z(self.poses["Vise"], float(self.params["CAP_GRIP_WORLD_Z_OFFSET_M"]))

    def cap_grip_tcp_pose(self, base_pose: dict) -> dict:
        return offset_pose_tcp_axis(base_pose, "z", float(self.params["CAP_GRIP_TCP_Z_OFFSET_M"]))

    def run_dump(self) -> None:
        self.logger.info("=== dump simulation ===")
        self.move_ptp(
            "Middle",
            self.poses["Middle"],
            vel_scale=int(self.params["FRAME5_MOVE_JNT_VEL_SCALE"]),
        )
        self.move_ptp(
            "Glass",
            self.poses["Glass"],
            vel_scale=int(self.params["FRAME5_MOVE_JNT_VEL_SCALE"]),
        )
        start_q_rad = list(self.poses["Glass"]["q_rad"])
        start_tcp = list(self.poses["Glass"]["tcp_pose_world"]["values"])
        if not self.is_dry_run():
            if self.session is None:
                raise SimulationError("robot session is required for live motion")
            _, state = self.session.selected_arm_state()
            start_q_rad = [float(value) for value in getattr(state, "q", [])]
            start_tcp = [float(value) for value in getattr(state, "tcp_pose", [])]
        pivot_offset = [
            float(self.params["DUMP_VP_OFFSET_X_M"]),
            float(self.params["DUMP_VP_OFFSET_Y_M"]),
            float(self.params["DUMP_VP_OFFSET_Z_M"]),
        ]
        middle_tcp = rotate_tcp_about_virtual_offset(
            start_tcp,
            offset_m_xyz=pivot_offset,
            yaw_deg=float(self.params["DUMP_TOOL_Z_DEG"]) / 2.0,
        )
        target_tcp = rotate_tcp_about_virtual_offset(
            start_tcp,
            offset_m_xyz=pivot_offset,
            yaw_deg=float(self.params["DUMP_TOOL_Z_DEG"]),
        )
        if not movec_geometry_is_valid(start_tcp, middle_tcp, target_tcp):
            raise SimulationError("dump MoveC geometry is invalid")
        start_pose = {
            "name": "dump_start",
            "q_rad": start_q_rad,
            "tcp_pose_world": {
                "order": ["x", "y", "z", "qw", "qx", "qy", "qz"],
                "values": start_tcp,
            },
        }
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
        self.move_c(
            "dump_tool_z",
            middle_pose,
            target_pose,
            vel_m_s=float(self.params["DUMP_MOVEC_VEL_M_S"]),
            acc_m_s2=float(self.params["DUMP_MOVEC_ACC_M_S2"]),
            jerk_m_s3=float(self.params["DUMP_MOVEC_JERK_M_S3"]),
            equal_radius=float(self.params["DUMP_MOVEC_EQUAL_RADIUS"]),
        )
        self.move_c(
            "dump_tool_z_return",
            middle_pose,
            start_pose,
            vel_m_s=float(self.params["DUMP_MOVEC_VEL_M_S"]),
            acc_m_s2=float(self.params["DUMP_MOVEC_ACC_M_S2"]),
            jerk_m_s3=float(self.params["DUMP_MOVEC_JERK_M_S3"]),
            equal_radius=float(self.params["DUMP_MOVEC_EQUAL_RADIUS"]),
        )
        self.move_ptp(
            "Plastic",
            self.poses["Plastic"],
            vel_scale=int(self.params["FRAME5_MOVE_JNT_VEL_SCALE"]),
        )
        self.move_ptp(
            "Middle",
            self.poses["Middle"],
            vel_scale=int(self.params["FRAME5_MOVE_JNT_VEL_SCALE"]),
        )

    def run_twist(self) -> None:
        self.logger.info("=== twist simulation ===")
        self.move_ptp(
            "Middle",
            self.poses["Middle"],
            vel_scale=int(self.params["MOVE_JNT_VEL_SCALE"]),
        )
        self.move_ptp(
            "above_vise",
            self.above_vise_pose(),
            vel_scale=int(self.params["MOVE_JNT_VEL_SCALE"]),
        )
        cap_world_pose = self.cap_grip_world_pose()
        self.move_l(
            "cap_grip_world_z",
            cap_world_pose,
            vel_m_s=float(self.params["CARTESIAN_INSERT_VEL_M_S"]),
        )
        cap_tcp_pose = self.cap_grip_tcp_pose(cap_world_pose)
        self.move_l(
            "cap_grip_tcp_z",
            cap_tcp_pose,
            vel_m_s=float(self.params["CARTESIAN_INSERT_VEL_M_S"]),
        )
        self.twist_cap()
        if self.is_dry_run():
            self.logger.info(
                "MoveL -> cap_lift (current world Z + %.1f mm; vel=%.3f m/s)",
                float(self.params["CAP_LIFT_Z_OFFSET_M"]) * 1000.0,
                float(self.params["CARTESIAN_INSERT_VEL_M_S"]),
            )
        else:
            if self.session is None:
                raise SimulationError("robot session is required for live motion")
            lift_pose = current_pose_offset_z(
                self.session,
                float(self.params["CAP_LIFT_Z_OFFSET_M"]),
            )
            self.move_l(
                "cap_lift",
                lift_pose,
                vel_m_s=float(self.params["CARTESIAN_INSERT_VEL_M_S"]),
            )
        self.move_ptp(
            "Middle",
            self.poses["Middle"],
            vel_scale=int(self.params["MOVE_JNT_VEL_SCALE"]),
        )

    def twist_cap(self) -> None:
        self.logger.info(
            "Twist cap: TCP X 0 -> %+0.1f -> %+0.1f repeat=%d jntVelScale=%d",
            float(self.params["CAP_TWIST_DEG"]),
            float(self.params["CAP_TWIST_NEG_DEG"]),
            int(self.params["CAP_TWIST_REPEAT_COUNT"]),
            int(self.params["CAP_TWIST_MOVE_JNT_VEL_SCALE"]),
        )
        if self.is_dry_run():
            return
        if self.session is None:
            raise SimulationError("robot session is required for live motion")
        _, state = self.session.selected_arm_state()
        start_q_rad = [float(value) for value in getattr(state, "q", [])]
        start_tcp = [float(value) for value in getattr(state, "tcp_pose", [])]
        for index in range(int(self.params["CAP_TWIST_REPEAT_COUNT"])):
            positive_pose = {
                "name": f"twist_{index + 1}_positive",
                "q_rad": start_q_rad,
                "tcp_pose_world": {
                    "order": ["x", "y", "z", "qw", "qx", "qy", "qz"],
                    "values": rotate_tcp_about_tool_axis(
                        start_tcp,
                        roll_deg=float(self.params["CAP_TWIST_DEG"]),
                    ),
                },
            }
            negative_pose = {
                "name": f"twist_{index + 1}_negative",
                "q_rad": start_q_rad,
                "tcp_pose_world": {
                    "order": ["x", "y", "z", "qw", "qx", "qy", "qz"],
                    "values": rotate_tcp_about_tool_axis(
                        start_tcp,
                        roll_deg=float(self.params["CAP_TWIST_NEG_DEG"]),
                    ),
                },
            }
            self.move_ptp(
                positive_pose["name"],
                positive_pose,
                vel_scale=int(self.params["CAP_TWIST_MOVE_JNT_VEL_SCALE"]),
                use_ref_joints=True,
            )
            self.move_ptp(
                negative_pose["name"],
                negative_pose,
                vel_scale=int(self.params["CAP_TWIST_MOVE_JNT_VEL_SCALE"]),
                use_ref_joints=True,
            )

    def run_wiggle(self) -> None:
        self.logger.info("=== wiggle simulation ===")
        self.move_ptp(
            "Middle",
            self.poses["Middle"],
            vel_scale=int(self.params["MOVE_JNT_VEL_SCALE"]),
        )
        self.move_ptp(
            "above_vise",
            self.above_vise_pose(),
            vel_scale=int(self.params["MOVE_JNT_VEL_SCALE"]),
        )
        cap_world_pose = self.cap_grip_world_pose()
        self.move_l(
            "wiggle_grip_world_z",
            cap_world_pose,
            vel_m_s=float(self.params["CARTESIAN_INSERT_VEL_M_S"]),
        )
        cap_tcp_pose = self.cap_grip_tcp_pose(cap_world_pose)
        self.move_l(
            "wiggle_grip_tcp_z",
            cap_tcp_pose,
            vel_m_s=float(self.params["CARTESIAN_INSERT_VEL_M_S"]),
        )
        self.logger.info(
            "Wiggle: yaw=+/-%.1f deg about TCP +X pivot %.1f cm, repeat=%d, jntVelScale=%d",
            float(self.params["WIGGLE_YAW_DEG"]),
            float(self.params["WIGGLE_PIVOT_OFFSET_CM"]),
            int(self.params["WIGGLE_REPEAT_COUNT"]),
            int(self.params["WIGGLE_VEL_SCALE"]),
        )
        if not self.is_dry_run():
            if self.session is None:
                raise SimulationError("robot session is required for live motion")
            wiggle_about_virtual_point(
                self.session,
                pivot_offset_cm=float(self.params["WIGGLE_PIVOT_OFFSET_CM"]),
                yaw_deg=float(self.params["WIGGLE_YAW_DEG"]),
                repeat_count=int(self.params["WIGGLE_REPEAT_COUNT"]),
                vel_scale=int(self.params["WIGGLE_VEL_SCALE"]),
                zone_radius=str(self.params["MOVE_ZONE_RADIUS"]),
            )
        if self.is_dry_run():
            self.logger.info(
                "MoveL -> wiggle_lift (current world Z + %.1f mm; vel=%.3f m/s)",
                float(self.params["CAP_LIFT_Z_OFFSET_M"]) * 1000.0,
                float(self.params["CARTESIAN_INSERT_VEL_M_S"]),
            )
        else:
            if self.session is None:
                raise SimulationError("robot session is required for live motion")
            lift_pose = current_pose_offset_z(
                self.session,
                float(self.params["CAP_LIFT_Z_OFFSET_M"]),
            )
            self.move_l(
                "wiggle_lift",
                lift_pose,
                vel_m_s=float(self.params["CARTESIAN_INSERT_VEL_M_S"]),
            )
        self.move_ptp(
            "Middle",
            self.poses["Middle"],
            vel_scale=int(self.params["MOVE_JNT_VEL_SCALE"]),
        )


def build_params(args: argparse.Namespace) -> dict[str, Any]:
    params = dict(PARAMS)
    overrides = {
        "ROBOT_SN": args.robot_sn,
        "KEY_POSITION_DIR": args.key_position_dir,
        "VISE_APPROACH_Z_OFFSET_M": args.vise_approach_z_offset_m,
        "CAP_GRIP_WORLD_Z_OFFSET_M": args.cap_grip_world_z_offset_m,
        "CAP_GRIP_TCP_Z_OFFSET_M": args.cap_grip_tcp_z_offset_m,
        "CAP_LIFT_Z_OFFSET_M": args.cap_lift_z_offset_m,
        "DUMP_TOOL_Z_DEG": args.dump_tool_z_deg,
        "DUMP_VP_OFFSET_X_M": args.dump_vp_offset_x_m,
        "DUMP_VP_OFFSET_Y_M": args.dump_vp_offset_y_m,
        "DUMP_VP_OFFSET_Z_M": args.dump_vp_offset_z_m,
        "DUMP_MOVEC_VEL_M_S": args.dump_movec_vel_m_s,
        "DUMP_MOVEC_ACC_M_S2": args.dump_movec_acc_m_s2,
        "DUMP_MOVEC_JERK_M_S3": args.dump_movec_jerk_m_s3,
        "DUMP_MOVEC_EQUAL_RADIUS": args.dump_movec_equal_radius,
        "CAP_TWIST_DEG": args.cap_twist_deg,
        "CAP_TWIST_NEG_DEG": args.cap_twist_neg_deg,
        "CAP_TWIST_REPEAT_COUNT": args.cap_twist_repeat_count,
        "CAP_TWIST_MOVE_JNT_VEL_SCALE": args.cap_twist_jnt_vel_scale,
        "WIGGLE_PIVOT_OFFSET_CM": args.wiggle_pivot_offset_cm,
        "WIGGLE_YAW_DEG": args.wiggle_yaw_deg,
        "WIGGLE_REPEAT_COUNT": args.wiggle_repeat_count,
        "WIGGLE_VEL_SCALE": args.wiggle_vel_scale,
    }
    for key, value in overrides.items():
        params[key] = value
    return params


def run_simulation(ctx: SimulationContext, motions: list[str]) -> None:
    ctx.logger.info("Using key positions from %s", ctx.key_position_dir)
    candidates = robot_sn_candidates(str(ctx.params["ROBOT_SN"]))
    ctx.logger.info("Robot connection candidates: %s", ", ".join(candidates))
    if ctx.is_dry_run():
        for motion in motions:
            getattr(ctx, f"run_{motion}")()
        return
    last_error: Exception | None = None
    for index, robot_sn in enumerate(candidates, start=1):
        try:
            ctx.logger.info(
                "Connecting to Flexiv robot candidate %d/%d: %s",
                index,
                len(candidates),
                robot_sn,
            )
            with RobotSession(robot_sn, logger=ctx.logger) as session:
                ctx.session = session
                session.switch_mode("NRT_PRIMITIVE_EXECUTION")
                for motion in motions:
                    getattr(ctx, f"run_{motion}")()
            ctx.session = None
            return
        except Exception as exc:  # noqa: BLE001 - surface the most actionable controller error
            ctx.session = None
            message = str(exc)
            if "E-stop is not released" in message:
                raise SimulationError(
                    "Connected to the Flexiv robot, but the E-stop is still active. "
                    "Release/reset the E-stop in Flexiv Element for robot "
                    f"{robot_sn}, then rerun the simulator."
                ) from exc
            last_error = exc
            ctx.logger.warning(
                "Robot connection failed for %s: %s",
                robot_sn,
                exc,
            )
    attempted = ", ".join(candidates)
    raise SimulationError(
        "Unable to connect to the Flexiv robot in Element. "
        f"Tried: {attempted}. "
        "If the simulated robot is already open in Element, pass its exact "
        "displayed device ID with --robot-sn."
    ) from last_error


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Simulate GreenPinkCameraFast dump, twist, and wiggle motions."
    )
    parser.add_argument(
        "motions",
        nargs="*",
        choices=("all", "dump", "twist", "wiggle"),
        help="Motion demos to run. Default: all",
    )
    parser.add_argument("--robot-sn", default=PARAMS["ROBOT_SN"])
    parser.add_argument("--key-position-dir", default=PARAMS["KEY_POSITION_DIR"])
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--vise-approach-z-offset-m", type=float, default=PARAMS["VISE_APPROACH_Z_OFFSET_M"])
    parser.add_argument("--cap-grip-world-z-offset-m", type=float, default=PARAMS["CAP_GRIP_WORLD_Z_OFFSET_M"])
    parser.add_argument("--cap-grip-tcp-z-offset-m", type=float, default=PARAMS["CAP_GRIP_TCP_Z_OFFSET_M"])
    parser.add_argument("--cap-lift-z-offset-m", type=float, default=PARAMS["CAP_LIFT_Z_OFFSET_M"])
    parser.add_argument("--dump-tool-z-deg", type=float, default=PARAMS["DUMP_TOOL_Z_DEG"])
    parser.add_argument("--dump-vp-offset-x-m", type=float, default=PARAMS["DUMP_VP_OFFSET_X_M"])
    parser.add_argument("--dump-vp-offset-y-m", type=float, default=PARAMS["DUMP_VP_OFFSET_Y_M"])
    parser.add_argument("--dump-vp-offset-z-m", type=float, default=PARAMS["DUMP_VP_OFFSET_Z_M"])
    parser.add_argument("--dump-movec-vel-m-s", type=float, default=PARAMS["DUMP_MOVEC_VEL_M_S"])
    parser.add_argument("--dump-movec-acc-m-s2", type=float, default=PARAMS["DUMP_MOVEC_ACC_M_S2"])
    parser.add_argument("--dump-movec-jerk-m-s3", type=float, default=PARAMS["DUMP_MOVEC_JERK_M_S3"])
    parser.add_argument("--dump-movec-equal-radius", type=float, default=PARAMS["DUMP_MOVEC_EQUAL_RADIUS"])
    parser.add_argument("--cap-twist-deg", type=float, default=PARAMS["CAP_TWIST_DEG"])
    parser.add_argument("--cap-twist-neg-deg", type=float, default=PARAMS["CAP_TWIST_NEG_DEG"])
    parser.add_argument("--cap-twist-repeat-count", type=int, default=PARAMS["CAP_TWIST_REPEAT_COUNT"])
    parser.add_argument("--cap-twist-jnt-vel-scale", type=int, default=PARAMS["CAP_TWIST_MOVE_JNT_VEL_SCALE"])
    parser.add_argument("--wiggle-pivot-offset-cm", type=float, default=PARAMS["WIGGLE_PIVOT_OFFSET_CM"])
    parser.add_argument("--wiggle-yaw-deg", type=float, default=PARAMS["WIGGLE_YAW_DEG"])
    parser.add_argument("--wiggle-repeat-count", type=int, default=PARAMS["WIGGLE_REPEAT_COUNT"])
    parser.add_argument("--wiggle-vel-scale", type=int, default=PARAMS["WIGGLE_VEL_SCALE"])
    return parser.parse_args()


def resolve_motion_list(requested: list[str]) -> list[str]:
    if not requested or "all" in requested:
        return ["dump", "twist", "wiggle"]
    return requested


def main() -> int:
    args = parse_args()
    logger = build_logger(args.verbose)
    motions = resolve_motion_list(list(args.motions))
    ctx = SimulationContext(args, logger)
    run_simulation(ctx, motions)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
