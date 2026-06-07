#!/usr/bin/env python

"""Interactively record named robot waypoints for later trajectory playback."""

import argparse
import json
import math
import os
import re
import shlex
import sys
import time
from datetime import datetime
from pathlib import Path

import flexivrdk
import spdlog


ROBOT_SN = "Rizon4-062930"
DEFAULT_OUTPUT = "trajectory_waypoints.txt"
DEFAULT_KEY_POSITIONS_DIR = "key_positions"
KEY_POSITIONS_INDEX = "index.json"
DEFAULT_KEY_MOVE_JNT_VEL_SCALE = 10
DEFAULT_KEY_MOVE_JNT_ACC_MULTIPLIER = 1.0
DEFAULT_KEY_MOVE_ZONE_RADIUS = "ZFine"
DEFAULT_KEY_MOVE_TARGET_TOLER_LEVEL = 1
DEFAULT_ORTHO_ANGLE_STEP_DEG = 90.0
DEFAULT_ORTHO_JNT_VEL_SCALE = 5
DEFAULT_ORTHO_JNT_ACC_MULTIPLIER = 1.0
DEFAULT_ORTHO_ZONE_RADIUS = "ZFine"
DEFAULT_ORTHO_TARGET_TOLER_LEVEL = 1
DEFAULT_TOOL_MOVE_LINEAR_VEL = 0.02
DEFAULT_TOOL_MOVE_JNT_VEL_SCALE = 5
DEFAULT_TOOL_MOVE_JNT_ACC_MULTIPLIER = 1.0
DEFAULT_TOOL_MOVE_ZONE_RADIUS = "ZFine"
DEFAULT_TOOL_MOVE_TARGET_TOLER_LEVEL = 1
DEFAULT_TWIST_DEG = 5.0
DEFAULT_TWIST_REPEAT_COUNT = 1
HOLD_MAX_VEL = 1.0
HOLD_MAX_ACC = 2.0
GRIPPER_ID = "Flexiv-GN01"
INIT_WAIT_SEC = 4.0
DEFAULT_OPEN_WIDTH = 0.04
DEFAULT_CLOSE_WIDTH = 0.0
DEFAULT_VELOCITY = 0.05
DEFAULT_OPEN_FORCE_LIMIT = 10.0
DEFAULT_CLOSE_FORCE_LIMIT = 40.0
DEFAULT_ROTATE_JNT_VEL_SCALE = 5
DEFAULT_ROTATE_ZONE_RADIUS = "ZFine"
DEFAULT_ROTATE_TARGET_TOLER_LEVEL = 1
DEFAULT_ROTATE_JNT_ACC_MULTIPLIER = 1.0
DEFAULT_ROTATE_VP_PRIMITIVE = "movec"
DEFAULT_ROTATE_VP_VEL = 0.03
DEFAULT_ROTATE_VP_ACC = 0.1
DEFAULT_ROTATE_VP_JERK = 100.0
DEFAULT_ROTATE_VP_EQUAL_RADIUS = 0.1
DEFAULT_GRASP_FORCE = 50.0
DEFAULT_GRASP_CONTACT_FORCE = 8.0
DEFAULT_GRASP_VEL = 0.01
MAX_GRASPCOMP_GRIP_VEL = 0.05
DEFAULT_GRASP_MAX_VEL_FORCE_DIR = 0.02
DEFAULT_GRASP_TARGET_WIDTH = 0.0
DEFAULT_GRASP_OPEN_WIDTH = 0.06
DEFAULT_GRASP_PRECONTACT_MOVE_Z_M = 0.20
DEFAULT_GRASP_PRECONTACT_MOVE_VEL = 0.15
DEFAULT_GRASP_CONTACT_VEL = 0.05
DEFAULT_GRASP_CONTACT_ENABLE_FINE_CONTACT = 1
DEFAULT_GRASP_ZEROFT_DATA_COLLECT_TIME = 0.2
DEFAULT_GRASP_ZEROFT_ENABLE_STATIC_CHECK = 0
DEFAULT_GRASP_ZEROFT_CALIB_EXTRA_PAYLOAD = 0
DEFAULT_ZEROFT_TIMEOUT_SEC = 5.0
DEFAULT_CONTACT_TIMEOUT_SEC = 15.0
DEFAULT_GRASP_TIMEOUT_SEC = 15.0
DEFAULT_POLL_SEC = 0.2
DEFAULT_FLOATING_CARTESIAN_MAX_VEL = 0.5
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
DEFAULT_INJECTABLE_KEY_POSITION = "Plate"
DEFAULT_INJECTABLE_CALIBRATION = os.path.join(
    SCRIPT_DIR, "calibration", "camera_tcp.yaml"
)
DEFAULT_INJECTABLE_CAPTURE_OUT = os.path.join(REPO_DIR, "injectable_capture.jpg")
DEFAULT_INJECTABLE_DETECT_OUT = os.path.join(
    REPO_DIR, "injectable_capture_detected.jpg"
)
DEFAULT_INJECTABLE_ALIGN_OUT = os.path.join(REPO_DIR, "injectable_align_detected.jpg")
DEFAULT_INJECTABLE_ALIGN_PRIMITIVE = "movel"
DEFAULT_INJECTABLE_ALIGN_LINEAR_VEL = 0.02
DEFAULT_INJECTABLE_ALIGN_OFFSET_TOWARD_PINK_M = 0.02
# Residual correction along post-align TCP Y axis. 0 means no compensation
# (default since center is computed as the on-axis midpoint of pink/teal).
DEFAULT_INJECTABLE_ALIGN_OFFSET_TCP_Y_M = 0.0
DEFAULT_CALI_VISE_KEY_POSITION = "Vise"
DEFAULT_CALI_VISE_BOARD = os.path.join(SCRIPT_DIR, "calibration", "tag_01.yaml")
DEFAULT_CALI_VISE_CALIBRATION = os.path.join(
    SCRIPT_DIR, "calibration", "camera_tcp.yaml"
)
DEFAULT_CALI_VISE_REFERENCE = os.path.join(
    SCRIPT_DIR, "calibration", "tag_01_to_vise_tcp.json"
)
DEFAULT_CALI_VISE_CAPTURE_OUT = os.path.join(
    REPO_DIR, "calibration", "captures", "cali_vise_raw.png"
)
DEFAULT_CALI_VISE_DETECT_OUT = os.path.join(
    REPO_DIR, "calibration", "captures", "cali_vise_axes.png"
)
DEFAULT_CALI_VISE_MIN_CHARUCO_CORNERS = 8
# Tag registry: each tag carries a board YAML plus a list of key positions
# rigidly attached to it. Each key-position entry says which key position to
# UPDATE at runtime, the board-to-key-pos reference JSON, and whether to
# constrain the recovered TCP X axis to world vertical (vise gripper only).
# Ground-truth files used by --save-tag-reference live in capture_tag_axes.py's
# TAG_REGISTRY (Vise_accurate / Plate / Spring / Plastic / Glass) and are
# intentionally separate.
TAG_REGISTRY = {
    1: {
        "board": os.path.join(SCRIPT_DIR, "calibration", "tag_01.yaml"),
        "key_positions": [
            {
                "name": "Vise",
                "reference": os.path.join(
                    SCRIPT_DIR, "calibration", "tag_01_to_vise_tcp.json"
                ),
                "constrain_x_vertical": True,
            },
        ],
    },
    2: {
        "board": os.path.join(SCRIPT_DIR, "calibration", "tag_02.yaml"),
        "key_positions": [
            {
                "name": "Plate",
                "reference": os.path.join(
                    SCRIPT_DIR, "calibration", "tag_02_to_plate_tcp.json"
                ),
                "constrain_x_vertical": False,
            },
        ],
    },
    3: {
        "board": os.path.join(SCRIPT_DIR, "calibration", "tag_03.yaml"),
        "key_positions": [
            {
                "name": "Spring",
                "reference": os.path.join(
                    SCRIPT_DIR, "calibration", "tag_03_to_spring_tcp.json"
                ),
                "constrain_x_vertical": False,
            },
            {
                "name": "Plastic",
                "reference": os.path.join(
                    SCRIPT_DIR, "calibration", "tag_03_to_plastic_tcp.json"
                ),
                "constrain_x_vertical": False,
            },
            {
                "name": "Glass",
                "reference": os.path.join(
                    SCRIPT_DIR, "calibration", "tag_03_to_glass_tcp.json"
                ),
                "constrain_x_vertical": False,
            },
        ],
    },
}
DEFAULT_CALI_TAG_ID = 1
MODE_VALUES = {
    "NRT_PRIMITIVE_EXECUTION": 8,
    "NRT_JOINT_IMPEDANCE": 4,
}
FLOATING_DAMPING_LEVEL = [0.0] * 7
FLOATING_RESPONSE_TORQUE = [1.5, 1.5, 1.5, 1.5, 0.5, 0.5, 0.3]
FLOATING_LOAD_COMPENSATION_SCALE = 1.2
FLOATING_CARTESIAN_AXIS_NAMES = ["x", "y", "z", "rx", "ry", "rz"]
FLOATING_CARTESIAN_DAMPING_LEVEL = [0.0] * 6
FLOATING_CARTESIAN_INERTIA_SCALE = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
FLOATING_FRICTION_COMP_SCALE = [100.0] * 7


def project_path(filename):
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)


def safe_filename_slug(name):
    slug = re.sub(r"[^A-Za-z0-9._-]+", "_", name.strip()).strip("._-")
    return slug or "unnamed"


def prompt_trajectory_name(args):
    if args.trajectory_name:
        return args.trajectory_name.strip()

    name = input("Trajectory name (blank = trajectory_waypoints): ").strip()
    return name or "trajectory_waypoints"


def output_path_for_trajectory(args, trajectory_name):
    if args.output:
        return os.path.abspath(args.output)

    if trajectory_name == "trajectory_waypoints":
        return os.path.abspath(project_path(DEFAULT_OUTPUT))

    return os.path.abspath(
        project_path(f"trajectory_waypoints_{safe_filename_slug(trajectory_name)}.txt")
    )


def key_positions_dir(args):
    if args.key_position_dir:
        return os.path.abspath(args.key_position_dir)
    return os.path.abspath(project_path(DEFAULT_KEY_POSITIONS_DIR))


def load_injectable_detector_module():
    camera_dir = project_path("camera")
    if camera_dir not in sys.path:
        sys.path.insert(0, camera_dir)
    import detect_injectable_static  # type: ignore

    return detect_injectable_static


def load_handeye_module():
    calibration_dir = project_path("calibration")
    if calibration_dir not in sys.path:
        sys.path.insert(0, calibration_dir)
    import calibrate_eye_in_hand as handeye  # type: ignore

    return handeye


def require_realsense():
    try:
        import pyrealsense2 as rs  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "Missing dependency: pyrealsense2. Install it with: pip install pyrealsense2"
        ) from exc
    return rs


def selected_robot_state(robot):
    states = robot.states()

    if not isinstance(states, dict):
        return None, states

    for group, state in states.items():
        if "ARM" in str(group).upper():
            return str(group), state

    group, state = next(iter(states.items()))
    return str(group), state


def vector(values):
    return [float(value) for value in values]


def clamp(value, low, high):
    return max(low, min(value, high))


def dof_from_state(robot, state):
    q = vector(state.q)
    if q:
        return len(q)
    return int(robot.info().DoF)


def sized_joint_vector(values, dof):
    values = [float(value) for value in values]
    if len(values) >= dof:
        return values[:dof]
    return values + [values[-1] if values else 0.0] * (dof - len(values))


def floating_response_torque(dof):
    return [
        value * FLOATING_LOAD_COMPENSATION_SCALE
        for value in sized_joint_vector(FLOATING_RESPONSE_TORQUE, dof)
    ]


def floating_coord_tcp_start():
    return flexivrdk.Coord(
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        ["TCP", "START"],
        [0.0] * 7,
        [0.0] * 6,
    )


def rdk_mode(name):
    if hasattr(flexivrdk.Mode, name):
        return getattr(flexivrdk.Mode, name)

    try:
        return flexivrdk.Mode(MODE_VALUES[name])
    except Exception as e:
        available_modes = [
            mode_name for mode_name in dir(flexivrdk.Mode) if not mode_name.startswith("_")
        ]
        raise RuntimeError(
            f"Unable to resolve flexivrdk.Mode.{name}. "
            f"Available mode attributes: {available_modes}"
        ) from e


def wait_until_operational(robot, logger):
    logger.info("Enabling robot ...")
    robot.Enable()

    while not robot.operational():
        time.sleep(1)

    logger.info("Robot is now operational")


def initialize_robot(robot_sn, logger):
    robot = flexivrdk.Robot(robot_sn)

    if robot.fault():
        logger.warn("Fault occurred on the connected robot, trying to clear ...")
        if not robot.ClearFault():
            raise RuntimeError("Fault cannot be cleared")
        logger.info("Fault on the connected robot is cleared")

    wait_until_operational(robot, logger)
    return robot


def execute_primitive(robot, name, params):
    if hasattr(robot, "groups") and hasattr(flexivrdk, "PrimitiveArgs"):
        joint_groups = robot.groups()
        robot.ExecutePrimitive(
            {
                group: flexivrdk.PrimitiveArgs(name, params)
                for group in joint_groups
            }
        )
    else:
        robot.ExecutePrimitive(name, params)


def primitive_state_value(state, key):
    values = getattr(state, "names_and_values", state)
    if not isinstance(values, dict):
        return None
    return values.get(key)


def primitive_state_reached(robot, state_key):
    states = robot.primitive_states()

    if isinstance(states, dict) and state_key in states:
        value = states[state_key]
        if isinstance(value, (list, tuple)):
            return all(bool(v) for v in value)
        return bool(value)

    if isinstance(states, dict):
        for state in states.values():
            value = primitive_state_value(state, state_key)
            if value is None:
                continue
            if isinstance(value, (list, tuple)):
                if not all(bool(v) for v in value):
                    return False
            elif not bool(value):
                return False
        return any(primitive_state_value(state, state_key) is not None for state in states.values())

    return False


def wait_for_primitive(robot, state_key="reachedTarget", dt=DEFAULT_POLL_SEC):
    while not primitive_state_reached(robot, state_key):
        time.sleep(dt)


def any_primitive_state_reached(robot, state_keys):
    return any(primitive_state_reached(robot, state_key) for state_key in state_keys)


def primitive_state_lookup(robot, state_key):
    states = robot.primitive_states()

    if isinstance(states, dict) and state_key in states:
        return states[state_key]

    if isinstance(states, dict):
        for state in states.values():
            values = getattr(state, "names_and_values", state)
            if isinstance(values, dict) and state_key in values:
                return values[state_key]

    return None


def wait_for_primitive_any(robot, state_keys, dt=DEFAULT_POLL_SEC, timeout_sec=None):
    deadline = None if timeout_sec is None else time.monotonic() + max(0.0, timeout_sec)
    while not any_primitive_state_reached(robot, state_keys):
        if deadline is not None and time.monotonic() >= deadline:
            return False
        time.sleep(dt)
    return True


def joint_mask_for_selection(joint_indices, dof):
    if not joint_indices:
        return [1.0] * dof

    mask = [0.0] * dof
    for index in joint_indices:
        if index < 1 or index > dof:
            raise ValueError(f"Joint index {index} is outside valid range 1..{dof}")
        mask[index - 1] = 1.0
    return mask


def parse_joint_selection(tokens, dof):
    joint_tokens = []
    skip_words = {"float", "floating", "on", "joint", "joints"}
    for token in tokens:
        lower = token.lower()
        if lower in skip_words:
            continue
        if lower == "all":
            return None
        joint_tokens.extend(part for part in token.split(",") if part)

    if not joint_tokens:
        return None

    joints = []
    for token in joint_tokens:
        try:
            joints.append(int(token))
        except ValueError as e:
            raise ValueError(
                "Floating joint selection must be joint numbers like "
                "'float on 3' or 'float on 1,3,5'"
            ) from e

    # Preserve user order in logs while removing duplicates.
    unique_joints = list(dict.fromkeys(joints))
    joint_mask_for_selection(unique_joints, dof)
    return unique_joints


def split_selection_tokens(tokens):
    selections = []
    skip_words = {"float", "floating", "on", "cartesian", "axis", "axes"}
    for token in tokens:
        lower = token.lower()
        if lower in skip_words:
            continue
        selections.extend(part for part in lower.split(",") if part)
    return selections


def parse_cartesian_axis_selection(tokens):
    selections = split_selection_tokens(tokens)
    if not selections:
        return None
    has_cartesian_word = any(
        token.lower() in {"cartesian", "axis", "axes"} for token in tokens
    )

    axis_names = FLOATING_CARTESIAN_AXIS_NAMES
    groups = {
        "all": axis_names,
        "xyz": ["x", "y", "z"],
        "linear": ["x", "y", "z"],
        "translation": ["x", "y", "z"],
        "translational": ["x", "y", "z"],
        "rxryrz": ["rx", "ry", "rz"],
        "rxyz": ["rx", "ry", "rz"],
        "rot": ["rx", "ry", "rz"],
        "rotation": ["rx", "ry", "rz"],
        "rotational": ["rx", "ry", "rz"],
    }

    axes = []
    for selection in selections:
        if selection == "all" and not has_cartesian_word and len(selections) == 1:
            return None
        if selection in groups:
            axes.extend(groups[selection])
        elif selection in axis_names:
            axes.append(selection)
        elif selection.startswith("r") and selection[1:] in {"x", "y", "z"}:
            axes.append(selection)
        else:
            return None

    return list(dict.fromkeys(axes))


def cartesian_axis_mask_for_selection(axis_names):
    if not axis_names:
        return [1] * 6

    mask = [0] * 6
    for axis_name in axis_names:
        mask[FLOATING_CARTESIAN_AXIS_NAMES.index(axis_name)] = 1
    return mask


def format_cartesian_axis_selection(axis_names):
    if not axis_names:
        return "all Cartesian axes"
    return ", ".join(axis_names)


def format_joint_selection(joint_indices, dof):
    if not joint_indices:
        return "all joints"
    return ", ".join(str(index) for index in joint_indices)


def start_joint_floating(robot, logger, joint_indices=None):
    _, state = selected_robot_state(robot)
    dof = dof_from_state(robot, state)
    floating_joints = joint_mask_for_selection(joint_indices, dof)

    robot.SwitchMode(rdk_mode("NRT_PRIMITIVE_EXECUTION"))
    execute_primitive(
        robot,
        "FloatingJoint",
        {
            "floatingJoint": floating_joints,
            "dampingLevel": sized_joint_vector(FLOATING_DAMPING_LEVEL, dof),
            "responseTorque": floating_response_torque(dof),
            "diEnableFloating": "NONE",
        },
    )
    logger.info(
        "Joint floating is ON: "
        f"{format_joint_selection(joint_indices, dof)} in zero-gravity primitive"
    )


def start_cartesian_floating(robot, logger, axis_names=None, args=None):
    _, state = selected_robot_state(robot)
    dof = dof_from_state(robot, state)
    floating_axis = cartesian_axis_mask_for_selection(axis_names)

    robot.SwitchMode(rdk_mode("NRT_PRIMITIVE_EXECUTION"))
    params = {
        "floatingAxis": floating_axis,
        "enableElbowMotion": bool(getattr(args, "floating_enable_elbow_motion", False)),
        "floatingCoord": floating_coord_tcp_start(),
        "dampingLevel": FLOATING_CARTESIAN_DAMPING_LEVEL,
        "enableSixAxisJntCtrl": bool(
            getattr(args, "floating_enable_six_axis_jnt_ctrl", False)
        ),
        "diEnableFloating": "NONE",
        "responseTorque": floating_response_torque(dof),
        "inertiaScale": FLOATING_CARTESIAN_INERTIA_SCALE,
        "maxVel": float(
            getattr(args, "floating_cartesian_max_vel", DEFAULT_FLOATING_CARTESIAN_MAX_VEL)
        ),
        "frictionCompScale": sized_joint_vector(FLOATING_FRICTION_COMP_SCALE, dof),
    }
    execute_primitive(robot, "FloatingCartesian", params)
    logger.info(
        "Cartesian floating is ON: "
        f"{format_cartesian_axis_selection(axis_names)} in TCP START frame"
    )


def stop_floating(robot, logger):
    robot.Stop()
    logger.info("Floating primitive stopped")


def hold_current_joints(robot, logger):
    _, state = selected_robot_state(robot)
    dof = dof_from_state(robot, state)

    robot.SwitchMode(rdk_mode("NRT_JOINT_IMPEDANCE"))

    info = robot.info()
    if not hasattr(info, "K_q_nom"):
        raise RuntimeError("robot.info().K_q_nom is unavailable; cannot restore nominal stiffness")

    robot.SetJointImpedance(vector(info.K_q_nom))
    _, hold_state = selected_robot_state(robot)
    q = vector(hold_state.q)
    robot.SendJointPosition(q, [0.0] * dof, [HOLD_MAX_VEL] * dof, [HOLD_MAX_ACC] * dof)
    logger.info("Joint floating is OFF: restored nominal stiffness and holding current joints")


def wait_until_gripper_stopped(gripper, dt=0.1, start_timeout=1.0, stop_timeout=15.0):
    start_deadline = time.monotonic() + max(0.0, start_timeout)
    while True:
        states = gripper.states()
        if states.is_moving:
            break
        if time.monotonic() >= start_deadline:
            return True
        time.sleep(dt)

    stop_deadline = time.monotonic() + max(0.0, stop_timeout)
    while True:
        states = gripper.states()
        if not states.is_moving:
            return True
        if time.monotonic() >= stop_deadline:
            return False
        time.sleep(dt)


def setup_gripper(robot, gripper_name, logger, *, init=False):
    gripper = flexivrdk.Gripper(robot)
    tool = flexivrdk.Tool(robot)

    logger.info(f"Enabling gripper [{gripper_name}]")
    gripper.Enable(gripper_name)

    logger.info(f"Switching robot tool to [{gripper_name}]")
    tool.Switch(gripper_name)

    if init:
        logger.info("Initializing gripper ...")
        gripper.Init()
        time.sleep(INIT_WAIT_SEC)
    else:
        logger.info("Skipping gripper initialization")

    return gripper


def move_gripper(gripper, action, args, logger):
    params = gripper.params()
    open_width = clamp(args.open_width, params.min_width, params.max_width)
    close_width = clamp(args.close_width, params.min_width, params.max_width)
    velocity = clamp(args.gripper_velocity, params.min_vel, params.max_vel)
    open_force_limit = clamp(args.open_force_limit, params.min_force, params.max_force)
    close_force_limit = clamp(args.close_force_limit, params.min_force, params.max_force)

    if action == "open":
        target_width = open_width
        force_limit = open_force_limit
        label = "Opening"
    else:
        target_width = close_width
        force_limit = close_force_limit
        label = "Closing"

    logger.info(
        f"{label} gripper to {target_width:.3f} m at "
        f"{velocity:.3f} m/s, force limit {force_limit:.1f} N"
    )
    gripper.Move(target_width, velocity, force_limit)
    stopped = wait_until_gripper_stopped(gripper)

    states = gripper.states()
    if not stopped:
        raise RuntimeError(
            f"Gripper did not stop within timeout after {action}; "
            f"width {states.width:.3f} m, force {states.force:.1f} N, "
            f"moving {states.is_moving}"
        )
    logger.info(
        f"Gripper state: width {states.width:.3f} m, "
        f"force {states.force:.1f} N, moving {states.is_moving}"
    )


def parse_force_n(raw):
    normalized = raw.strip().lower().removesuffix("n").strip()
    return float(normalized)


def prompt_close_force_limit(args, gripper, logger):
    params = gripper.params()
    while True:
        raw = input(
            f"Close force limit in N (blank = {args.close_force_limit:g} N)> "
        ).strip()
        if not raw:
            return args.close_force_limit

        try:
            requested_force = parse_force_n(raw)
        except ValueError:
            logger.error("Enter a force like 20 or 20N, or press Enter for default")
            continue

        clamped_force = clamp(requested_force, params.min_force, params.max_force)
        if clamped_force != requested_force:
            logger.warn(
                f"Requested close force {requested_force:g} N is outside gripper range; "
                f"using {clamped_force:g} N"
            )
        return clamped_force


def gripper_status(gripper, gripper_name, last_action):
    states = gripper.states()
    return {
        "name": gripper_name,
        "last_command": last_action,
        "width_m": float(states.width),
        "force_n": float(states.force),
        "is_moving": bool(states.is_moving),
    }


def waypoint_record(
    name,
    trajectory_name,
    robot,
    robot_sn,
    floating_enabled,
    floating_mode,
    floating_joints,
    floating_axes,
    gripper,
    gripper_name,
    last_action,
):
    group, state = selected_robot_state(robot)
    q_rad = vector(state.q)
    tcp_pose = vector(state.tcp_pose)

    return {
        "name": name,
        "trajectory_name": trajectory_name,
        "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "robot_sn": robot_sn,
        "joint_group": group,
        "floating_enabled": bool(floating_enabled),
        "floating_mode": floating_mode if floating_enabled else None,
        "floating_joints": (
            (floating_joints or "all")
            if floating_enabled and floating_mode == "joint"
            else []
        ),
        "floating_axes": (
            (floating_axes or "all")
            if floating_enabled and floating_mode == "cartesian"
            else []
        ),
        "tcp_pose_world": {
            "unit": "m, quaternion",
            "order": ["x", "y", "z", "qw", "qx", "qy", "qz"],
            "values": tcp_pose,
        },
        "joint_angles_rad": q_rad,
        "joint_angles_deg": [math.degrees(value) for value in q_rad],
        "gripper_status": gripper_status(gripper, gripper_name, last_action),
    }


def key_position_record(name, source_trajectory_name, waypoint):
    record = dict(waypoint)
    record["record_type"] = "key_position"
    record["key_position_name"] = name
    record["source_trajectory_name"] = source_trajectory_name
    record.pop("trajectory_name", None)
    return record


def append_waypoint(path, waypoint):
    with open(path, "a", encoding="utf-8") as output:
        output.write(json.dumps(waypoint, separators=(",", ":")) + "\n")


def key_position_path(directory, name):
    return os.path.join(directory, f"{safe_filename_slug(name)}.json")


def index_path_for_key_positions(directory):
    return os.path.join(directory, KEY_POSITIONS_INDEX)


def load_key_position_index(directory):
    path = index_path_for_key_positions(directory)
    if not os.path.exists(path):
        return {"updated_at": None, "positions": {}}

    with open(path, "r", encoding="utf-8") as input_file:
        index = json.load(input_file)

    if not isinstance(index, dict):
        return {"updated_at": None, "positions": {}}
    if not isinstance(index.get("positions"), dict):
        index["positions"] = {}
    return index


def write_key_position(directory, record):
    os.makedirs(directory, exist_ok=True)
    path = key_position_path(directory, record["key_position_name"])
    with open(path, "w", encoding="utf-8") as output:
        json.dump(record, output, indent=2)
        output.write("\n")

    index = load_key_position_index(directory)
    index["updated_at"] = datetime.now().astimezone().isoformat(timespec="milliseconds")
    index["positions"][record["key_position_name"]] = {
        "file": os.path.basename(path),
        "timestamp": record["timestamp"],
        "robot_sn": record["robot_sn"],
        "joint_group": record["joint_group"],
        "source_trajectory_name": record["source_trajectory_name"],
    }
    with open(index_path_for_key_positions(directory), "w", encoding="utf-8") as output:
        json.dump(index, output, indent=2)
        output.write("\n")
    return path


def parse_key_position_command(command):
    tokens = shlex.split(command)
    if not tokens:
        return None

    if tokens[0].lower() not in {
        "key",
        "keypos",
        "key-position",
        "key_position",
        "position",
        "pos",
    }:
        return None

    if len(tokens) < 2:
        raise ValueError("Usage: key <name>, for example: key Tray")
    return " ".join(tokens[1:]).strip()


def parse_move_to_key_position_command(command):
    tokens = shlex.split(command)
    if not tokens:
        return None

    if tokens[0].lower() not in {
        "moveto",
        "move-to",
        "move_to",
        "move",
        "goto",
        "go-to",
    }:
        return None

    if len(tokens) < 2:
        raise ValueError("Usage: moveTo <key position>, for example: moveTo Tray")
    return " ".join(tokens[1:]).strip()


def load_key_position(directory, name):
    index = load_key_position_index(directory)
    entry = index.get("positions", {}).get(name)

    if entry is None:
        for indexed_name, indexed_entry in index.get("positions", {}).items():
            if indexed_name.lower() == name.lower():
                entry = indexed_entry
                break

    if entry is not None:
        path = os.path.join(directory, entry["file"])
    else:
        path = key_position_path(directory, name)

    if not os.path.exists(path):
        requested = name.strip().lower()
        candidates = []
        for filename in sorted(os.listdir(directory)):
            if not filename.lower().endswith(".json"):
                continue
            if filename == KEY_POSITIONS_INDEX:
                continue
            candidate_path = os.path.join(directory, filename)
            try:
                with open(candidate_path, "r", encoding="utf-8") as input_file:
                    candidate_record = json.load(input_file)
            except Exception:
                continue
            if not isinstance(candidate_record, dict):
                continue
            candidate_name = (
                candidate_record.get("key_position_name")
                or candidate_record.get("name")
                or os.path.splitext(filename)[0]
            )
            if str(candidate_name).strip().lower() == requested:
                candidates.append((candidate_path, candidate_record))

        if not candidates:
            raise FileNotFoundError(f"Key position [{name}] not found in {directory}")

        path, record = candidates[0]
    else:
        with open(path, "r", encoding="utf-8") as input_file:
            record = json.load(input_file)

    if not isinstance(record, dict):
        raise ValueError(f"{path}: expected a JSON object")

    pose = record.get("tcp_pose_world", {}).get("values")
    joints_deg = record.get("joint_angles_deg")
    if pose is None or len(pose) != 7:
        raise ValueError(f"{path}: expected 7 TCP pose values")
    if joints_deg is None or len(joints_deg) != 7:
        raise ValueError(f"{path}: expected 7 joint angles in degrees")

    return record, path


def moveTo_key_position(robot, key_position_name, directory, args, logger):
    record, path = load_key_position(directory, key_position_name)
    tcp_pose = [float(value) for value in record["tcp_pose_world"]["values"]]
    ref_joints_deg = [float(value) for value in record["joint_angles_deg"]]
    label = record.get("key_position_name") or record.get("name") or key_position_name

    logger.info(
        f"MoveTo key position [{label}] from {path} "
        f"at jntVelScale={args.key_move_jnt_vel_scale}"
    )
    robot.SwitchMode(rdk_mode("NRT_PRIMITIVE_EXECUTION"))
    execute_primitive(
        robot,
        "MovePTP",
        {
            "target": coord_from_tcp(tcp_pose, ref_joints_deg),
            "jntVelScale": int(args.key_move_jnt_vel_scale),
            "zoneRadius": args.key_move_zone_radius,
            "targetTolerLevel": int(args.key_move_target_toler_level),
            "jntAccMultiplier": float(args.key_move_jnt_acc_multiplier),
            "enableFixRefJntPos": True,
            "refJntPos": flexivrdk.JPos(ref_joints_deg),
        },
    )
    wait_for_primitive(robot, "reachedTarget", args.poll_sec)
    logger.info(f"Reached key position [{label}]")


def moveTo(robot, key_position_name, directory, args, logger):
    moveTo_key_position(robot, key_position_name, directory, args, logger)


def capture_injectable_rgbd(args):
    detector = load_injectable_detector_module()
    rs = require_realsense()
    import numpy as np
    from contextlib import nullcontext

    devices = detector._realsense_devices(rs)
    if not devices:
        raise RuntimeError("No Intel RealSense camera detected.")

    camera = detector._select_camera(devices, args.injectable_camera_serial)
    if camera is None:
        raise RuntimeError(
            f"No RealSense camera found with serial number: {args.injectable_camera_serial}"
        )

    print(
        "Using camera: {name} | serial: {serial} | firmware: {firmware}".format(
            **camera
        )
    )

    if str(Path(REPO_DIR)) not in sys.path:
        sys.path.insert(0, str(Path(REPO_DIR)))
    try:
        from helper.injectable_camera_session import temporarily_release_shared_camera
        release_context = temporarily_release_shared_camera(camera["serial"])
    except ImportError:
        release_context = nullcontext()

    with release_context:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(camera["serial"])
        config.enable_stream(
            rs.stream.color,
            int(args.injectable_width),
            int(args.injectable_height),
            rs.format.bgr8,
            int(args.injectable_fps),
        )
        config.enable_stream(
            rs.stream.depth,
            int(args.injectable_width),
            int(args.injectable_height),
            rs.format.z16,
            int(args.injectable_fps),
        )

        try:
            profile = pipeline.start(config)
        except RuntimeError as exc:
            raise RuntimeError(
                f"Could not start camera stream: {exc}. "
                "Try --injectable-width 640 --injectable-height 480 --injectable-fps 30"
            ) from exc

        detector._configure_color_sensor(
            rs,
            profile,
            exposure=args.injectable_exposure,
            gain=args.injectable_gain,
            white_balance=args.injectable_white_balance,
        )
        align = rs.align(rs.stream.color)

        try:
            aligned = None
            for _ in range(max(int(args.injectable_warmup_frames), 1)):
                frames = pipeline.wait_for_frames(timeout_ms=5000)
                aligned = align.process(frames)

            if aligned is None:
                raise RuntimeError("Camera did not return frames.")

            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                raise RuntimeError("Camera did not return aligned color/depth frames.")

            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale = float(depth_sensor.get_depth_scale())
            intrinsics = color_frame.profile.as_video_stream_profile().get_intrinsics()
            color_image = np.asanyarray(color_frame.get_data()).copy()
            depth_image = np.asanyarray(depth_frame.get_data()).copy()
            return color_image, depth_image, intrinsics, depth_scale
        finally:
            pipeline.stop()


def detection_center(detection):
    cx, cy = detection["rect"][0]
    return float(cx), float(cy)


def detection_angle_deg(detection):
    return float(detection["rect"][2])


def sort_injectable_detections(detections):
    return sorted(detections, key=lambda det: (detection_center(det)[1], detection_center(det)[0]))


def save_injectable_image(path, image):
    import cv2

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), image):
        raise RuntimeError(f"Could not save image: {output_path}")
    return output_path


def print_injectable_detections(detections, logger):
    logger.info(f"Detected {len(detections)} injectable(s)")
    for index, detection in enumerate(detections, start=1):
        cx, cy = detection_center(detection)
        logger.info(
            f"  #{index}: center=({cx:.1f}, {cy:.1f}) px, "
            f"angle={detection_angle_deg(detection):.1f} deg"
        )


def annotate_selected_injectable(image, detection, index):
    import cv2
    import numpy as np

    annotated = image.copy()
    box = np.asarray(detection["box"], dtype=np.int32)
    cv2.drawContours(annotated, [box], 0, (0, 255, 255), 3)
    center = tuple(int(round(value)) for value in detection_center(detection))
    cv2.drawMarker(
        annotated,
        center,
        (0, 255, 255),
        markerType=cv2.MARKER_CROSS,
        markerSize=30,
        thickness=2,
    )
    angle_rad = math.radians(detection_angle_deg(detection))
    end = (
        int(round(center[0] + 45.0 * math.cos(angle_rad))),
        int(round(center[1] + 45.0 * math.sin(angle_rad))),
    )
    cv2.arrowedLine(annotated, center, end, (0, 255, 255), 2, tipLength=0.2)
    cv2.putText(
        annotated,
        f"selected #{index}",
        (center[0] + 10, max(center[1] - 10, 20)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
    )
    return annotated


def sample_depth_m(depth_image, depth_scale, pixel_xy, radius_px=4):
    import numpy as np

    cx = int(round(pixel_xy[0]))
    cy = int(round(pixel_xy[1]))
    x0 = max(0, cx - radius_px)
    x1 = min(depth_image.shape[1], cx + radius_px + 1)
    y0 = max(0, cy - radius_px)
    y1 = min(depth_image.shape[0], cy + radius_px + 1)
    patch = depth_image[y0:y1, x0:x1]
    valid = patch[patch > 0]
    if valid.size == 0:
        raise RuntimeError(f"No valid depth near pixel ({cx}, {cy})")
    return float(np.median(valid) * depth_scale)


def transform_point(T_world_frame, point_frame):
    import numpy as np

    point = np.asarray(point_frame, dtype=float).reshape(3)
    return T_world_frame[:3, :3] @ point + T_world_frame[:3, 3]


def normalize_np(values, label):
    import numpy as np

    vector_np = np.asarray(values, dtype=float).reshape(3)
    norm = float(np.linalg.norm(vector_np))
    if norm < 1e-9:
        raise RuntimeError(f"{label} has near-zero length")
    return vector_np / norm


def build_rotation_from_xz(x_hint_world, z_axis_world, x_reference_world=None):
    import numpy as np

    z_axis = normalize_np(z_axis_world, "TCP z axis")
    x_hint = np.asarray(x_hint_world, dtype=float).reshape(3)
    x_axis = x_hint - np.dot(x_hint, z_axis) * z_axis
    x_axis = normalize_np(x_axis, "injectable axis projected into TCP plane")
    if x_reference_world is not None and np.dot(x_axis, x_reference_world) < 0.0:
        x_axis = -x_axis
    y_axis = normalize_np(np.cross(z_axis, x_axis), "TCP y axis")
    x_axis = normalize_np(np.cross(y_axis, z_axis), "TCP x axis")
    return np.column_stack([x_axis, y_axis, z_axis])


def compute_planar_injectable_target(
    detection,
    depth_image,
    depth_scale,
    intrinsics,
    current_tcp,
    T_tcp_camera,
):
    import numpy as np

    rs = require_realsense()
    handeye = load_handeye_module()

    T_world_tcp = handeye.pose_vec_to_transform(current_tcp)
    T_world_camera = T_world_tcp @ T_tcp_camera

    pink_px = tuple(float(value) for value in detection["pink_center"])
    teal_px = tuple(float(value) for value in detection["teal_center"])
    pink_depth_m = sample_depth_m(depth_image, depth_scale, pink_px)
    teal_depth_m = sample_depth_m(depth_image, depth_scale, teal_px)
    pink_camera = np.asarray(
        rs.rs2_deproject_pixel_to_point(
            intrinsics,
            [float(pink_px[0]), float(pink_px[1])],
            pink_depth_m,
        ),
        dtype=float,
    )
    teal_camera = np.asarray(
        rs.rs2_deproject_pixel_to_point(
            intrinsics,
            [float(teal_px[0]), float(teal_px[1])],
            teal_depth_m,
        ),
        dtype=float,
    )
    pink_world = transform_point(T_world_camera, pink_camera)
    teal_world = transform_point(T_world_camera, teal_camera)
    # On-axis midpoint of the two end features; immune to mask-asymmetry bias.
    center_world = 0.5 * (pink_world + teal_world)
    center_depth_m = float(0.5 * (pink_depth_m + teal_depth_m))

    current_pos = np.asarray(current_tcp[:3], dtype=float)
    current_R = np.asarray(quat_to_matrix(current_tcp[3:]), dtype=float)
    z_axis_world = normalize_np(current_R[:, 2], "current TCP z axis")
    axis_world = teal_world - pink_world

    target_R = build_rotation_from_xz(
        axis_world,
        z_axis_world,
    )
    pink_offset_xy = np.array([pink_world[0] - teal_world[0], pink_world[1] - teal_world[1], 0.0])
    pink_offset_dir = normalize_np(
        pink_offset_xy,
        "injectable pink offset direction in world XY",
    )
    tcp_y_world = target_R[:, 1]
    target_pos = np.array(
        [
            center_world[0]
            + DEFAULT_INJECTABLE_ALIGN_OFFSET_TOWARD_PINK_M * pink_offset_dir[0]
            + DEFAULT_INJECTABLE_ALIGN_OFFSET_TCP_Y_M * tcp_y_world[0],
            center_world[1]
            + DEFAULT_INJECTABLE_ALIGN_OFFSET_TOWARD_PINK_M * pink_offset_dir[1]
            + DEFAULT_INJECTABLE_ALIGN_OFFSET_TCP_Y_M * tcp_y_world[1],
            current_pos[2],
        ],
        dtype=float,
    )
    target_tcp = handeye.tcp_pose_from_pos_R(target_pos, target_R)
    return {
        "target_tcp": target_tcp,
        "center_world": center_world,
        "pink_world": pink_world,
        "teal_world": teal_world,
        "center_depth_m": center_depth_m,
        "delta_xy_mm": [
            float((target_pos[0] - current_pos[0]) * 1000.0),
            float((target_pos[1] - current_pos[1]) * 1000.0),
        ],
        "offset_toward_pink_mm": float(
            DEFAULT_INJECTABLE_ALIGN_OFFSET_TOWARD_PINK_M * 1000.0
        ),
        "offset_tcp_y_mm": float(DEFAULT_INJECTABLE_ALIGN_OFFSET_TCP_Y_M * 1000.0),
        "rotation_deg": float(handeye.rotation_angle_deg(current_R.T @ target_R)),
    }


def execute_injectable_alignment_move(robot, target_tcp, args, logger):
    _, state = selected_robot_state(robot)
    ref_joints_deg = [math.degrees(value) for value in vector(state.q)]

    robot.SwitchMode(rdk_mode("NRT_PRIMITIVE_EXECUTION"))
    if args.injectable_align_primitive == "movel":
        execute_primitive(
            robot,
            "MoveL",
            {
                "target": coord_from_tcp(target_tcp, ref_joints_deg),
                "vel": float(args.injectable_align_linear_vel),
                "zoneRadius": args.tool_move_zone_radius,
            },
        )
    else:
        execute_primitive(
            robot,
            "MovePTP",
            {
                "target": coord_from_tcp(target_tcp, ref_joints_deg),
                "jntVelScale": int(args.tool_move_jnt_vel_scale),
                "zoneRadius": args.tool_move_zone_radius,
                "targetTolerLevel": int(args.tool_move_target_toler_level),
                "jntAccMultiplier": float(args.tool_move_jnt_acc_multiplier),
                "enableFixRefJntPos": False,
                "refJntPos": flexivrdk.JPos(ref_joints_deg),
            },
        )
    wait_for_primitive(robot, "reachedTarget", args.poll_sec)
    logger.info(
        f"Injectable alignment reached target with {args.injectable_align_primitive.upper()}"
    )


def detect_injectable(robot, key_dir, args, logger):
    detector = load_injectable_detector_module()
    image = detector.capture_robot_camera(
        serial=args.injectable_camera_serial,
        width=args.injectable_width,
        height=args.injectable_height,
        fps=args.injectable_fps,
        warmup_frames=args.injectable_warmup_frames,
        exposure=args.injectable_exposure,
        gain=args.injectable_gain,
        white_balance=args.injectable_white_balance,
    )

    capture_path = save_injectable_image(args.injectable_capture_out, image)
    logger.info(f"Captured -> {capture_path}")

    result, detections = detector.detect(image)
    detections = sort_injectable_detections(detections)
    detect_path = save_injectable_image(args.injectable_detect_out, result)
    print_injectable_detections(detections, logger)
    logger.info(f"Saved -> {detect_path}")
    return {
        "capture_path": str(capture_path),
        "detect_path": str(detect_path),
        "detections": detections,
    }


def align_injectable(robot, key_dir, args, logger, target_index=1):
    detector = load_injectable_detector_module()
    handeye = load_handeye_module()
    import numpy as np

    color_image, depth_image, intrinsics, depth_scale = capture_injectable_rgbd(args)

    capture_path = save_injectable_image(args.injectable_capture_out, color_image)
    logger.info(f"Captured -> {capture_path}")

    result, detections = detector.detect(color_image)
    detections = sort_injectable_detections(detections)
    print_injectable_detections(detections, logger)
    if not detections:
        detect_path = save_injectable_image(args.injectable_align_out, result)
        raise RuntimeError(
            f"No injectable detections found. Saved annotation -> {detect_path}"
        )
    if target_index < 1 or target_index > len(detections):
        raise ValueError(
            f"Injectable index {target_index} is out of range 1..{len(detections)}"
        )

    selected = detections[target_index - 1]
    annotated = annotate_selected_injectable(result, selected, target_index)
    detect_path = save_injectable_image(args.injectable_align_out, annotated)
    logger.info(f"Saved -> {detect_path}")

    _, state = selected_robot_state(robot)
    current_tcp = vector(state.tcp_pose)
    target = compute_planar_injectable_target(
        selected,
        depth_image,
        depth_scale,
        intrinsics,
        current_tcp,
        np.asarray(
            handeye.read_yaml(Path(args.injectable_calibration))["T_tcp_camera"],
            dtype=float,
        ).reshape(4, 4),
    )
    target_tcp = target["target_tcp"]
    logger.info(
        "Planar alignment target: "
        f"dX={target['delta_xy_mm'][0]:+.1f} mm, "
        f"dY={target['delta_xy_mm'][1]:+.1f} mm, "
        f"dRot={target['rotation_deg']:.1f} deg, "
        f"center depth={target['center_depth_m'] * 1000.0:.1f} mm, "
        f"offsetTowardPink={target['offset_toward_pink_mm']:.1f} mm, "
        f"offsetTcpY={target['offset_tcp_y_mm']:.1f} mm, "
        "tcp+X->green/teal"
    )
    logger.info(
        "Target TCP [x y z qw qx qy qz]: "
        f"{[round(value, 6) for value in target_tcp]}"
    )
    execute_injectable_alignment_move(robot, target_tcp, args, logger)


def pose_record(values):
    return {
        "unit": "m, quaternion",
        "order": ["x", "y", "z", "qw", "qx", "qy", "qz"],
        "values": [float(value) for value in values],
    }


def parse_cali_tag_command(command):
    tokens = shlex.split(command)
    if not tokens:
        return None

    lower = [token.lower() for token in tokens]

    # Legacy aliases all resolve to tag 1 (Vise).
    if lower in (
        ["cali-vise"],
        ["calivise"],
        ["cali", "vise"],
        ["calibrate", "vise"],
        ["relocalize", "vise"],
    ):
        return 1

    # New form: 'cali tag N', 'cali-tag N', 'calibrate tag N'.
    head = None
    rest = []
    if lower[0] in ("cali-tag", "calitag"):
        head, rest = "cali-tag", lower[1:]
    elif len(lower) >= 2 and lower[0] in ("cali", "calibrate", "relocalize") and lower[1] == "tag":
        head, rest = "cali-tag", lower[2:]

    if head is None:
        return None

    if len(rest) != 1:
        raise ValueError(
            "Usage: cali tag <N>, where N is "
            f"one of {sorted(TAG_REGISTRY)}"
        )
    try:
        tag_id = int(rest[0])
    except ValueError as exc:
        raise ValueError(
            f"Tag id must be an integer, got: {rest[0]!r}"
        ) from exc
    if tag_id not in TAG_REGISTRY:
        raise ValueError(
            f"Unknown tag id {tag_id}. Available: {sorted(TAG_REGISTRY)}"
        )
    return tag_id


# Backward-compat shim; older callers import this name.
def parse_cali_vise_command(command):
    result = parse_cali_tag_command(command)
    if result == 1:
        return True
    return None


def load_json_file(path):
    with open(path, "r", encoding="utf-8") as input_file:
        data = json.load(input_file)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a JSON object")
    return data


def load_pose_vec_from_record(record, path, field_name):
    values = record.get(field_name, {}).get("values")
    if values is None:
        raise ValueError(f"{path}: missing {field_name}.values")
    pose = [float(value) for value in values]
    if len(pose) != 7:
        raise ValueError(f"{path}: {field_name}.values must have 7 elements")
    return pose


def load_joint_seed_from_record(record, path):
    q_rad = record.get("joint_angles_rad")
    q_deg = record.get("joint_angles_deg")
    if q_rad is None or len(q_rad) != 7:
        raise ValueError(f"{path}: expected 7 joint angles in radians")
    if q_deg is None or len(q_deg) != 7:
        raise ValueError(f"{path}: expected 7 joint angles in degrees")
    return [float(value) for value in q_rad], [float(value) for value in q_deg]


def load_reference_source_key_position(reference, reference_path):
    source = reference.get("source_key_position") or {}
    source_path = source.get("path")
    if not source_path:
        return None, None

    path = Path(source_path)
    if not path.is_absolute():
        path = Path(REPO_DIR) / path
    path = path.resolve()

    record = load_json_file(path)
    load_joint_seed_from_record(record, path)
    return record, path


def annotate_charuco_detection(
    handeye,
    cv2,
    image,
    board,
    dictionary,
    camera_matrix,
    dist_coeffs,
    pose,
    axis_length_m,
):
    annotated = image.copy()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    charuco_corners, charuco_ids, marker_corners, marker_ids = handeye.detect_charuco_corners(
        cv2,
        gray,
        board,
        dictionary,
        camera_matrix,
        dist_coeffs,
    )
    if marker_ids is not None and len(marker_ids) > 0:
        cv2.aruco.drawDetectedMarkers(annotated, marker_corners, marker_ids)
    if charuco_ids is not None and len(charuco_ids) > 0:
        cv2.aruco.drawDetectedCornersCharuco(annotated, charuco_corners, charuco_ids)
    cv2.drawFrameAxes(
        annotated,
        camera_matrix,
        dist_coeffs,
        pose.rvec,
        pose.tvec,
        float(axis_length_m),
        2,
    )
    lines = [
        f"markers={pose.marker_count}  charuco={pose.charuco_count}",
        f"reproj={pose.reprojection_error_px:.2f}px",
        "axes: X red, Y green, Z blue",
    ]
    y = 28
    for line in lines:
        cv2.putText(
            annotated,
            line,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            annotated,
            line,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (30, 30, 30),
            1,
            cv2.LINE_AA,
        )
        y += 26
    return annotated


def transform_to_tcp_pose(handeye, T):
    T = handeye.np.asarray(T, dtype=float).reshape(4, 4)
    return handeye.vector_to_list(T[:3, 3]) + handeye.vector_to_list(
        handeye.matrix_to_quat_wxyz(T[:3, :3])
    )


def constrain_vise_tcp_to_tag_axes(handeye, T_world_vise_tcp, T_world_board):
    """Align the vise TCP axes to the detected tag-1 board axes.

    Desired convention from hardware setup:
    - tag +X = TCP -Y
    - tag +Y = TCP -Z
    Therefore TCP +X = tag +Z.
    """
    constrained = handeye.np.asarray(T_world_vise_tcp, dtype=float).reshape(4, 4).copy()
    board = handeye.np.asarray(T_world_board, dtype=float).reshape(4, 4)

    board_x = board[:3, 0]
    board_z = board[:3, 2]
    x_axis = board_z / float(handeye.np.linalg.norm(board_z))
    y_hint = -board_x
    y_hint = y_hint / float(handeye.np.linalg.norm(y_hint))
    z_axis = handeye.np.cross(x_axis, y_hint)
    z_axis = z_axis / float(handeye.np.linalg.norm(z_axis))
    y_axis = handeye.np.cross(z_axis, x_axis)
    y_axis = y_axis / float(handeye.np.linalg.norm(y_axis))

    constrained[:3, :3] = handeye.np.column_stack([x_axis, y_axis, z_axis])
    return constrained


def _resolve_tag_paths(tag_id, args):
    if tag_id not in TAG_REGISTRY:
        raise ValueError(
            f"Unknown tag id {tag_id}. Available: {sorted(TAG_REGISTRY)}"
        )
    entry = TAG_REGISTRY[tag_id]
    captures_dir = os.path.join(REPO_DIR, "calibration", "captures")
    name_slug = entry["key_positions"][0]["name"].lower()
    # Tag 1 honors the legacy --cali-vise-* overrides so the recipe and any
    # external scripts continue to work; other tags use the registry defaults.
    if tag_id == 1:
        board = args.cali_vise_board
        calibration = args.cali_vise_calibration
        capture_out = args.cali_vise_capture_out
        detect_out = args.cali_vise_detect_out
        min_charuco_corners = int(args.cali_vise_min_charuco_corners)
        # Allow legacy --cali-vise-key-position / --cali-vise-reference to
        # override the single key position on tag 1.
        key_positions = [
            {
                "name": args.cali_vise_key_position,
                "reference": args.cali_vise_reference,
                "constrain_x_vertical": entry["key_positions"][0]["constrain_x_vertical"],
            },
        ]
    else:
        board = entry["board"]
        calibration = args.cali_vise_calibration
        capture_out = os.path.join(captures_dir, f"cali_tag{tag_id}_raw.png")
        detect_out = os.path.join(captures_dir, f"cali_tag{tag_id}_axes.png")
        min_charuco_corners = int(args.cali_vise_min_charuco_corners)
        key_positions = entry["key_positions"]
    return {
        "board": board,
        "calibration": calibration,
        "capture_out": capture_out,
        "detect_out": detect_out,
        "min_charuco_corners": min_charuco_corners,
        "key_positions": key_positions,
    }


def cali_tag(robot, key_dir, tag_id, args, logger):
    handeye = load_handeye_module()
    cv2 = handeye.require_cv2()
    import numpy as np

    paths = _resolve_tag_paths(tag_id, args)
    key_positions = paths["key_positions"]
    names = ", ".join(kp["name"] for kp in key_positions)
    board_cfg = handeye.load_board_config(Path(paths["board"]))
    board, dictionary = handeye.create_charuco_board(cv2, board_cfg)
    axis_length_m = 2.0 * float(board_cfg["square_length_m"])

    image, intrinsics, camera = handeye.capture_realsense_color(
        serial=args.injectable_camera_serial,
        width=int(args.injectable_width),
        height=int(args.injectable_height),
        fps=int(args.injectable_fps),
        warmup_frames=int(args.injectable_warmup_frames),
        exposure=args.injectable_exposure,
        gain=args.injectable_gain,
        white_balance=args.injectable_white_balance,
    )
    capture_path = save_injectable_image(paths["capture_out"], image)
    logger.info(
        f"Cali-tag{tag_id} ({names}) captured tag image with "
        f"{camera['name']} serial={camera['serial']} -> {capture_path}"
    )

    camera_matrix = np.asarray(intrinsics["camera_matrix"], dtype=float).reshape(3, 3)
    dist_coeffs = np.asarray(intrinsics["dist_coeffs"], dtype=float).reshape(-1, 1)
    pose = handeye.estimate_charuco_pose(
        cv2,
        image,
        board,
        dictionary,
        camera_matrix,
        dist_coeffs,
        min_charuco_corners=paths["min_charuco_corners"],
    )
    annotated = annotate_charuco_detection(
        handeye,
        cv2,
        image,
        board,
        dictionary,
        camera_matrix,
        dist_coeffs,
        pose,
        axis_length_m,
    )
    detect_path = save_injectable_image(paths["detect_out"], annotated)
    logger.info(f"Cali-tag{tag_id} ({names}) saved tag annotation -> {detect_path}")

    calibration = handeye.read_yaml(Path(paths["calibration"]))
    T_tcp_camera = np.asarray(calibration["T_tcp_camera"], dtype=float).reshape(4, 4)

    group, state = selected_robot_state(robot)
    current_tcp = vector(state.tcp_pose)
    T_world_tcp = handeye.pose_vec_to_transform(current_tcp)
    T_world_board = T_world_tcp @ T_tcp_camera @ pose.T_camera_board

    for kp in key_positions:
        kp_name = kp["name"]
        reference_path = kp["reference"]
        constrain_x_vertical = kp["constrain_x_vertical"]

        reference = load_json_file(reference_path)
        reference_source_record, reference_source_path = load_reference_source_key_position(
            reference, reference_path
        )
        transforms_dict = reference.get("transforms", {})
        matrix = (
            transforms_dict.get("T_board_keypos_tcp", {}).get("matrix")
            or transforms_dict.get("T_board_vise_tcp", {}).get("matrix")
        )
        if matrix is None:
            raise ValueError(
                f"{reference_path}: missing transforms.T_board_keypos_tcp.matrix"
            )
        T_board_keypos_tcp = np.asarray(matrix, dtype=float).reshape(4, 4)
        reference_tcp_pose = load_pose_vec_from_record(
            reference, reference_path, "reference_tcp_pose_world"
        )
        reference_T_world_keypos = handeye.pose_vec_to_transform(reference_tcp_pose)

        T_world_keypos_tcp = T_world_board @ T_board_keypos_tcp
        existing_record, existing_path = load_key_position(key_dir, kp_name)
        old_tcp = [float(value) for value in existing_record["tcp_pose_world"]["values"]]
        old_T_world_keypos = handeye.pose_vec_to_transform(old_tcp)
        if constrain_x_vertical:
            T_world_keypos_tcp = constrain_vise_tcp_to_tag_axes(
                handeye,
                T_world_keypos_tcp,
                T_world_board,
            )
        target_tcp = transform_to_tcp_pose(handeye, T_world_keypos_tcp)
        delta_mm = (T_world_keypos_tcp[:3, 3] - old_T_world_keypos[:3, 3]) * 1000.0
        delta_rot_deg = handeye.rotation_angle_deg(
            old_T_world_keypos[:3, :3].T @ T_world_keypos_tcp[:3, :3]
        )

        updated = dict(existing_record)
        updated["name"] = kp_name
        updated["timestamp"] = datetime.now().astimezone().isoformat(timespec="milliseconds")
        updated["robot_sn"] = args.robot_sn
        updated["joint_group"] = group
        updated["floating_enabled"] = False
        updated["floating_mode"] = None
        updated["floating_joints"] = []
        updated["floating_axes"] = []
        updated["tcp_pose_world"] = pose_record(target_tcp)
        if reference_source_record is not None:
            seed_rad, seed_deg = load_joint_seed_from_record(
                reference_source_record, reference_source_path
            )
            updated["joint_angles_rad"] = seed_rad
            updated["joint_angles_deg"] = seed_deg
        updated["gripper_status"] = dict(existing_record.get("gripper_status") or {})
        updated["record_type"] = "key_position"
        updated["key_position_name"] = kp_name
        updated["source_trajectory_name"] = f"cali-tag{tag_id}"
        updated["relocalized_from_tag"] = {
            "timestamp": updated["timestamp"],
            "tag_id": tag_id,
            "board_name": board_cfg["name"],
            "board_yaml": os.path.abspath(paths["board"]),
            "calibration_yaml": os.path.abspath(paths["calibration"]),
            "reference_json": os.path.abspath(reference_path),
            "capture_image": str(capture_path),
            "annotated_image": str(detect_path),
            "camera_name": camera["name"],
            "camera_serial": camera["serial"],
            "charuco_count": int(pose.charuco_count),
            "marker_count": int(pose.marker_count),
            "reprojection_error_px": float(pose.reprojection_error_px),
            "board_pose_world": {
                "matrix": handeye.matrix_to_list(T_world_board),
                "translation_m": handeye.vector_to_list(T_world_board[:3, 3]),
                "quaternion_wxyz": handeye.vector_to_list(
                    handeye.matrix_to_quat_wxyz(T_world_board[:3, :3])
                ),
            },
            "keypos_pose_world": {
                "matrix": handeye.matrix_to_list(T_world_keypos_tcp),
                "translation_m": handeye.vector_to_list(T_world_keypos_tcp[:3, 3]),
                "quaternion_wxyz": handeye.vector_to_list(
                    handeye.matrix_to_quat_wxyz(T_world_keypos_tcp[:3, :3])
                ),
            },
            "delta_from_previous_mm": [
                float(delta_mm[0]),
                float(delta_mm[1]),
                float(delta_mm[2]),
            ],
            "delta_from_previous_rot_deg": float(delta_rot_deg),
        }
        saved_path = write_key_position(key_dir, updated)
        logger.info(f"Cali-tag{tag_id} updated [{kp_name}] -> {saved_path}")
        logger.info(
            f"New {kp_name} TCP [x y z qw qx qy qz]: "
            f"{[round(value, 6) for value in target_tcp]}"
        )
        logger.info(
            f"Delta vs previous {kp_name}: "
            f"dX={delta_mm[0]:+.1f} mm, dY={delta_mm[1]:+.1f} mm, "
            f"dZ={delta_mm[2]:+.1f} mm, dRot={delta_rot_deg:.2f} deg"
        )
        if constrain_x_vertical:
            logger.info(
                f"Applied {kp_name} orientation prior: "
                f"tag +X = TCP -Y, tag +Y = TCP -Z"
            )
        if reference_source_path is not None:
            logger.info(
                f"Refreshed joint reference seed from source key position: "
                f"{reference_source_path}"
            )
        else:
            logger.info(
                f"Preserved joint reference seed from existing key position: {existing_path}"
            )


def cali_vise(robot, key_dir, args, logger):
    """Backward-compat shim: equivalent to cali_tag(1)."""
    cali_tag(robot, key_dir, 1, args, logger)


def quat_normalize(quat):
    qw, qx, qy, qz = [float(value) for value in quat]
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm == 0.0:
        raise ValueError("Quaternion norm is zero")
    return [qw / norm, qx / norm, qy / norm, qz / norm]


def quat_multiply(a, b):
    aw, ax, ay, az = quat_normalize(a)
    bw, bx, by, bz = quat_normalize(b)
    return quat_normalize(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def quat_to_matrix(quat):
    qw, qx, qy, qz = quat_normalize(quat)
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


def mat_vec_mul(matrix, vec):
    return [
        sum(matrix[row][col] * vec[col] for col in range(3))
        for row in range(3)
    ]


def vec_add(a, b):
    return [a[index] + b[index] for index in range(3)]


def vec_sub(a, b):
    return [a[index] - b[index] for index in range(3)]


def vec_cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def vec_norm(values):
    return math.sqrt(sum(value * value for value in values))


def rpy_deg_to_quat(roll_deg, pitch_deg, yaw_deg):
    r = math.radians(roll_deg) / 2.0
    p = math.radians(pitch_deg) / 2.0
    y = math.radians(yaw_deg) / 2.0
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return quat_normalize([qw, qx, qy, qz])


def quat_to_rpy_deg(qw, qx, qy, qz):
    qw, qx, qy, qz = quat_normalize([qw, qx, qy, qz])

    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def round_angle_to_step(angle_deg, step_deg):
    if step_deg <= 0.0:
        raise ValueError("Angle step must be greater than 0")
    return step_deg * math.floor((angle_deg / step_deg) + 0.5)


def orthogonal_rpy_deg(rpy_deg, step_deg=DEFAULT_ORTHO_ANGLE_STEP_DEG):
    return [round_angle_to_step(float(angle), float(step_deg)) for angle in rpy_deg]


def parse_orthogonalize_command(command):
    tokens = shlex.split(command)
    if not tokens:
        return None

    if tokens[0].lower() not in {
        "ortho",
        "orthogonal",
        "orthogonalize",
        "square",
        "square-tcp",
        "snap-ortho",
    }:
        return None

    if len(tokens) == 1:
        return DEFAULT_ORTHO_ANGLE_STEP_DEG
    if len(tokens) == 2:
        return float(tokens[1])
    raise ValueError("Usage: ortho [angle_step_deg], for example: ortho or ortho 90")


def parse_magnitude(raw, unit_suffixes):
    text = raw.strip().lower()
    for suffix in unit_suffixes:
        if text.endswith(suffix):
            text = text[: -len(suffix)].strip()
            break
    return float(text)


def parse_tool_frame_maneuver_command(command):
    tokens = shlex.split(command)
    if not tokens:
        return None

    match = re.fullmatch(r"move(r?[xyz])", tokens[0].lower())
    if not match:
        return None

    if len(tokens) == 2:
        raw_value = tokens[1]
    elif len(tokens) == 3:
        raw_value = tokens[1] + tokens[2]
    else:
        raise ValueError(
            "Usage: MoveX <cm>, MoveZ <cm>, or MoveRx <deg>, for example: MoveX 10"
        )

    axis = match.group(1)
    if axis in {"x", "y", "z"}:
        return "linear", axis, parse_magnitude(raw_value, ["cm"])
    return "rotation", axis, parse_magnitude(raw_value, ["degrees", "degree", "deg"])


def parse_twist_command(command):
    tokens = shlex.split(command)
    if not tokens:
        return None

    if tokens[0].lower() not in {
        "twist",
        "cap-twist",
        "twist-cap",
        "test-twist",
    }:
        return None

    rotation_tokens, repeat_count = parse_rotation_repeat(tokens[1:])
    if not rotation_tokens:
        return None, repeat_count
    if len(rotation_tokens) == 1:
        return parse_magnitude(
            rotation_tokens[0],
            ["degrees", "degree", "deg"],
        ), repeat_count
    raise ValueError("Usage: twist [deg] [xN], for example: twist, twist 5, twist 5 x2")


def parse_grasp_command(command):
    tokens = shlex.split(command)
    if not tokens:
        return None

    if tokens[0].lower() not in {"grasp", "graspcomp"}:
        return None

    width_m = DEFAULT_GRASP_OPEN_WIDTH
    force_n = DEFAULT_GRASP_FORCE

    for token in tokens[1:]:
        if "=" not in token:
            raise ValueError(
                "Usage: grasp [w=<open_m>] [f=<N>], for example: grasp w=0.1 f=50"
            )
        key, raw_value = token.split("=", 1)
        key = key.strip().lower()
        value = float(raw_value.strip())
        if key in {"w", "width", "gripwidth"}:
            width_m = value
        elif key in {"f", "force", "gripforce"}:
            force_n = value
        else:
            raise ValueError(
                "Usage: grasp [w=<open_m>] [f=<N>], for example: grasp w=0.1 f=50"
            )

    return width_m, force_n


def parse_detect_injectable_command(command):
    tokens = shlex.split(command)
    if not tokens:
        return None
    lower = [token.lower() for token in tokens]
    if lower not in (
        ["detect", "injectable"],
        ["injectable", "detect"],
    ):
        return None
    return True


def parse_align_injectable_command(command):
    tokens = shlex.split(command)
    if not tokens:
        return None
    lower = [token.lower() for token in tokens]
    if lower[:2] not in (
        ["align", "injectable"],
        ["injectable", "align"],
    ):
        return None
    if len(tokens) == 2:
        return 1
    if len(tokens) == 3:
        try:
            index = int(tokens[2])
        except ValueError as exc:
            raise ValueError(
                "Usage: align injectable [index], for example: align injectable 1"
            ) from exc
        if index < 1:
            raise ValueError("Injectable index must be 1 or greater")
        return index
    raise ValueError(
        "Usage: align injectable [index], for example: align injectable 1"
    )


def tool_frame_translation_target(tcp_pose, axis, distance_cm):
    pose = [float(value) for value in tcp_pose]
    axis_vectors = {
        "x": [1.0, 0.0, 0.0],
        "y": [0.0, 1.0, 0.0],
        "z": [0.0, 0.0, 1.0],
    }
    delta_tool = [value * float(distance_cm) / 100.0 for value in axis_vectors[axis]]
    delta_world = mat_vec_mul(quat_to_matrix(pose[3:]), delta_tool)
    return vec_add(pose[:3], delta_world) + quat_normalize(pose[3:])


def tool_frame_rotation_target(tcp_pose, axis, angle_deg):
    pose = [float(value) for value in tcp_pose]
    rotations = {
        "rx": (float(angle_deg), 0.0, 0.0),
        "ry": (0.0, float(angle_deg), 0.0),
        "rz": (0.0, 0.0, float(angle_deg)),
    }
    target_quat = quat_multiply(pose[3:], rpy_deg_to_quat(*rotations[axis]))
    return pose[:3] + target_quat


def execute_tool_frame_maneuver(robot, maneuver_type, axis, value, args, logger):
    _, state = selected_robot_state(robot)
    current_tcp = vector(state.tcp_pose)
    ref_joints_deg = [math.degrees(joint) for joint in vector(state.q)]

    robot.SwitchMode(rdk_mode("NRT_PRIMITIVE_EXECUTION"))
    if maneuver_type == "linear":
        target_tcp = tool_frame_translation_target(current_tcp, axis, value)
        logger.info(
            f"Tool-frame Move{axis.upper()}: {value:g} cm, "
            f"target TCP {[round(v, 6) for v in target_tcp]}"
        )
        execute_primitive(
            robot,
            "MoveL",
            {
                "target": coord_from_tcp(target_tcp, ref_joints_deg),
                "vel": float(args.tool_move_linear_vel),
                "zoneRadius": args.tool_move_zone_radius,
            },
        )
    else:
        target_tcp = tool_frame_rotation_target(current_tcp, axis, value)
        logger.info(
            f"Tool-frame Move{axis.upper()}: {value:g} deg, "
            f"target TCP {[round(v, 6) for v in target_tcp]}"
        )
        execute_primitive(
            robot,
            "MovePTP",
            {
                "target": coord_from_tcp(target_tcp, ref_joints_deg),
                "jntVelScale": int(args.tool_move_jnt_vel_scale),
                "zoneRadius": args.tool_move_zone_radius,
                "targetTolerLevel": int(args.tool_move_target_toler_level),
                "jntAccMultiplier": float(args.tool_move_jnt_acc_multiplier),
                "enableFixRefJntPos": False,
                "refJntPos": flexivrdk.JPos(ref_joints_deg),
            },
        )

    wait_for_primitive(robot, "reachedTarget", args.poll_sec)
    logger.info("Tool-frame maneuver reached target")


def execute_tcp_x_twist(robot, angle_deg, repeat_count, args, logger):
    _, state = selected_robot_state(robot)
    start_tcp = vector(state.tcp_pose)
    start_ref_joints_deg = [math.degrees(joint) for joint in vector(state.q)]
    twist_tcp = tool_frame_rotation_target(start_tcp, "rx", angle_deg)

    logger.info(
        f"Twist test: TCP X {angle_deg:+.3f} deg then back to 0, "
        f"repeat={repeat_count}"
    )
    logger.info(f"Start TCP {[round(value, 6) for value in start_tcp]}")
    logger.info(f"Twist TCP {[round(value, 6) for value in twist_tcp]}")

    robot.SwitchMode(rdk_mode("NRT_PRIMITIVE_EXECUTION"))
    for index in range(repeat_count):
        for label, target_tcp in (
            (f"twist_{index + 1}", twist_tcp),
            (f"twist_{index + 1}_return", start_tcp),
        ):
            logger.info(f"MovePTP -> {label}")
            execute_primitive(
                robot,
                "MovePTP",
                {
                    "target": coord_from_tcp(target_tcp, start_ref_joints_deg),
                    "jntVelScale": int(args.tool_move_jnt_vel_scale),
                    "zoneRadius": args.tool_move_zone_radius,
                    "targetTolerLevel": int(args.tool_move_target_toler_level),
                    "jntAccMultiplier": float(args.tool_move_jnt_acc_multiplier),
                    "enableFixRefJntPos": False,
                    "refJntPos": flexivrdk.JPos(start_ref_joints_deg),
                },
            )
            wait_for_primitive(robot, "reachedTarget", args.poll_sec)
    logger.info("Twist test complete; returned to starting TCP pose")


def tcp_axis_vector_for_world_direction(tcp_pose, world_direction):
    pose = [float(value) for value in tcp_pose]
    if len(pose) != 7:
        raise ValueError(f"tcp_pose must have 7 values, got {len(pose)}")
    desired_norm = vec_norm(world_direction)
    if desired_norm == 0.0:
        raise ValueError("world_direction must be non-zero")
    desired = [float(value) / desired_norm for value in world_direction]
    matrix = quat_to_matrix(pose[3:])
    axes = {
        "X": [matrix[0][0], matrix[1][0], matrix[2][0]],
        "Y": [matrix[0][1], matrix[1][1], matrix[2][1]],
        "Z": [matrix[0][2], matrix[1][2], matrix[2][2]],
    }
    best_axis = "X"
    best_dot = -2.0
    for axis_name, axis_vector in axes.items():
        dot = sum(axis_vector[index] * desired[index] for index in range(3))
        if abs(dot) > best_dot:
            best_axis = axis_name if dot >= 0.0 else f"-{axis_name}"
            best_dot = abs(dot)

    axis_vectors = {
        "X": [1, 0, 0],
        "-X": [-1, 0, 0],
        "Y": [0, 1, 0],
        "-Y": [0, -1, 0],
        "Z": [0, 0, 1],
        "-Z": [0, 0, -1],
    }
    active_index = next(
        index for index, value in enumerate(axis_vectors[best_axis]) if value != 0
    )
    return best_axis, axis_vectors[best_axis], active_index


def align_tcp_z_with_world_z(robot, args, logger):
    _, state = selected_robot_state(robot)
    current_tcp = vector(state.tcp_pose)
    current_rpy = quat_to_rpy_deg(*current_tcp[3:])
    current_tcp_z_world = quat_to_matrix(current_tcp[3:])[2][2]
    target_roll = 0.0 if current_tcp_z_world >= 0.0 else 180.0
    target_rpy = [target_roll, 0.0, current_rpy[2]]
    target_quat = rpy_deg_to_quat(*target_rpy)
    target_tcp = current_tcp[:3] + target_quat
    ref_joints_deg = [math.degrees(value) for value in vector(state.q)]

    logger.info(
        "Aligning TCP Z with nearest world vertical: "
        f"RPY {[round(value, 3) for value in current_rpy]} deg -> "
        f"{[round(value, 3) for value in target_rpy]} deg"
    )
    robot.SwitchMode(rdk_mode("NRT_PRIMITIVE_EXECUTION"))
    execute_primitive(
        robot,
        "MovePTP",
        {
            "target": coord_from_tcp(target_tcp, ref_joints_deg),
            "jntVelScale": int(args.ortho_jnt_vel_scale),
            "zoneRadius": args.ortho_zone_radius,
            "targetTolerLevel": int(args.ortho_target_toler_level),
            "jntAccMultiplier": float(args.ortho_jnt_acc_multiplier),
            "enableFixRefJntPos": False,
            "refJntPos": flexivrdk.JPos(ref_joints_deg),
        },
    )
    wait_for_primitive(robot, "reachedTarget", args.poll_sec)
    _, aligned_state = selected_robot_state(robot)
    aligned_tcp = vector(aligned_state.tcp_pose)
    axis_label, _, _ = tcp_axis_vector_for_world_direction(aligned_tcp, [0.0, 0.0, 1.0])
    logger.info(f"TCP vertical alignment complete: world +Z -> TCP {axis_label}")


def execute_zero_ft_sensor(robot, args, logger):
    params = {
        "dataCollectTime": float(DEFAULT_GRASP_ZEROFT_DATA_COLLECT_TIME),
        "enableStaticCheck": int(DEFAULT_GRASP_ZEROFT_ENABLE_STATIC_CHECK),
        "calibExtraPayload": int(DEFAULT_GRASP_ZEROFT_CALIB_EXTRA_PAYLOAD),
    }
    logger.info(f"ZeroFTSensor params: {params}")
    robot.SwitchMode(rdk_mode("NRT_PRIMITIVE_EXECUTION"))
    execute_primitive(robot, "ZeroFTSensor", params)
    completed = wait_for_primitive_any(
        robot,
        ("terminated",),
        args.poll_sec,
        DEFAULT_ZEROFT_TIMEOUT_SEC,
    )
    if not completed:
        robot.Stop()
        raise RuntimeError(
            f"ZeroFTSensor did not terminate within {DEFAULT_ZEROFT_TIMEOUT_SEC:.1f}s"
        )
    logger.info("ZeroFTSensor complete")


def execute_grasp_precontact_move(robot, args, logger):
    _, state = selected_robot_state(robot)
    current_tcp = vector(state.tcp_pose)
    target_tcp = list(current_tcp)
    target_tcp[2] -= float(DEFAULT_GRASP_PRECONTACT_MOVE_Z_M)
    ref_joints_deg = [math.degrees(value) for value in vector(state.q)]

    logger.info(
        "Pre-contact MoveL: "
        f"world -Z {DEFAULT_GRASP_PRECONTACT_MOVE_Z_M:.3f} m at "
        f"{DEFAULT_GRASP_PRECONTACT_MOVE_VEL:.3f} m/s"
    )
    robot.SwitchMode(rdk_mode("NRT_PRIMITIVE_EXECUTION"))
    execute_primitive(
        robot,
        "MoveL",
        {
            "target": coord_from_tcp(target_tcp, ref_joints_deg),
            "vel": float(DEFAULT_GRASP_PRECONTACT_MOVE_VEL),
            "zoneRadius": DEFAULT_TOOL_MOVE_ZONE_RADIUS,
        },
    )
    wait_for_primitive(robot, "reachedTarget", args.poll_sec)
    logger.info("Pre-contact MoveL complete; robot reached target and stopped")


def execute_contact_approach(robot, args, logger):
    params = {
        "contactCoord": "world",
        "contactDir": [0.0, 0.0, -1.0],
        "contactVel": float(DEFAULT_GRASP_CONTACT_VEL),
        "maxContactForce": float(DEFAULT_GRASP_CONTACT_FORCE),
        "enableFineContact": int(DEFAULT_GRASP_CONTACT_ENABLE_FINE_CONTACT),
    }
    logger.info(f"Contact params: {params}")
    robot.SwitchMode(rdk_mode("NRT_PRIMITIVE_EXECUTION"))
    execute_primitive(robot, "Contact", params)
    completed = wait_for_primitive_any(
        robot,
        ("terminated",),
        args.poll_sec,
        DEFAULT_CONTACT_TIMEOUT_SEC,
    )
    cur_contact_force = primitive_state_lookup(robot, "curContactForce")
    forward_dis = primitive_state_lookup(robot, "forwardDis")
    if not completed:
        robot.Stop()
        raise RuntimeError(
            "Contact did not terminate within "
            f"{DEFAULT_CONTACT_TIMEOUT_SEC:.1f}s; "
            f"curContactForce={cur_contact_force} forwardDis={forward_dis}"
        )
    logger.info(
        "Contact complete: "
        f"curContactForce={cur_contact_force} forwardDis={forward_dis}"
    )


def execute_adaptive_grasp(
    robot,
    gripper,
    width_m,
    force_n,
    args,
    logger,
    *,
    ensure_gripper_initialized=False,
):
    if gripper is None:
        raise RuntimeError("GraspComp requires an enabled gripper")

    if ensure_gripper_initialized:
        logger.info("Initializing gripper before grasp sequence ...")
        gripper.Init()
        time.sleep(INIT_WAIT_SEC)

    params = gripper.params()
    requested_open_width_m = float(width_m)
    requested_force_n = float(force_n)
    open_width_m = clamp(
        requested_open_width_m,
        max(0.0, params.min_width),
        min(0.1, params.max_width),
    )
    grip_width_m = clamp(
        float(DEFAULT_GRASP_TARGET_WIDTH),
        max(0.0, params.min_width),
        min(0.1, params.max_width),
    )
    grip_force_n = clamp(requested_force_n, -50.0, 50.0)
    requested_grip_vel_m_s = float(args.gripper_velocity)
    grip_vel_m_s = clamp(
        requested_grip_vel_m_s,
        max(float(params.min_vel), DEFAULT_GRASP_VEL),
        min(float(params.max_vel), MAX_GRASPCOMP_GRIP_VEL),
    )
    open_force_limit_n = clamp(
        float(args.open_force_limit),
        float(params.min_force),
        float(params.max_force),
    )
    contact_force_n = max(0.0, float(DEFAULT_GRASP_CONTACT_FORCE))
    max_vel_force_dir = float(DEFAULT_GRASP_MAX_VEL_FORCE_DIR)

    if open_width_m != requested_open_width_m:
        logger.warn(
            f"Requested grasp open width {requested_open_width_m:.3f} m is outside supported range; "
            f"using {open_width_m:.3f} m"
        )
    if grip_force_n != requested_force_n:
        logger.warn(
            f"Requested grasp force {requested_force_n:.1f} N is outside supported range; "
            f"using {grip_force_n:.1f} N"
        )
    if grip_vel_m_s != requested_grip_vel_m_s:
        logger.warn(
            f"Requested grasp velocity {requested_grip_vel_m_s:.3f} m/s is outside "
            f"GraspComp range; using {grip_vel_m_s:.3f} m/s"
        )

    logger.info(
        f"Opening gripper to {open_width_m:.3f} m at {grip_vel_m_s:.3f} m/s, "
        f"force limit {open_force_limit_n:.1f} N"
    )
    gripper.Move(float(open_width_m), float(grip_vel_m_s), float(open_force_limit_n))
    stopped = wait_until_gripper_stopped(gripper)
    open_states = gripper.states()
    if not stopped:
        raise RuntimeError(
            "Gripper did not stop within timeout before GraspComp; "
            f"width {open_states.width:.3f} m, force {open_states.force:.1f} N, "
            f"moving {open_states.is_moving}"
        )
    logger.info(
        f"Gripper pre-open complete: width {open_states.width:.3f} m, "
        f"force {open_states.force:.1f} N, moving {open_states.is_moving}"
    )

    align_tcp_z_with_world_z(robot, args, logger)
    execute_grasp_precontact_move(robot, args, logger)
    execute_zero_ft_sensor(robot, args, logger)
    execute_contact_approach(robot, args, logger)

    _, state = selected_robot_state(robot)
    current_tcp = vector(state.tcp_pose)
    axis_label, contact_axis, active_index = tcp_axis_vector_for_world_direction(
        current_tcp,
        [0.0, 0.0, -1.0],
    )
    contact_force = [0.0, 0.0, 0.0]
    contact_force[active_index] = contact_force_n
    comp_axis = [0, 0, 0]
    comp_force = [0.0, 0.0, 0.0]

    primitive_params = {
        "gripperType": args.gripper_name,
        "gripVel": float(grip_vel_m_s),
        "gripWidth": float(grip_width_m),
        "gripForce": float(grip_force_n),
        "contactAxis": contact_axis,
        "contactForce": contact_force,
        "compAxis": comp_axis,
        "compForce": comp_force,
        "maxVelForceDir": max_vel_force_dir,
    }

    logger.info(
        "Adaptive grasp sequence: "
        f"openWidth={open_width_m:.3f} m -> align TCP Z with world Z -> "
        f"MoveL(world -Z {DEFAULT_GRASP_PRECONTACT_MOVE_Z_M:.3f} m @ "
        f"{DEFAULT_GRASP_PRECONTACT_MOVE_VEL:.3f} m/s) -> ZeroFTSensor -> Contact(world -Z @ "
        f"{DEFAULT_GRASP_CONTACT_VEL:.3f} m/s) -> "
        f"closeTarget={grip_width_m:.3f} m gripForce={grip_force_n:.1f} N "
        f"gripVel={grip_vel_m_s:.3f} m/s contactAxis(world -Z -> TCP {axis_label}) "
        "compAxis=none"
    )
    logger.info(f"GraspComp params: {primitive_params}")

    robot.SwitchMode(rdk_mode("NRT_PRIMITIVE_EXECUTION"))
    execute_primitive(robot, "GraspComp", primitive_params)
    completed = wait_for_primitive_any(
        robot,
        ("terminated", "gripComplete", "graspComplete"),
        args.poll_sec,
        DEFAULT_GRASP_TIMEOUT_SEC,
    )

    reach_grip_force = primitive_state_lookup(robot, "reachGripForce")
    reach_grip_width = primitive_state_lookup(robot, "reachGripWidth")
    terminated = primitive_state_lookup(robot, "terminated")
    is_grip_moving = primitive_state_lookup(robot, "isGripMoving")
    cur_grip_width = primitive_state_lookup(robot, "curGripWidth")
    grip_complete = primitive_state_lookup(robot, "gripComplete")
    if grip_complete is None:
        grip_complete = primitive_state_lookup(robot, "graspComplete")
    if not completed:
        robot.Stop()
        raise RuntimeError(
            "GraspComp did not complete within "
            f"{DEFAULT_GRASP_TIMEOUT_SEC:.1f}s; "
            f"terminated={terminated} gripComplete={grip_complete} "
            f"reachGripForce={reach_grip_force} reachGripWidth={reach_grip_width} "
            f"isGripMoving={is_grip_moving} curGripWidth={cur_grip_width}"
        )
    logger.info(
        "GraspComp complete: "
        f"terminated={terminated} "
        f"reachGripForce={reach_grip_force} "
        f"reachGripWidth={reach_grip_width} "
        f"isGripMoving={is_grip_moving} "
        f"curGripWidth={cur_grip_width} "
        f"gripComplete={grip_complete}"
    )
    states = gripper.states()
    logger.info(
        f"Gripper state: width {states.width:.3f} m, "
        f"force {states.force:.1f} N, moving {states.is_moving}"
    )


def orthogonalize_tcp_orientation(robot, step_deg, args, logger):
    _, state = selected_robot_state(robot)
    current_tcp = vector(state.tcp_pose)
    current_rpy = quat_to_rpy_deg(*current_tcp[3:])
    target_rpy = orthogonal_rpy_deg(current_rpy, step_deg)
    target_quat = rpy_deg_to_quat(*target_rpy)
    target_tcp = current_tcp[:3] + target_quat
    ref_joints_deg = [math.degrees(value) for value in vector(state.q)]

    logger.info(
        "Orthogonalizing TCP orientation: "
        f"RPY {[round(value, 3) for value in current_rpy]} deg -> "
        f"{[round(value, 3) for value in target_rpy]} deg"
    )
    robot.SwitchMode(rdk_mode("NRT_PRIMITIVE_EXECUTION"))
    execute_primitive(
        robot,
        "MovePTP",
        {
            "target": coord_from_tcp(target_tcp, ref_joints_deg),
            "jntVelScale": int(args.ortho_jnt_vel_scale),
            "zoneRadius": args.ortho_zone_radius,
            "targetTolerLevel": int(args.ortho_target_toler_level),
            "jntAccMultiplier": float(args.ortho_jnt_acc_multiplier),
            "enableFixRefJntPos": False,
            "refJntPos": flexivrdk.JPos(ref_joints_deg),
        },
    )
    wait_for_primitive(robot, "reachedTarget", args.poll_sec)
    logger.info("TCP orientation orthogonalization reached target")


def rotate_about_virtual_point(tcp_pose, offset_cm, roll_deg, pitch_deg, yaw_deg):
    """Return target TCP pose after local RPY rotation about an EE-frame +X pivot."""
    pose = [float(value) for value in tcp_pose]
    if len(pose) != 7:
        raise ValueError(f"tcp_pose must have 7 values, got {len(pose)}")

    position = pose[:3]
    current_quat = quat_normalize(pose[3:])
    offset = [float(offset_cm) / 100.0, 0.0, 0.0]

    current_matrix = quat_to_matrix(current_quat)
    pivot_world = vec_add(position, mat_vec_mul(current_matrix, offset))

    delta_quat = rpy_deg_to_quat(roll_deg, pitch_deg, yaw_deg)
    target_quat = quat_multiply(current_quat, delta_quat)
    target_matrix = quat_to_matrix(target_quat)
    target_position = vec_sub(pivot_world, mat_vec_mul(target_matrix, offset))

    return target_position + target_quat


def parse_rotation_repeat(rotation_tokens):
    repeat_count = None
    if not rotation_tokens:
        return rotation_tokens, repeat_count

    last_token = rotation_tokens[-1].lower()
    repeat_match = re.fullmatch(r"x(\d+)", last_token)
    if repeat_match:
        repeat_count = int(repeat_match.group(1))
        rotation_tokens = rotation_tokens[:-1]
    elif len(rotation_tokens) >= 2 and rotation_tokens[-2].lower() in {
        "repeat",
        "repeats",
        "cycle",
        "cycles",
    }:
        repeat_count = int(rotation_tokens[-1])
        rotation_tokens = rotation_tokens[:-2]

    if repeat_count is not None and repeat_count < 1:
        raise ValueError("Repeat count must be 1 or greater")

    return rotation_tokens, repeat_count


def parse_virtual_point_rotation_command(command):
    tokens = shlex.split(command)
    if not tokens:
        return None

    first = tokens[0].lower()
    if first not in {"rotate", "rotate-vp", "rotate_about", "pivot"}:
        return None

    rest = tokens[1:]
    if rest and rest[0].lower() in {"vp", "virtual", "virtual-point", "about"}:
        rest = rest[1:]

    if not rest:
        raise ValueError(
            "Usage: rotate vp <offset_cm> <roll_deg> <pitch_deg> <yaw_deg> [xN] "
            "or rotate vp <offset_cm> <roll|pitch|yaw> <deg> [xN]"
        )

    offset_cm = float(rest[0])
    rotation_tokens, repeat_count = parse_rotation_repeat(rest[1:])
    if len(rotation_tokens) == 2 and rotation_tokens[0].lower() in {
        "roll",
        "rx",
        "x",
        "pitch",
        "ry",
        "y",
        "yaw",
        "rz",
        "z",
    }:
        axis = rotation_tokens[0].lower()
        value = float(rotation_tokens[1])
        roll_deg, pitch_deg, yaw_deg = 0.0, 0.0, 0.0
        if axis in {"roll", "rx", "x"}:
            roll_deg = value
        elif axis in {"pitch", "ry", "y"}:
            pitch_deg = value
        else:
            yaw_deg = value
        return offset_cm, roll_deg, pitch_deg, yaw_deg, repeat_count

    if len(rotation_tokens) != 3:
        raise ValueError(
            "Usage: rotate vp <offset_cm> <roll_deg> <pitch_deg> <yaw_deg> [xN] "
            "or rotate vp <offset_cm> <roll|pitch|yaw> <deg> [xN]"
        )

    roll_deg, pitch_deg, yaw_deg = [float(value) for value in rotation_tokens]
    return offset_cm, roll_deg, pitch_deg, yaw_deg, repeat_count


def virtual_point_rotation_keyframes(roll_deg, pitch_deg, yaw_deg, repeat_count):
    zero = (0.0, 0.0, 0.0)
    positive = (roll_deg, pitch_deg, yaw_deg)

    if repeat_count is None:
        return [zero, positive]

    keyframes = [zero]
    for _ in range(repeat_count):
        keyframes.append(positive)
        keyframes.append(tuple(-value for value in positive))
    keyframes.append(zero)
    return keyframes


def midpoint_rpy(start_rpy, end_rpy):
    return tuple((start_rpy[index] + end_rpy[index]) / 2.0 for index in range(3))


def coord_from_tcp(tcp_pose, ref_joints_deg):
    x, y, z, qw, qx, qy, qz = tcp_pose
    return flexivrdk.Coord(
        [x, y, z],
        quat_to_rpy_deg(qw, qx, qy, qz),
        ["WORLD", "WORLD_ORIGIN"],
        ref_joints_deg,
    )


def move_to_tcp_pose(robot, target_tcp, args, logger):
    _, state = selected_robot_state(robot)
    ref_joints_deg = [math.degrees(value) for value in vector(state.q)]

    robot.SwitchMode(rdk_mode("NRT_PRIMITIVE_EXECUTION"))
    execute_primitive(
        robot,
        "MovePTP",
        {
            "target": coord_from_tcp(target_tcp, ref_joints_deg),
            "jntVelScale": int(args.rotate_jnt_vel_scale),
            "zoneRadius": args.rotate_zone_radius,
            "targetTolerLevel": int(args.rotate_target_toler_level),
            "jntAccMultiplier": float(args.rotate_jnt_acc_multiplier),
            "enableFixRefJntPos": False,
            "refJntPos": flexivrdk.JPos(ref_joints_deg),
        },
    )
    wait_for_primitive(robot, "reachedTarget", args.poll_sec)
    logger.info("Virtual-point rotation reached target")


def movec_geometry_is_valid(start_tcp, middle_tcp, target_tcp):
    start_pos = start_tcp[:3]
    middle_pos = middle_tcp[:3]
    target_pos = target_tcp[:3]
    start_to_middle = vec_sub(middle_pos, start_pos)
    start_to_target = vec_sub(target_pos, start_pos)
    if vec_norm(start_to_middle) < 1e-5 or vec_norm(start_to_target) < 1e-5:
        return False
    return vec_norm(vec_cross(start_to_middle, start_to_target)) > 1e-8


def move_vp_arc_segment(robot, start_tcp, offset_cm, start_rpy, end_rpy, args, logger):
    middle_rpy = midpoint_rpy(start_rpy, end_rpy)
    middle_tcp = rotate_about_virtual_point(start_tcp, offset_cm, *middle_rpy)
    target_tcp = rotate_about_virtual_point(start_tcp, offset_cm, *end_rpy)

    if args.rotate_vp_primitive == "moveptp":
        move_to_tcp_pose(robot, target_tcp, args, logger)
        return

    current_tcp = vector(selected_robot_state(robot)[1].tcp_pose)
    if not movec_geometry_is_valid(current_tcp, middle_tcp, target_tcp):
        logger.warn("MoveC arc geometry is invalid; falling back to MovePTP for this leg")
        move_to_tcp_pose(robot, target_tcp, args, logger)
        return

    _, state = selected_robot_state(robot)
    ref_joints_deg = [math.degrees(value) for value in vector(state.q)]

    robot.SwitchMode(rdk_mode("NRT_PRIMITIVE_EXECUTION"))
    execute_primitive(
        robot,
        "MoveC",
        {
            "middlePose": coord_from_tcp(middle_tcp, ref_joints_deg),
            "target": coord_from_tcp(target_tcp, ref_joints_deg),
            "vel": float(args.rotate_vp_vel),
            "targetTolerLevel": int(args.rotate_target_toler_level),
            "acc": float(args.rotate_vp_acc),
            "jerk": float(args.rotate_vp_jerk),
            "equalRadius": float(args.rotate_vp_equal_radius),
        },
    )
    wait_for_primitive(robot, "reachedTarget", args.poll_sec)


def print_status(robot, floating_enabled, floating_mode, floating_joints, floating_axes, logger):
    group, state = selected_robot_state(robot)
    tcp_pose = vector(state.tcp_pose)
    q_deg = [math.degrees(value) for value in vector(state.q)]
    dof = dof_from_state(robot, state)
    if not floating_enabled:
        floating_detail = "none"
    elif floating_mode == "cartesian":
        floating_detail = format_cartesian_axis_selection(floating_axes)
    else:
        floating_detail = format_joint_selection(floating_joints, dof)

    logger.info(f"Joint group: {group if group is not None else 'default'}")
    logger.info(
        f"Floating: {'on' if floating_enabled else 'off'} "
        f"({floating_mode or 'none'}: {floating_detail})"
    )
    logger.info(f"TCP pose world [x y z qw qx qy qz]: {[round(v, 6) for v in tcp_pose]}")
    logger.info(f"Joint angles deg: {[round(v, 3) for v in q_deg]}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record named TCP poses and joint angles to a text file."
    )
    parser.add_argument(
        "--robot-sn",
        default=ROBOT_SN,
        help=f"Robot serial number, default: {ROBOT_SN}",
    )
    parser.add_argument(
        "--output",
        default=None,
        help=(
            "Output txt path. If omitted, the trajectory name becomes "
            "project/trajectory_waypoints_<name>.txt"
        ),
    )
    parser.add_argument(
        "--trajectory-name",
        default=None,
        help="Recorded trajectory name. If omitted, prompt interactively at startup.",
    )
    parser.add_argument(
        "--key-position-dir",
        default=None,
        help=f"Directory for named single-pose key positions, default: project/{DEFAULT_KEY_POSITIONS_DIR}",
    )
    parser.add_argument(
        "--key-move-jnt-vel-scale",
        type=int,
        default=DEFAULT_KEY_MOVE_JNT_VEL_SCALE,
        help=(
            "MovePTP joint velocity scale for moveTo key positions, "
            f"default: {DEFAULT_KEY_MOVE_JNT_VEL_SCALE}"
        ),
    )
    parser.add_argument(
        "--key-move-jnt-acc-multiplier",
        type=float,
        default=DEFAULT_KEY_MOVE_JNT_ACC_MULTIPLIER,
        help=(
            "MovePTP joint acceleration multiplier for moveTo key positions, "
            f"default: {DEFAULT_KEY_MOVE_JNT_ACC_MULTIPLIER}"
        ),
    )
    parser.add_argument(
        "--key-move-zone-radius",
        default=DEFAULT_KEY_MOVE_ZONE_RADIUS,
        help=(
            "MovePTP zone radius for moveTo key positions, "
            f"default: {DEFAULT_KEY_MOVE_ZONE_RADIUS}"
        ),
    )
    parser.add_argument(
        "--key-move-target-toler-level",
        type=int,
        default=DEFAULT_KEY_MOVE_TARGET_TOLER_LEVEL,
        help=(
            "MovePTP target tolerance level for moveTo key positions, "
            f"default: {DEFAULT_KEY_MOVE_TARGET_TOLER_LEVEL}"
        ),
    )
    parser.add_argument(
        "--ortho-angle-step-deg",
        type=float,
        default=DEFAULT_ORTHO_ANGLE_STEP_DEG,
        help=(
            "Default angle snapping step for ortho command in degrees, "
            f"default: {DEFAULT_ORTHO_ANGLE_STEP_DEG}"
        ),
    )
    parser.add_argument(
        "--ortho-jnt-vel-scale",
        type=int,
        default=DEFAULT_ORTHO_JNT_VEL_SCALE,
        help=(
            "MovePTP joint velocity scale for ortho adjustment, "
            f"default: {DEFAULT_ORTHO_JNT_VEL_SCALE}"
        ),
    )
    parser.add_argument(
        "--ortho-jnt-acc-multiplier",
        type=float,
        default=DEFAULT_ORTHO_JNT_ACC_MULTIPLIER,
        help=(
            "MovePTP joint acceleration multiplier for ortho adjustment, "
            f"default: {DEFAULT_ORTHO_JNT_ACC_MULTIPLIER}"
        ),
    )
    parser.add_argument(
        "--ortho-zone-radius",
        default=DEFAULT_ORTHO_ZONE_RADIUS,
        help=f"MovePTP zone radius for ortho adjustment, default: {DEFAULT_ORTHO_ZONE_RADIUS}",
    )
    parser.add_argument(
        "--ortho-target-toler-level",
        type=int,
        default=DEFAULT_ORTHO_TARGET_TOLER_LEVEL,
        help=(
            "MovePTP target tolerance level for ortho adjustment, "
            f"default: {DEFAULT_ORTHO_TARGET_TOLER_LEVEL}"
        ),
    )
    parser.add_argument(
        "--tool-move-linear-vel",
        type=float,
        default=DEFAULT_TOOL_MOVE_LINEAR_VEL,
        help=(
            "MoveL velocity for tool-frame MoveX/MoveY/MoveZ commands in m/s, "
            f"default: {DEFAULT_TOOL_MOVE_LINEAR_VEL}"
        ),
    )
    parser.add_argument(
        "--tool-move-jnt-vel-scale",
        type=int,
        default=DEFAULT_TOOL_MOVE_JNT_VEL_SCALE,
        help=(
            "MovePTP joint velocity scale for tool-frame MoveRx/MoveRy/MoveRz, "
            f"default: {DEFAULT_TOOL_MOVE_JNT_VEL_SCALE}"
        ),
    )
    parser.add_argument(
        "--tool-move-jnt-acc-multiplier",
        type=float,
        default=DEFAULT_TOOL_MOVE_JNT_ACC_MULTIPLIER,
        help=(
            "MovePTP joint acceleration multiplier for tool-frame rotations, "
            f"default: {DEFAULT_TOOL_MOVE_JNT_ACC_MULTIPLIER}"
        ),
    )
    parser.add_argument(
        "--tool-move-zone-radius",
        default=DEFAULT_TOOL_MOVE_ZONE_RADIUS,
        help=(
            "Move primitive zone radius for tool-frame maneuvers, "
            f"default: {DEFAULT_TOOL_MOVE_ZONE_RADIUS}"
        ),
    )
    parser.add_argument(
        "--tool-move-target-toler-level",
        type=int,
        default=DEFAULT_TOOL_MOVE_TARGET_TOLER_LEVEL,
        help=(
            "MovePTP target tolerance level for tool-frame rotations, "
            f"default: {DEFAULT_TOOL_MOVE_TARGET_TOLER_LEVEL}"
        ),
    )
    parser.add_argument(
        "--twist-deg",
        type=float,
        default=DEFAULT_TWIST_DEG,
        help=f"Default TCP-X twist test angle in degrees, default: {DEFAULT_TWIST_DEG}",
    )
    parser.add_argument(
        "--twist-repeat-count",
        type=int,
        default=DEFAULT_TWIST_REPEAT_COUNT,
        help=(
            "Default twist test repeat count for the interactive twist command, "
            f"default: {DEFAULT_TWIST_REPEAT_COUNT}"
        ),
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite the output file at startup instead of appending",
    )
    parser.add_argument(
        "--start-floating",
        action="store_true",
        help="Turn on joint floating as soon as the robot is ready",
    )
    parser.add_argument(
        "--floating-cartesian-max-vel",
        type=float,
        default=DEFAULT_FLOATING_CARTESIAN_MAX_VEL,
        help=(
            "FloatingCartesian maximum linear velocity in m/s, "
            f"default: {DEFAULT_FLOATING_CARTESIAN_MAX_VEL}"
        ),
    )
    parser.add_argument(
        "--floating-enable-elbow-motion",
        action="store_true",
        help="Allow elbow motion during FloatingCartesian",
    )
    parser.add_argument(
        "--floating-enable-six-axis-jnt-ctrl",
        action="store_true",
        help="Enable six-axis joint position control during FloatingCartesian",
    )
    parser.add_argument(
        "--gripper-name",
        default=GRIPPER_ID,
        help=f"Full gripper device name in Flexiv Elements, default: {GRIPPER_ID}",
    )
    parser.add_argument(
        "--init-gripper",
        action="store_true",
        help="Initialize the gripper on startup. Default is to skip init.",
    )
    parser.add_argument(
        "--open-width",
        type=float,
        default=DEFAULT_OPEN_WIDTH,
        help=f"Opening width in meters, default: {DEFAULT_OPEN_WIDTH}",
    )
    parser.add_argument(
        "--close-width",
        type=float,
        default=DEFAULT_CLOSE_WIDTH,
        help=f"Closing width in meters, default: {DEFAULT_CLOSE_WIDTH}",
    )
    parser.add_argument(
        "--gripper-velocity",
        type=float,
        default=DEFAULT_VELOCITY,
        help=f"Gripper velocity in m/s, default: {DEFAULT_VELOCITY}",
    )
    parser.add_argument(
        "--open-force-limit",
        type=float,
        default=DEFAULT_OPEN_FORCE_LIMIT,
        help=f"Open force limit in N, default: {DEFAULT_OPEN_FORCE_LIMIT}",
    )
    parser.add_argument(
        "--close-force-limit",
        type=float,
        default=DEFAULT_CLOSE_FORCE_LIMIT,
        help=f"Close force limit in N, default: {DEFAULT_CLOSE_FORCE_LIMIT}",
    )
    parser.add_argument(
        "--rotate-jnt-vel-scale",
        type=int,
        default=DEFAULT_ROTATE_JNT_VEL_SCALE,
        help=(
            "MovePTP joint velocity scale for virtual-point rotations, "
            f"default: {DEFAULT_ROTATE_JNT_VEL_SCALE}"
        ),
    )
    parser.add_argument(
        "--rotate-zone-radius",
        default=DEFAULT_ROTATE_ZONE_RADIUS,
        help=(
            "MovePTP zone radius for virtual-point rotations, "
            f"default: {DEFAULT_ROTATE_ZONE_RADIUS}"
        ),
    )
    parser.add_argument(
        "--rotate-target-toler-level",
        type=int,
        default=DEFAULT_ROTATE_TARGET_TOLER_LEVEL,
        help=(
            "MovePTP target tolerance level for virtual-point rotations, "
            f"default: {DEFAULT_ROTATE_TARGET_TOLER_LEVEL}"
        ),
    )
    parser.add_argument(
        "--rotate-jnt-acc-multiplier",
        type=float,
        default=DEFAULT_ROTATE_JNT_ACC_MULTIPLIER,
        help=(
            "MovePTP joint acceleration multiplier for virtual-point rotations, "
            f"default: {DEFAULT_ROTATE_JNT_ACC_MULTIPLIER}"
        ),
    )
    parser.add_argument(
        "--rotate-vp-primitive",
        choices=["movec", "moveptp"],
        default=DEFAULT_ROTATE_VP_PRIMITIVE,
        help=(
            "Primitive for virtual-point rotations: movec preserves the circular TCP arc, "
            "moveptp hops endpoint-to-endpoint, default: movec"
        ),
    )
    parser.add_argument(
        "--rotate-vp-vel",
        type=float,
        default=DEFAULT_ROTATE_VP_VEL,
        help=f"MoveC velocity for virtual-point rotation arcs in m/s, default: {DEFAULT_ROTATE_VP_VEL}",
    )
    parser.add_argument(
        "--rotate-vp-acc",
        type=float,
        default=DEFAULT_ROTATE_VP_ACC,
        help=f"MoveC acceleration for virtual-point rotation arcs in m/s^2, default: {DEFAULT_ROTATE_VP_ACC}",
    )
    parser.add_argument(
        "--rotate-vp-jerk",
        type=float,
        default=DEFAULT_ROTATE_VP_JERK,
        help=f"MoveC jerk for virtual-point rotation arcs in m/s^3, default: {DEFAULT_ROTATE_VP_JERK}",
    )
    parser.add_argument(
        "--rotate-vp-equal-radius",
        type=float,
        default=DEFAULT_ROTATE_VP_EQUAL_RADIUS,
        help=(
            "MoveC equalRadius parameter for coupling orientation to path length, "
            f"default: {DEFAULT_ROTATE_VP_EQUAL_RADIUS}"
        ),
    )
    parser.add_argument(
        "--injectable-key-position",
        default=DEFAULT_INJECTABLE_KEY_POSITION,
        help=(
            "Named reference pose operators typically use before injectable detect/align "
            "(not auto-moved by these commands), "
            f"default: {DEFAULT_INJECTABLE_KEY_POSITION}"
        ),
    )
    parser.add_argument(
        "--injectable-calibration",
        default=DEFAULT_INJECTABLE_CALIBRATION,
        help=(
            "Path to eye-in-hand calibration YAML used for planar injectable alignment, "
            f"default: {DEFAULT_INJECTABLE_CALIBRATION}"
        ),
    )
    parser.add_argument(
        "--injectable-camera-serial",
        default=None,
        help="Specific RealSense serial number to use for injectable detect/align",
    )
    parser.add_argument(
        "--injectable-width",
        type=int,
        default=640,
        help="Injectable camera stream width, default: 640",
    )
    parser.add_argument(
        "--injectable-height",
        type=int,
        default=480,
        help="Injectable camera stream height, default: 480",
    )
    parser.add_argument(
        "--injectable-fps",
        type=int,
        default=30,
        help="Injectable camera stream frame rate, default: 30",
    )
    parser.add_argument(
        "--injectable-warmup-frames",
        type=int,
        default=30,
        help="Frames to discard before injectable capture, default: 30",
    )
    parser.add_argument(
        "--injectable-capture-out",
        default=DEFAULT_INJECTABLE_CAPTURE_OUT,
        help=f"Path to save the raw injectable capture, default: {DEFAULT_INJECTABLE_CAPTURE_OUT}",
    )
    parser.add_argument(
        "--injectable-detect-out",
        default=DEFAULT_INJECTABLE_DETECT_OUT,
        help=f"Path to save the annotated injectable detection image, default: {DEFAULT_INJECTABLE_DETECT_OUT}",
    )
    parser.add_argument(
        "--injectable-align-out",
        default=DEFAULT_INJECTABLE_ALIGN_OUT,
        help=f"Path to save the selected injectable alignment image, default: {DEFAULT_INJECTABLE_ALIGN_OUT}",
    )
    parser.add_argument(
        "--injectable-exposure",
        type=float,
        default=20000,
        help="Manual color exposure for injectable capture, default: 20000",
    )
    parser.add_argument(
        "--injectable-gain",
        type=float,
        default=None,
        help="Manual color gain for injectable capture",
    )
    parser.add_argument(
        "--injectable-white-balance",
        type=float,
        default=None,
        help="Manual white balance in Kelvin for injectable capture",
    )
    parser.add_argument(
        "--injectable-align-primitive",
        choices=["movel", "moveptp"],
        default=DEFAULT_INJECTABLE_ALIGN_PRIMITIVE,
        help=(
            "Primitive for planar injectable alignment, "
            f"default: {DEFAULT_INJECTABLE_ALIGN_PRIMITIVE}"
        ),
    )
    parser.add_argument(
        "--injectable-align-linear-vel",
        type=float,
        default=DEFAULT_INJECTABLE_ALIGN_LINEAR_VEL,
        help=(
            "MoveL velocity in m/s for planar injectable alignment, "
            f"default: {DEFAULT_INJECTABLE_ALIGN_LINEAR_VEL}"
        ),
    )
    parser.add_argument(
        "--cali-vise-key-position",
        default=DEFAULT_CALI_VISE_KEY_POSITION,
        help=(
            "Key position name to update from the ChArUco tag relocalization, "
            f"default: {DEFAULT_CALI_VISE_KEY_POSITION}"
        ),
    )
    parser.add_argument(
        "--cali-vise-board",
        default=DEFAULT_CALI_VISE_BOARD,
        help=f"Path to the ChArUco board YAML for cali-vise, default: {DEFAULT_CALI_VISE_BOARD}",
    )
    parser.add_argument(
        "--cali-vise-calibration",
        default=DEFAULT_CALI_VISE_CALIBRATION,
        help=(
            "Path to eye-in-hand calibration YAML for cali-vise, "
            f"default: {DEFAULT_CALI_VISE_CALIBRATION}"
        ),
    )
    parser.add_argument(
        "--cali-vise-reference",
        default=DEFAULT_CALI_VISE_REFERENCE,
        help=(
            "Path to the saved board-to-vise reference JSON for cali-vise, "
            f"default: {DEFAULT_CALI_VISE_REFERENCE}"
        ),
    )
    parser.add_argument(
        "--cali-vise-capture-out",
        default=DEFAULT_CALI_VISE_CAPTURE_OUT,
        help=f"Path to save the raw cali-vise capture, default: {DEFAULT_CALI_VISE_CAPTURE_OUT}",
    )
    parser.add_argument(
        "--cali-vise-detect-out",
        default=DEFAULT_CALI_VISE_DETECT_OUT,
        help=f"Path to save the annotated cali-vise image, default: {DEFAULT_CALI_VISE_DETECT_OUT}",
    )
    parser.add_argument(
        "--cali-vise-min-charuco-corners",
        type=int,
        default=DEFAULT_CALI_VISE_MIN_CHARUCO_CORNERS,
        help=(
            "Minimum ChArUco corners required for cali-vise pose estimation, "
            f"default: {DEFAULT_CALI_VISE_MIN_CHARUCO_CORNERS}"
        ),
    )
    parser.add_argument(
        "--poll-sec",
        type=float,
        default=DEFAULT_POLL_SEC,
        help=f"Primitive completion polling interval in seconds, default: {DEFAULT_POLL_SEC}",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    logger = spdlog.ConsoleLogger("WaypointRecorder")
    trajectory_name = prompt_trajectory_name(args)
    output_path = output_path_for_trajectory(args, trajectory_name)
    key_dir = key_positions_dir(args)
    robot = None
    gripper = None
    floating_active = False
    floating_mode = None
    floating_joints = None
    floating_axes = None
    last_gripper_action = "initialized"

    try:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        os.makedirs(key_dir, exist_ok=True)
        if args.overwrite:
            open(output_path, "w", encoding="utf-8").close()

        robot = initialize_robot(args.robot_sn, logger)
        gripper = setup_gripper(
            robot,
            args.gripper_name,
            logger,
            init=args.init_gripper,
        )
        gripper_initialized = bool(args.init_gripper)
        logger.info(f"Trajectory name: {trajectory_name}")
        logger.info(f"Recording waypoints to: {output_path}")
        logger.info(f"Recording key positions to: {key_dir}")

        if args.start_floating:
            start_joint_floating(robot, logger, floating_joints)
            floating_active = True
            floating_mode = "joint"

        print(
            "\nCommands:\n"
            "  Type an entry name and press Enter to record the current pose.\n"
            "  key <name>       - save current pose as a named key position.\n"
            "                     Examples: key Tray, key Intermediate, key Vise\n"
            "  moveTo <name>    - move to a saved key position.\n"
            "                     Examples: moveTo Tray, moveTo Vise\n"
            "  ortho [deg]      - keep TCP XYZ and snap TCP rx ry rz to nearest 90 deg.\n"
            "                     Examples: ortho, ortho 90\n"
            "  MoveX/Y/Z <cm>   - translate in the current TCP/tool frame.\n"
            "                     Examples: MoveX 10, MoveZ -20\n"
            "  MoveRx/Ry/Rz <deg> - rotate in the current TCP/tool frame.\n"
            "                     Example: MoveRx 10\n"
            "  twist [deg] [xN] - recipe-style TCP-X twist, then return to start.\n"
            "                     Examples: twist, twist 5, twist 5 x2\n"
            "  grasp [w=<m>] [f=<N>] - open gripper, align TCP Z, ZeroFTSensor,\n"
            "                     Contact in world -Z, then GraspComp.\n"
            "                     Example: grasp w=0.1 f=50\n"
            "  float on [joints] - joint floating for all joints or selected joints.\n"
            "                      Examples: float on, float on 7, float on 1,3,5\n"
            "  float <axis>      - Cartesian floating along one or more TCP START axes.\n"
            "                      Examples: float x, float Rx, float x Rz\n"
            "                      Use float cartesian all for all 6 Cartesian axes.\n"
            "  float off - disable floating and hold the current joints.\n"
            "  rotate vp <x_cm> <roll> <pitch> <yaw> [xN]\n"
            "            rotate about a virtual point x_cm along EE +X, using local RPY degrees.\n"
            "            Examples: rotate vp 5 0 0 30, rotate vp 5 yaw 30 x5\n"
            "  detect injectable - capture a still image at the current pose and\n"
            "                     save injectable detections.\n"
            "  align injectable [N] - capture RGB-D at the current pose, then align\n"
            "                     XY + wrist rotation to injectable #N while keeping Z fixed.\n"
            "  cali tag <N>     - capture the ChArUco tag N, localize it, and update each\n"
            "                     attached key position from its tag_NN_to_<name>_tcp.json.\n"
            "                     Tags: 1=Vise, 2=Plate, 3=Spring+Plastic+Glass.\n"
            "                     (cali-vise = cali tag 1.)\n"
            "  gripper open  - open the gripper.\n"
            "  gripper close - close the gripper.\n"
            "  status    - print current TCP pose and joint angles.\n"
            "  quit      - stop recording.\n",
            flush=True,
        )

        while True:
            command = input("Entry name or command> ").strip()
            command_lower = command.lower()

            if not command:
                continue

            if command_lower in {"q", "quit", "exit"}:
                break

            try:
                key_position_name = parse_key_position_command(command)
            except ValueError as e:
                logger.error(str(e))
                continue
            if key_position_name is not None:
                waypoint = waypoint_record(
                    key_position_name,
                    trajectory_name,
                    robot,
                    args.robot_sn,
                    floating_active,
                    floating_mode,
                    floating_joints,
                    floating_axes,
                    gripper,
                    args.gripper_name,
                    last_gripper_action,
                )
                record = key_position_record(key_position_name, trajectory_name, waypoint)
                saved_path = write_key_position(key_dir, record)
                logger.info(f"Recorded key position [{key_position_name}] -> {saved_path}")
                continue

            try:
                move_to_key_position_name = parse_move_to_key_position_command(command)
            except ValueError as e:
                logger.error(str(e))
                continue
            if move_to_key_position_name is not None:
                if floating_active:
                    stop_floating(robot, logger)
                    floating_active = False
                    floating_mode = None
                    floating_joints = None
                    floating_axes = None
                moveTo(robot, move_to_key_position_name, key_dir, args, logger)
                continue

            try:
                ortho_step_deg = parse_orthogonalize_command(command)
            except ValueError as e:
                logger.error(str(e))
                continue
            if ortho_step_deg is not None:
                if floating_active:
                    stop_floating(robot, logger)
                    floating_active = False
                    floating_mode = None
                    floating_joints = None
                    floating_axes = None
                if command_lower in {"ortho", "orthogonal", "orthogonalize", "square"}:
                    ortho_step_deg = args.ortho_angle_step_deg
                orthogonalize_tcp_orientation(robot, ortho_step_deg, args, logger)
                continue

            try:
                tool_maneuver = parse_tool_frame_maneuver_command(command)
            except ValueError as e:
                logger.error(str(e))
                continue
            if tool_maneuver is not None:
                if floating_active:
                    stop_floating(robot, logger)
                    floating_active = False
                    floating_mode = None
                    floating_joints = None
                    floating_axes = None
                    hold_current_joints(robot, logger)
                maneuver_type, axis, value = tool_maneuver
                execute_tool_frame_maneuver(
                    robot, maneuver_type, axis, value, args, logger
                )
                continue

            try:
                twist_command = parse_twist_command(command)
            except ValueError as e:
                logger.error(str(e))
                continue
            if twist_command is not None:
                if floating_active:
                    stop_floating(robot, logger)
                    floating_active = False
                    floating_mode = None
                    floating_joints = None
                    floating_axes = None
                    hold_current_joints(robot, logger)

                twist_deg, repeat_count = twist_command
                if twist_deg is None:
                    twist_deg = args.twist_deg
                if repeat_count is None:
                    repeat_count = args.twist_repeat_count
                if repeat_count < 1:
                    logger.error("Twist repeat count must be 1 or greater")
                    continue
                execute_tcp_x_twist(
                    robot,
                    float(twist_deg),
                    int(repeat_count),
                    args,
                    logger,
                )
                continue

            try:
                detect_injectable_command = parse_detect_injectable_command(command)
            except ValueError as e:
                logger.error(str(e))
                continue
            if detect_injectable_command is not None:
                if floating_active:
                    stop_floating(robot, logger)
                    floating_active = False
                    floating_mode = None
                    floating_joints = None
                    floating_axes = None
                    hold_current_joints(robot, logger)
                detect_injectable(robot, key_dir, args, logger)
                continue

            try:
                align_injectable_index = parse_align_injectable_command(command)
            except ValueError as e:
                logger.error(str(e))
                continue
            if align_injectable_index is not None:
                if floating_active:
                    stop_floating(robot, logger)
                    floating_active = False
                    floating_mode = None
                    floating_joints = None
                    floating_axes = None
                    hold_current_joints(robot, logger)
                align_injectable(
                    robot,
                    key_dir,
                    args,
                    logger,
                    target_index=align_injectable_index,
                )
                continue

            try:
                cali_tag_id = parse_cali_tag_command(command)
            except ValueError as e:
                logger.error(str(e))
                continue
            if cali_tag_id is not None:
                if floating_active:
                    stop_floating(robot, logger)
                    floating_active = False
                    floating_mode = None
                    floating_joints = None
                    floating_axes = None
                    hold_current_joints(robot, logger)
                cali_tag(robot, key_dir, cali_tag_id, args, logger)
                continue

            try:
                grasp_command = parse_grasp_command(command)
            except ValueError as e:
                logger.error(str(e))
                continue
            if grasp_command is not None:
                if floating_active:
                    stop_floating(robot, logger)
                    floating_active = False
                    floating_mode = None
                    floating_joints = None
                    floating_axes = None
                    hold_current_joints(robot, logger)

                width_m, force_n = grasp_command
                execute_adaptive_grasp(
                    robot,
                    gripper,
                    width_m,
                    force_n,
                    args,
                    logger,
                    ensure_gripper_initialized=not gripper_initialized,
                )
                gripper_initialized = True
                last_gripper_action = f"grasp(w={width_m:.3f},f={force_n:.1f})"
                continue

            if command_lower in {"float off", "floating off", "off"}:
                if floating_active:
                    stop_floating(robot, logger)
                    floating_active = False
                    floating_mode = None
                    floating_joints = None
                    floating_axes = None
                hold_current_joints(robot, logger)
                continue

            if (
                command_lower in {"float on", "floating on", "on"}
                or command_lower.startswith("float ")
                or command_lower.startswith("floating ")
            ):
                tokens = shlex.split(command)
                lower_tokens = [token.lower() for token in tokens]
                if "off" in lower_tokens:
                    if floating_active:
                        stop_floating(robot, logger)
                        floating_active = False
                        floating_mode = None
                        floating_joints = None
                        floating_axes = None
                    hold_current_joints(robot, logger)
                    continue

                floating_axes = parse_cartesian_axis_selection(tokens)
                if floating_axes is not None:
                    if floating_active:
                        stop_floating(robot, logger)
                    start_cartesian_floating(robot, logger, floating_axes, args)
                    floating_active = True
                    floating_mode = "cartesian"
                    floating_joints = None
                    continue

                _, state = selected_robot_state(robot)
                dof = dof_from_state(robot, state)
                try:
                    floating_joints = parse_joint_selection(tokens, dof)
                except ValueError as e:
                    logger.error(str(e))
                    continue
                if floating_active:
                    stop_floating(robot, logger)
                start_joint_floating(robot, logger, floating_joints)
                floating_active = True
                floating_mode = "joint"
                floating_axes = None
                continue

            try:
                virtual_point_rotation = parse_virtual_point_rotation_command(command)
            except ValueError as e:
                logger.error(str(e))
                continue
            if virtual_point_rotation is not None:
                if floating_active:
                    stop_floating(robot, logger)
                    floating_active = False
                    floating_mode = None
                    floating_joints = None
                    floating_axes = None

                offset_cm, roll_deg, pitch_deg, yaw_deg, repeat_count = virtual_point_rotation
                _, state = selected_robot_state(robot)
                start_tcp = vector(state.tcp_pose)
                keyframes = virtual_point_rotation_keyframes(
                    roll_deg, pitch_deg, yaw_deg, repeat_count
                )
                logger.info(
                    "Virtual-point rotation: "
                    f"pivot EE +X {offset_cm:.3f} cm, "
                    f"local RPY delta [{roll_deg:.3f}, {pitch_deg:.3f}, {yaw_deg:.3f}] deg"
                )
                logger.info(
                    f"Executing {len(keyframes) - 1} virtual-point arc legs "
                    f"with {args.rotate_vp_primitive.upper()}"
                )
                if repeat_count is not None:
                    logger.info(
                        f"Oscillating +delta/-delta for {repeat_count} cycles, "
                        "then returning to the starting pose"
                    )

                for step_index, (start_rpy, end_rpy) in enumerate(
                    zip(keyframes, keyframes[1:]), 1
                ):
                    logger.info(
                        f"Arc {step_index}/{len(keyframes) - 1} local RPY "
                        f"{[round(value, 3) for value in start_rpy]} -> "
                        f"{[round(value, 3) for value in end_rpy]} deg"
                    )
                    move_vp_arc_segment(
                        robot,
                        start_tcp,
                        offset_cm,
                        start_rpy,
                        end_rpy,
                        args,
                        logger,
                    )
                continue

            if command_lower in {"gripper open", "open gripper", "g open"}:
                move_gripper(gripper, "open", args, logger)
                last_gripper_action = "open"
                continue

            if command_lower in {"gripper close", "close gripper", "g close"}:
                args.close_force_limit = prompt_close_force_limit(args, gripper, logger)
                move_gripper(gripper, "close", args, logger)
                last_gripper_action = "close"
                continue

            if command_lower in {"status", "s"}:
                print_status(
                    robot,
                    floating_active,
                    floating_mode,
                    floating_joints,
                    floating_axes,
                    logger,
                )
                if gripper is not None:
                    states = gripper.states()
                    logger.info(
                        f"Gripper: width {states.width:.3f} m, "
                        f"force {states.force:.1f} N, moving {states.is_moving}"
                    )
                continue

            waypoint = waypoint_record(
                command,
                trajectory_name,
                robot,
                args.robot_sn,
                floating_active,
                floating_mode,
                floating_joints,
                floating_axes,
                gripper,
                args.gripper_name,
                last_gripper_action,
            )
            append_waypoint(output_path, waypoint)
            logger.info(f"Recorded [{command}]")

        logger.info("Done")
        return 0

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
        return 0
    except Exception as e:
        logger.error(str(e))
        return 1
    finally:
        if robot is not None:
            try:
                if floating_active:
                    stop_floating(robot, logger)
                    hold_current_joints(robot, logger)
                robot.Stop()
            except Exception as e:
                logger.error(f"Shutdown error: {e}")


if __name__ == "__main__":
    raise SystemExit(main())
