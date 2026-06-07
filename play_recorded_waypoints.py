#!/usr/bin/env python

"""Replay recorded waypoints with slow MovePTP primitive motions."""

import argparse
import glob
import json
import math
import os
import time

import flexivrdk
import spdlog


ROBOT_SN = "Rizon4-062930"
GRIPPER_ID = "Flexiv-GN01"
DEFAULT_TRAJECTORY = "trajectory_waypoints.txt"
DEFAULT_TRAJECTORY_1 = "trajectory_waypoints_1.txt"
DEFAULT_TRAJECTORY_2 = "trajectory_waypoints_2.txt"
TRAJECTORY_GLOB = "trajectory_waypoints*.txt"
DEFAULT_JNT_VEL_SCALE = 5
DEFAULT_JNT_ACC_MULTIPLIER = 1.0
DEFAULT_ZONE_RADIUS = "ZFine"
DEFAULT_TARGET_TOLER_LEVEL = 1
DEFAULT_OPEN_WIDTH = 0.04
DEFAULT_CLOSE_WIDTH = 0.0
DEFAULT_GRIPPER_VELOCITY = 0.1
DEFAULT_OPEN_FORCE_LIMIT = 10.0
DEFAULT_CLOSE_FORCE_LIMIT = 40.0
GRIPPER_INIT_WAIT_SEC = 4.0
MODE_VALUES = {
    "NRT_PRIMITIVE_EXECUTION": 8,
}


def project_path(filename):
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)


def clamp(value, low, high):
    return max(low, min(value, high))


def positive_int(value):
    parsed = int(value)
    if parsed < 1:
        raise argparse.ArgumentTypeError("must be 1 or greater")
    return parsed


def nonnegative_float(value):
    parsed = float(value)
    if parsed < 0.0:
        raise argparse.ArgumentTypeError("must be 0 or greater")
    return parsed


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


def load_waypoints(path):
    waypoints = []
    with open(path, "r", encoding="utf-8") as input_file:
        for line_number, line in enumerate(input_file, 1):
            line = line.strip()
            if not line:
                continue

            waypoint = json.loads(line)
            pose = waypoint.get("tcp_pose_world", {}).get("values")
            joints_deg = waypoint.get("joint_angles_deg")
            if pose is None or len(pose) != 7:
                raise ValueError(f"Line {line_number}: expected 7 TCP pose values")
            if joints_deg is None or len(joints_deg) != 7:
                raise ValueError(f"Line {line_number}: expected 7 joint angles in degrees")
            waypoints.append(waypoint)

    if not waypoints:
        raise ValueError(f"No waypoints found in {path}")

    return waypoints


def quat_to_rpy_deg(qw, qx, qy, qz):
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm

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


def waypoint_coord(waypoint):
    pose = waypoint["tcp_pose_world"]["values"]
    x, y, z, qw, qx, qy, qz = [float(value) for value in pose]
    orientation = quat_to_rpy_deg(qw, qx, qy, qz)
    ref_joints = [float(value) for value in waypoint["joint_angles_deg"]]

    return flexivrdk.Coord(
        [x, y, z],
        orientation,
        ["WORLD", "WORLD_ORIGIN"],
        ref_joints,
    )


def primitive_state_value(state, key):
    values = getattr(state, "names_and_values", state)
    return values[key]


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
            if isinstance(value, (list, tuple)):
                if not all(bool(v) for v in value):
                    return False
            elif not bool(value):
                return False
        return bool(states)

    return False


def wait_for_primitive(robot, state_key="reachedTarget", dt=0.2):
    while not primitive_state_reached(robot, state_key):
        time.sleep(dt)


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


def initialize_robot(robot_sn, logger):
    robot = flexivrdk.Robot(robot_sn)

    if robot.fault():
        logger.warn("Fault occurred on the connected robot, trying to clear ...")
        if not robot.ClearFault():
            raise RuntimeError("Fault cannot be cleared")
        logger.info("Fault on the connected robot is cleared")

    logger.info("Enabling robot ...")
    robot.Enable()
    while not robot.operational():
        time.sleep(1)
    logger.info("Robot is now operational")

    return robot


def setup_gripper(robot, gripper_name, init_gripper, logger):
    gripper = flexivrdk.Gripper(robot)
    tool = flexivrdk.Tool(robot)

    logger.info(f"Enabling gripper [{gripper_name}]")
    gripper.Enable(gripper_name)

    logger.info(f"Switching robot tool to [{gripper_name}]")
    tool.Switch(gripper_name)

    if init_gripper:
        logger.info("Initializing gripper ...")
        gripper.Init()
        time.sleep(GRIPPER_INIT_WAIT_SEC)

    return gripper


def wait_until_gripper_stopped(gripper, dt=0.1):
    time.sleep(dt)
    while gripper.states().is_moving:
        time.sleep(dt)


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
    else:
        target_width = close_width
        force_limit = close_force_limit

    logger.info(
        f"Gripper {action}: width {target_width:.3f} m, "
        f"velocity {velocity:.3f} m/s, force limit {force_limit:.1f} N"
    )
    gripper.Move(target_width, velocity, force_limit)
    wait_until_gripper_stopped(gripper)


def replay_gripper_if_needed(gripper, waypoint, last_action, args, logger):
    status = waypoint.get("gripper_status") or {}
    action = status.get("last_command")

    if action not in {"open", "close"} or action == last_action:
        return last_action

    move_gripper(gripper, action, args, logger)
    return action


def replay_waypoint(robot, waypoint, index, total, segment_index, args, logger):
    logger.info(
        f"Trajectory {segment_index} MovePTP {index}/{total}: "
        f"[{waypoint.get('name', 'unnamed')}] "
        f"at jntVelScale={args.jnt_vel_scale}"
    )
    execute_primitive(
        robot,
        "MovePTP",
        {
            "target": waypoint_coord(waypoint),
            "jntVelScale": int(args.jnt_vel_scale),
            "zoneRadius": args.zone_radius,
            "targetTolerLevel": int(args.target_toler_level),
            "jntAccMultiplier": float(args.jnt_acc_multiplier),
            "enableFixRefJntPos": True,
            "refJntPos": flexivrdk.JPos(
                [float(value) for value in waypoint["joint_angles_deg"]]
            ),
        },
    )
    wait_for_primitive(robot, "reachedTarget", args.poll_sec)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Replay recorded waypoint JSONL using slow MovePTP primitives."
    )
    parser.add_argument(
        "--trajectory",
        "-t",
        action="append",
        help=(
            "Recorded waypoint txt file or selector. Can be passed multiple times, "
            "or comma-separated. Selectors: latest, all, or a number from "
            "--list-trajectories. Default: latest trajectory_waypoints*.txt"
        ),
    )
    parser.add_argument(
        "--list-trajectories",
        action="store_true",
        help="List selectable project/trajectory_waypoints*.txt files and exit",
    )
    parser.add_argument(
        "--repeat",
        type=positive_int,
        default=1,
        help="Number of times to play the selected trajectory set, default: 1",
    )
    parser.add_argument(
        "--repeat-pause-sec",
        type=nonnegative_float,
        default=0.0,
        help="Pause between repeated playback passes, default: 0",
    )
    parser.add_argument(
        "--auto-continue",
        action="store_true",
        help="Continue through multiple selected trajectories without prompting",
    )
    parser.add_argument(
        "--robot-sn",
        default=ROBOT_SN,
        help=f"Robot serial number, default: {ROBOT_SN}",
    )
    parser.add_argument(
        "--jnt-vel-scale",
        type=int,
        default=DEFAULT_JNT_VEL_SCALE,
        choices=range(1, 101),
        metavar="[1-100]",
        help=f"MovePTP joint velocity scale, default: {DEFAULT_JNT_VEL_SCALE}",
    )
    parser.add_argument(
        "--jnt-acc-multiplier",
        type=float,
        default=DEFAULT_JNT_ACC_MULTIPLIER,
        help=f"MovePTP joint acceleration multiplier, default: {DEFAULT_JNT_ACC_MULTIPLIER}",
    )
    parser.add_argument(
        "--zone-radius",
        default=DEFAULT_ZONE_RADIUS,
        help=f"MovePTP blending zone radius, default: {DEFAULT_ZONE_RADIUS}",
    )
    parser.add_argument(
        "--target-toler-level",
        type=int,
        default=DEFAULT_TARGET_TOLER_LEVEL,
        help=f"MovePTP target tolerance level, default: {DEFAULT_TARGET_TOLER_LEVEL}",
    )
    parser.add_argument(
        "--pause-sec",
        type=nonnegative_float,
        default=0.5,
        help="Pause after each reached waypoint, default: 0.5",
    )
    parser.add_argument(
        "--poll-sec",
        type=nonnegative_float,
        default=0.2,
        help="Primitive state polling interval, default: 0.2",
    )
    parser.add_argument(
        "--skip-gripper",
        action="store_true",
        help="Do not replay recorded gripper open/close state changes",
    )
    parser.add_argument(
        "--init-gripper",
        action="store_true",
        help="Initialize the gripper before replaying gripper actions",
    )
    parser.add_argument(
        "--gripper-name",
        default=GRIPPER_ID,
        help=f"Full gripper device name in Flexiv Elements, default: {GRIPPER_ID}",
    )
    parser.add_argument("--open-width", type=float, default=DEFAULT_OPEN_WIDTH)
    parser.add_argument("--close-width", type=float, default=DEFAULT_CLOSE_WIDTH)
    parser.add_argument("--gripper-velocity", type=float, default=DEFAULT_GRIPPER_VELOCITY)
    parser.add_argument("--open-force-limit", type=float, default=DEFAULT_OPEN_FORCE_LIMIT)
    parser.add_argument("--close-force-limit", type=float, default=DEFAULT_CLOSE_FORCE_LIMIT)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Parse and print the replay plan without connecting to the robot",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip the startup confirmation prompt",
    )
    return parser.parse_args()


def discover_trajectory_paths():
    paths = glob.glob(project_path(TRAJECTORY_GLOB))
    return sorted(
        (os.path.abspath(path) for path in paths),
        key=lambda path: (os.path.getmtime(path), path),
        reverse=True,
    )


def trajectory_tokens(selections):
    for selection in selections or []:
        for token in selection.split(","):
            token = token.strip()
            if token:
                yield token


def resolve_trajectory_path(token):
    if os.path.isabs(token):
        return os.path.abspath(token)

    cwd_path = os.path.abspath(token)
    project_relative_path = os.path.abspath(project_path(token))

    if os.path.exists(cwd_path):
        return cwd_path
    if os.path.exists(project_relative_path):
        return project_relative_path
    if os.path.dirname(token):
        return cwd_path
    return project_relative_path


def trajectory_paths(args):
    discovered = discover_trajectory_paths()

    if args.trajectory:
        paths = []
        for token in trajectory_tokens(args.trajectory):
            selector = token.lower()
            if selector in {"latest", "newest", "most-recent", "recent"}:
                if not discovered:
                    raise ValueError(
                        f"No project/{TRAJECTORY_GLOB} files found for selector {token}"
                    )
                paths.append(discovered[0])
            elif selector == "all":
                if not discovered:
                    raise ValueError(f"No project/{TRAJECTORY_GLOB} files found")
                paths.extend(discovered)
            elif token.isdigit():
                choice_index = int(token)
                if choice_index < 1 or choice_index > len(discovered):
                    raise ValueError(
                        f"Trajectory selection {choice_index} is out of range; "
                        "run with --list-trajectories"
                    )
                paths.append(discovered[choice_index - 1])
            else:
                paths.append(resolve_trajectory_path(token))

        return paths

    if discovered:
        return [discovered[0]]

    return [os.path.abspath(project_path(DEFAULT_TRAJECTORY))]


def print_available_trajectories(paths, logger):
    if not paths:
        logger.info(f"No project/{TRAJECTORY_GLOB} files found")
        return

    logger.info("Selectable trajectories, newest first:")
    for index, path in enumerate(paths, 1):
        modified = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(os.path.getmtime(path)))
        logger.info(f"  {index}: {path} modified={modified}")


def print_plan(segments, args, logger):
    for segment_index, (path, waypoints) in enumerate(segments, 1):
        logger.info(f"Trajectory {segment_index}: {len(waypoints)} waypoints from {path}")
        for waypoint_index, waypoint in enumerate(waypoints, 1):
            gripper_action = (waypoint.get("gripper_status") or {}).get("last_command")
            logger.info(
                f"  {segment_index}.{waypoint_index}: "
                f"{waypoint.get('name', 'unnamed')} gripper={gripper_action}"
            )

    if args.repeat > 1:
        logger.info(
            f"Will play selected trajectory set {args.repeat} times "
            f"with {args.repeat_pause_sec:.3f} s between passes"
        )


def wait_for_continue(segment_index, total_segments, args, logger):
    if segment_index >= total_segments:
        return True

    if args.auto_continue or args.yes:
        return True

    answer = input(
        f"Trajectory {segment_index} complete. Type CONTINUE to play trajectory "
        f"{segment_index + 1}> "
    ).strip()

    if answer != "CONTINUE":
        logger.info("Playback stopped before next trajectory")
        return False

    return True


def replay_segment(
    robot,
    gripper,
    waypoints,
    segment_index,
    total_segments,
    pass_index,
    total_passes,
    last_gripper_action,
    args,
    logger,
):
    if total_passes > 1:
        logger.info(
            f"Starting pass {pass_index}/{total_passes}, "
            f"trajectory {segment_index}/{total_segments}"
        )
    else:
        logger.info(f"Starting trajectory {segment_index}/{total_segments}")

    for waypoint_index, waypoint in enumerate(waypoints, 1):
        replay_waypoint(
            robot,
            waypoint,
            waypoint_index,
            len(waypoints),
            segment_index,
            args,
            logger,
        )

        if gripper is not None:
            last_gripper_action = replay_gripper_if_needed(
                gripper, waypoint, last_gripper_action, args, logger
            )

        time.sleep(args.pause_sec)

    logger.info(f"Trajectory {segment_index} complete")
    return last_gripper_action


def main():
    args = parse_args()
    logger = spdlog.ConsoleLogger("WaypointPlayback")
    robot = None

    try:
        if args.list_trajectories:
            print_available_trajectories(discover_trajectory_paths(), logger)
            return 0

        paths = trajectory_paths(args)
        segments = [(path, load_waypoints(path)) for path in paths]
        total_waypoints = sum(len(waypoints) for _, waypoints in segments)
        logger.info(
            f"Loaded {total_waypoints} waypoints across {len(segments)} trajectory file(s)"
        )
        if args.repeat > 1:
            logger.info(
                f"Total commanded waypoint visits: {total_waypoints * args.repeat}"
            )
        logger.warn(
            f"Replay will move the robot with MovePTP at slow jntVelScale={args.jnt_vel_scale}. "
            "Keep E-stop within reach."
        )

        print_plan(segments, args, logger)

        if args.dry_run:
            logger.info("Dry run complete")
            return 0

        if not args.yes:
            answer = input("Type PLAY to start robot motion> ").strip()
            if answer != "PLAY":
                logger.info("Playback cancelled")
                return 0

        robot = initialize_robot(args.robot_sn, logger)

        gripper = None
        last_gripper_action = None
        if not args.skip_gripper:
            gripper = setup_gripper(robot, args.gripper_name, args.init_gripper, logger)
            first_action = (segments[0][1][0].get("gripper_status") or {}).get("last_command")
            if first_action in {"open", "close"}:
                move_gripper(gripper, first_action, args, logger)
                last_gripper_action = first_action

        robot.SwitchMode(rdk_mode("NRT_PRIMITIVE_EXECUTION"))

        stop_playback = False
        for pass_index in range(1, args.repeat + 1):
            for segment_index, (_, waypoints) in enumerate(segments, 1):
                last_gripper_action = replay_segment(
                    robot,
                    gripper,
                    waypoints,
                    segment_index,
                    len(segments),
                    pass_index,
                    args.repeat,
                    last_gripper_action,
                    args,
                    logger,
                )
                if not wait_for_continue(segment_index, len(segments), args, logger):
                    stop_playback = True
                    break

            if stop_playback:
                break

            if pass_index < args.repeat and args.repeat_pause_sec > 0.0:
                logger.info(
                    f"Waiting {args.repeat_pause_sec:.3f} s before playback pass "
                    f"{pass_index + 1}/{args.repeat}"
                )
                time.sleep(args.repeat_pause_sec)

        robot.Stop()
        logger.info("Playback complete")
        return 0

    except KeyboardInterrupt:
        logger.warn("Interrupted by user, stopping robot")
        if robot is not None:
            robot.Stop()
        return 1
    except Exception as e:
        logger.error(str(e))
        if robot is not None:
            robot.Stop()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
