#!/usr/bin/env python

"""Open or close a Flexiv GN01 gripper."""

import argparse
import time

import flexivrdk
import spdlog


ROBOT_SN = "Rizon4-062930"
GRIPPER_ID = "Flexiv-GN01"
INIT_WAIT_SEC = 4.0
DEFAULT_OPEN_WIDTH = 0.04
DEFAULT_CLOSE_WIDTH = 0.0
DEFAULT_VELOCITY = 0.1
DEFAULT_OPEN_FORCE_LIMIT = 10.0
DEFAULT_CLOSE_FORCE_LIMIT = 40.0


def wait_until_stopped(gripper, dt=0.1):
    time.sleep(dt)
    while gripper.states().is_moving:
        time.sleep(dt)


def clamp(value, low, high):
    return max(low, min(value, high))


def main():
    parser = argparse.ArgumentParser()
    action_group = parser.add_mutually_exclusive_group(required=True)
    action_group.add_argument(
        "-open",
        dest="action",
        action="store_const",
        const="open",
        help="Open the gripper",
    )
    action_group.add_argument(
        "-close",
        dest="action",
        action="store_const",
        const="close",
        help="Close the gripper",
    )
    parser.add_argument(
        "--gripper-name",
        default=GRIPPER_ID,
        help=f"Full gripper device name in Flexiv Elements, default: {GRIPPER_ID}",
    )
    parser.add_argument(
        "--hold-sec",
        type=float,
        default=1.0,
        help="Seconds to pause after each command, default: 1",
    )
    parser.add_argument(
        "--init",
        action="store_true",
        help="Manually initialize the gripper before moving",
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
    args = parser.parse_args()

    logger = spdlog.ConsoleLogger("GripperOpenClose")

    try:
        robot = flexivrdk.Robot(ROBOT_SN)

        if robot.fault():
            logger.warn("Fault occurred on the connected robot, trying to clear ...")
            if not robot.ClearFault():
                logger.error("Fault cannot be cleared, exiting ...")
                return 1
            logger.info("Fault on the connected robot is cleared")

        logger.info("Enabling robot ...")
        robot.Enable()
        while not robot.operational():
            time.sleep(1)
        logger.info("Robot is now operational")

        gripper = flexivrdk.Gripper(robot)
        tool = flexivrdk.Tool(robot)

        logger.info(f"Enabling gripper [{args.gripper_name}]")
        gripper.Enable(args.gripper_name)

        logger.info(f"Switching robot tool to [{args.gripper_name}]")
        tool.Switch(args.gripper_name)

        if args.init:
            logger.info("Initializing gripper ...")
            gripper.Init()
            time.sleep(INIT_WAIT_SEC)

        params = gripper.params()
        open_width = clamp(args.open_width, params.min_width, params.max_width)
        close_width = clamp(args.close_width, params.min_width, params.max_width)
        velocity = clamp(DEFAULT_VELOCITY, params.min_vel, params.max_vel)
        open_force_limit = clamp(DEFAULT_OPEN_FORCE_LIMIT, params.min_force, params.max_force)
        close_force_limit = clamp(DEFAULT_CLOSE_FORCE_LIMIT, params.min_force, params.max_force)
        initial_states = gripper.states()

        logger.info(
            f"Gripper limits: width [{params.min_width:.3f}, {params.max_width:.3f}] m, "
            f"force [{params.min_force:.1f}, {params.max_force:.1f}] N, "
            f"velocity [{params.min_vel:.3f}, {params.max_vel:.3f}] m/s"
        )
        logger.info(
            f"Current gripper state: width {initial_states.width:.3f} m, "
            f"force {initial_states.force:.1f} N, moving {initial_states.is_moving}"
        )
        if args.action == "open" and open_width != args.open_width:
            logger.warn(
                f"Requested open width {args.open_width:.3f} m was clamped to "
                f"{open_width:.3f} m"
            )
        if args.action == "close" and close_width != args.close_width:
            logger.warn(
                f"Requested close width {args.close_width:.3f} m was clamped to "
                f"{close_width:.3f} m"
            )

        if args.action == "open":
            target_width = open_width
            force_limit = open_force_limit
            action_label = "Opening"
        else:
            target_width = close_width
            force_limit = close_force_limit
            action_label = "Closing"
        logger.info(
            f"{action_label} gripper to {target_width:.3f} m at "
            f"{velocity:.3f} m/s, force limit {force_limit:.1f} N"
        )
        gripper.Move(target_width, velocity, force_limit)
        wait_until_stopped(gripper)
        final_states = gripper.states()
        logger.info(
            f"Final gripper state: width {final_states.width:.3f} m, "
            f"force {final_states.force:.1f} N, moving {final_states.is_moving}"
        )
        time.sleep(args.hold_sec)

        logger.info("Done")
        return 0

    except Exception as e:
        logger.error(str(e))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
