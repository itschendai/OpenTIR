"""Capture a single named pose into pipeline_poses.yaml.

Use when you need to add a one-off pose that isn't in the trainer's standard
checklist (e.g. ``cut_support_grip`` for the supported-cut experiment).

The arm enters FloatingJoint mode — drag it to the target pose by hand, then
press Enter. Joint angles, TCP pose, and gripper state are written into
``pipeline_poses.yaml`` under the name you pass on the command line.
"""

from __future__ import annotations

import argparse
import os
import sys

import pose_schema
from flexiv_helpers import RobotSession
from train_pipeline_poses import _start_floating, gripper_snapshot


DEFAULT_ROBOT_SN = "Rizon4-062930"
DEFAULT_POSE_FILE = "pipeline_poses.yaml"
DEFAULT_GRIPPER_NAME = "Flexiv-GN01"


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(prog="capture_pose")
    p.add_argument("name", help="Pose name to write under doc['poses'][name]")
    p.add_argument("--pose-file", default=DEFAULT_POSE_FILE)
    p.add_argument("--robot-sn", default=DEFAULT_ROBOT_SN)
    p.add_argument("--gripper-name", default=DEFAULT_GRIPPER_NAME)
    p.add_argument(
        "--init-gripper",
        action="store_true",
        help="Run the GN01 init sequence (open + calibrate ~4s). Skip if "
        "the gripper was already initialized this session.",
    )
    p.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite if a pose with this name already exists.",
    )
    return p.parse_args(argv)


def _resolve(path: str) -> str:
    if os.path.isabs(path):
        return path
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(here, path))


def main(argv=None) -> int:
    args = parse_args(argv)
    pose_file = _resolve(args.pose_file)

    try:
        doc = pose_schema.read_yaml(pose_file)
    except FileNotFoundError:
        doc = pose_schema.TrainerState.fresh(args.robot_sn).document

    poses = doc.setdefault("poses", {})
    if args.name in poses and not args.overwrite:
        print(
            f"pose {args.name!r} already exists; pass --overwrite to replace",
            file=sys.stderr,
        )
        return 2

    try:
        import spdlog  # type: ignore

        logger = spdlog.ConsoleLogger("CapturePose")
    except Exception:
        logger = None

    with RobotSession(args.robot_sn, logger=logger) as session:
        gripper = session.setup_gripper(args.gripper_name, init=args.init_gripper)
        _start_floating(session, logger=logger)
        input(
            f"\nDrag the arm to the {args.name!r} pose, then press Enter "
            f"to capture ... "
        )
        _, state = session.selected_arm_state()
        q_rad = [float(v) for v in getattr(state, "q", [])]
        tcp = [float(v) for v in getattr(state, "tcp_pose", [])]
        if len(q_rad) != 7 or len(tcp) != 7:
            print(
                f"unexpected arm state: q has {len(q_rad)}, tcp has {len(tcp)}; "
                f"expected 7 each",
                file=sys.stderr,
            )
            return 1
        width, force = gripper_snapshot(gripper)
        entry = pose_schema.build_pose_entry(q_rad, tcp, width, force)
        poses[args.name] = entry
        pose_schema.write_yaml(doc, pose_file)
        print(f"\nSaved pose {args.name!r} to {pose_file}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
