"""Standalone visual sanity replay for captured pipeline poses.

Walks the entries returned by :func:`pose_schema.phase_entries` and issues a
slow ``MovePTP`` for each pose / path waypoint. Joint-space motion is enforced
by using the captured TCP pose as the target *and* the captured joint angles
as the IK seed (``enableFixRefJntPos=True``), which guarantees the arm reaches
the exact joint configuration the operator demonstrated regardless of small
TCP-frame differences (e.g. gripper vs flange).

This is **not** an M5 implementation. There is no Cartesian impedance, no
gripper close, no force feedback, no vise interaction. It is the cheap
"do the captured poses look sane on hardware" check before investing in the
real pickup phase. The first run should use the default ``--jnt-vel-scale 5``
(half the orchestrator default) with the operator's hand on the E-stop.
"""

from __future__ import annotations

import argparse
import os
import sys

import pose_schema
from flexiv_helpers import (
    RobotSession,
    joints_to_jpos_deg,
    tcp_pose_to_coord_args,
)


DEFAULT_ROBOT_SN = "Rizon4-062930"
DEFAULT_POSE_FILE = "pipeline_poses.yaml"
DEFAULT_JNT_VEL_SCALE = 5
DEFAULT_ZONE_RADIUS = "ZFine"
DEFAULT_TARGET_TOLER_LEVEL = 1
DEFAULT_JNT_ACC_MULTIPLIER = 1.0


def _resolve_pose_file(path: str) -> str:
    if os.path.isabs(path):
        return path
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(here, path))


def _movePTP_params(
    entry: dict, flexivrdk_module, jnt_vel_scale: int, zone_radius: str,
    target_toler_level: int, jnt_acc_multiplier: float,
) -> dict:
    """Build the MovePTP params dict from a captured pose / waypoint entry."""
    tcp = entry["tcp_pose_world"]["values"]
    joints_deg = joints_to_jpos_deg(entry["q_rad"])
    coord_args = tcp_pose_to_coord_args(tcp, ref_joints_deg=joints_deg)
    return {
        "target": flexivrdk_module.Coord(*coord_args),
        "jntVelScale": int(jnt_vel_scale),
        "zoneRadius": zone_radius,
        "targetTolerLevel": int(target_toler_level),
        "jntAccMultiplier": float(jnt_acc_multiplier),
        "enableFixRefJntPos": True,
        "refJntPos": flexivrdk_module.JPos(joints_deg),
    }


def _move_to(session: RobotSession, entry: dict, label: str, args, logger) -> None:
    params = _movePTP_params(
        entry,
        session.flexivrdk,
        jnt_vel_scale=args.jnt_vel_scale,
        zone_radius=args.zone_radius,
        target_toler_level=args.target_toler_level,
        jnt_acc_multiplier=args.jnt_acc_multiplier,
    )
    if logger is not None:
        logger.info(f"MovePTP -> {label}")
    session.execute_primitive("MovePTP", params)
    session.wait_for_primitive("reachedTarget")


def _validate_entries_present(doc: dict, entries: list[tuple[str, str]]) -> list[str]:
    missing: list[str] = []
    poses = doc.get("poses", {}) or {}
    paths = doc.get("paths", {}) or {}
    for kind, name in entries:
        if kind == "pose" and name not in poses:
            missing.append(f"pose {name!r}")
        elif kind == "path" and name not in paths:
            missing.append(f"path {name!r}")
    return missing


def run_replay(args, logger=None, session_factory=None) -> int:
    """Execute the replay. ``session_factory`` lets tests inject a fake session."""
    pose_file = _resolve_pose_file(args.pose_file)
    if not os.path.exists(pose_file):
        msg = f"pose file not found: {pose_file}"
        if logger is not None:
            getattr(logger, "error", logger.info)(msg)
        print(msg, file=sys.stderr)
        return 2

    doc = pose_schema.read_yaml(pose_file)
    try:
        entries = pose_schema.phase_entries(args.phase)
    except KeyError as exc:
        print(f"unknown phase: {exc}", file=sys.stderr)
        return 2

    missing = _validate_entries_present(doc, entries)
    if missing:
        msg = (
            f"pose file is missing entries for phase {args.phase!r}: "
            f"{', '.join(missing)}. Re-run the trainer with --phase "
            f"{args.phase} --resume to capture them."
        )
        if logger is not None:
            getattr(logger, "error", logger.info)(msg)
        print(msg, file=sys.stderr)
        return 1

    pose_count = sum(1 for k, _ in entries if k == "pose")
    path_count = sum(1 for k, _ in entries if k == "path")
    if logger is not None:
        logger.info(
            f"Replay phase={args.phase!r} from {pose_file}: "
            f"{pose_count} poses, {path_count} paths "
            f"(jntVelScale={args.jnt_vel_scale})"
        )

    if session_factory is None:
        session = RobotSession(args.robot_sn, logger=logger)
    else:
        session = session_factory()

    with session:
        session.switch_mode("NRT_PRIMITIVE_EXECUTION")
        for kind, name in entries:
            if kind == "pose":
                _move_to(session, doc["poses"][name], f"pose [{name}]", args, logger)
            elif kind == "path":
                path = doc["paths"][name]
                waypoints = path.get("waypoints", []) or []
                if not waypoints:
                    if logger is not None:
                        logger.info(f"path [{name}] has 0 waypoints; skipping")
                    continue
                for i, wp in enumerate(waypoints, start=1):
                    label = (
                        f"path [{name}] wp {i}/{len(waypoints)} "
                        f"({wp.get('name', '?')})"
                    )
                    _move_to(session, wp, label, args, logger)
    if logger is not None:
        logger.info("Replay complete")
    return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="replay_phase",
        description=(
            "Slow MovePTP replay through captured pipeline poses, joint-space "
            "via enableFixRefJntPos. No compliance, no gripper, no Arduino — "
            "visual sanity check only."
        ),
    )
    parser.add_argument("--robot-sn", default=DEFAULT_ROBOT_SN)
    parser.add_argument("--pose-file", default=DEFAULT_POSE_FILE)
    parser.add_argument(
        "--phase",
        choices=pose_schema.PHASE_NAMES,
        default="pickup",
        help="Phase subset to replay (default 'pickup' = tray -> above_vise)",
    )
    parser.add_argument(
        "--jnt-vel-scale",
        type=int,
        default=DEFAULT_JNT_VEL_SCALE,
        help=(
            f"MovePTP joint velocity scale 1-100 (lower=slower), "
            f"default {DEFAULT_JNT_VEL_SCALE} = half the orchestrator default"
        ),
    )
    parser.add_argument("--zone-radius", default=DEFAULT_ZONE_RADIUS)
    parser.add_argument(
        "--target-toler-level", type=int, default=DEFAULT_TARGET_TOLER_LEVEL
    )
    parser.add_argument(
        "--jnt-acc-multiplier", type=float, default=DEFAULT_JNT_ACC_MULTIPLIER
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    try:
        import spdlog  # type: ignore

        logger = spdlog.ConsoleLogger("ReplayPhase")
    except Exception:
        logger = None
    args = parse_args(argv)
    return run_replay(args, logger=logger)


if __name__ == "__main__":
    raise SystemExit(main())
