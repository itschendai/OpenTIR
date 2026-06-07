"""Surgically edit a single pose or path waypoint in pipeline_poses.yaml.

Use cases:
- "The yaw is 88.7 deg, set it to exactly 90 without touching anything else."
- "Lower this pose by 5 mm in Z; keep x, y, orientation."
- "Lock the wrist to vertical-down (roll=180, pitch=0) but keep the yaw I had."
- "Copy the wrist orientation from `above_vise` to `disposal`; positions stay."
- "Snap the captured RPY to the nearest 1 degree so the numbers read cleanly."

By default this is a *paper edit*: it rewrites ``tcp_pose_world`` (position +
quaternion) and leaves ``q_rad`` / ``q_deg`` alone. The two will then disagree
until the orchestrator's IK re-solves at playback. Pass ``--use-robot`` to
have the script also send a MovePTP to the new pose and recapture both
``q_rad`` and ``tcp_pose_world`` from the resulting arm state — that keeps
the YAML self-consistent at the cost of moving the arm.

Until you pass ``--apply`` the script only prints before/after; nothing is
written to disk.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import pose_schema
from flexiv_helpers import quat_to_rpy_deg, rpy_deg_to_quat


DEFAULT_POSE_FILE = "pipeline_poses.yaml"
DEFAULT_ROBOT_SN = "Rizon4-062930"
DEFAULT_JNT_VEL_SCALE = 5


# ---------- pose lookup ----------


def _resolve_pose_file(path: str) -> str:
    if os.path.isabs(path):
        return path
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(here, path))


def _get_entry(doc: dict, args) -> tuple[dict, str]:
    """Return (entry_dict, human_label). Exits with rc=2 on bad reference."""
    if args.pose:
        poses = doc.get("poses", {}) or {}
        if args.pose not in poses:
            print(
                f"pose {args.pose!r} not found. Known: {sorted(poses)}",
                file=sys.stderr,
            )
            raise SystemExit(2)
        return poses[args.pose], f"pose [{args.pose}]"
    if args.path is not None:
        paths = doc.get("paths", {}) or {}
        if args.path not in paths:
            print(
                f"path {args.path!r} not found. Known: {sorted(paths)}",
                file=sys.stderr,
            )
            raise SystemExit(2)
        wps = paths[args.path].get("waypoints", []) or []
        if args.waypoint is None:
            print(
                f"path {args.path!r} has {len(wps)} waypoint(s); "
                f"specify --waypoint <N> (1-indexed)",
                file=sys.stderr,
            )
            raise SystemExit(2)
        idx = args.waypoint - 1
        if idx < 0 or idx >= len(wps):
            print(
                f"path {args.path!r} has {len(wps)} waypoint(s); "
                f"--waypoint {args.waypoint} is out of range",
                file=sys.stderr,
            )
            raise SystemExit(2)
        return wps[idx], (
            f"path [{args.path}] wp {args.waypoint}/{len(wps)} "
            f"({wps[idx].get('name', '?')})"
        )
    print("must specify --pose <name> or --path <name> --waypoint <N>", file=sys.stderr)
    raise SystemExit(2)


# ---------- display ----------


def _format_entry(entry: dict) -> list[str]:
    q_rad = list(entry.get("q_rad", [])) or [0.0] * 7
    q_deg = list(entry.get("q_deg") or [math.degrees(v) for v in q_rad])
    tcp = list(entry["tcp_pose_world"]["values"])
    rpy = quat_to_rpy_deg(tcp[3], tcp[4], tcp[5], tcp[6])
    gripper = entry.get("gripper_state", {}) or {}
    return [
        f"  joints (deg):  [{', '.join(f'{v:7.2f}' for v in q_deg)}]",
        f"  position (mm): x={tcp[0]*1000:8.2f}  y={tcp[1]*1000:8.2f}  z={tcp[2]*1000:8.2f}",
        f"  quat:          qw={tcp[3]:+.4f} qx={tcp[4]:+.4f} qy={tcp[5]:+.4f} qz={tcp[6]:+.4f}",
        f"  rpy (deg):     r={rpy[0]:+8.3f}  p={rpy[1]:+8.3f}  y={rpy[2]:+8.3f}",
        f"  gripper:       width={float(gripper.get('width_m', 0))*1000:.1f}mm  "
        f"force={float(gripper.get('force_n', 0)):.2f}N",
    ]


def _print_entry(entry: dict, label: str, header: str) -> None:
    print(f"--- {header} {label} ---")
    for line in _format_entry(entry):
        print(line)


# ---------- edits ----------


def _snap(value: float, step: float) -> float:
    if step <= 0:
        return value
    return round(value / step) * step


def _apply_edits(entry: dict, doc: dict, args) -> bool:
    """Mutate entry in place. Returns True iff something changed."""
    tcp = list(entry["tcp_pose_world"]["values"])
    pos = tcp[:3]
    rpy = list(quat_to_rpy_deg(tcp[3], tcp[4], tcp[5], tcp[6]))
    changed = False

    # ---- position ----
    if args.translate_mm:
        dx, dy, dz = args.translate_mm
        pos[0] += dx / 1000.0
        pos[1] += dy / 1000.0
        pos[2] += dz / 1000.0
        changed = True
    if args.set_x_mm is not None:
        pos[0] = args.set_x_mm / 1000.0
        changed = True
    if args.set_y_mm is not None:
        pos[1] = args.set_y_mm / 1000.0
        changed = True
    if args.set_z_mm is not None:
        pos[2] = args.set_z_mm / 1000.0
        changed = True

    # ---- orientation ----
    if args.set_rpy is not None:
        rpy = list(args.set_rpy)
        changed = True
    if args.set_roll is not None:
        rpy[0] = args.set_roll
        changed = True
    if args.set_pitch is not None:
        rpy[1] = args.set_pitch
        changed = True
    if args.set_yaw is not None:
        rpy[2] = args.set_yaw
        changed = True
    if args.snap_roll is not None:
        rpy[0] = _snap(rpy[0], args.snap_roll)
        changed = True
    if args.snap_pitch is not None:
        rpy[1] = _snap(rpy[1], args.snap_pitch)
        changed = True
    if args.snap_yaw is not None:
        rpy[2] = _snap(rpy[2], args.snap_yaw)
        changed = True
    if args.lock_vertical_down:
        rpy[0] = 180.0
        rpy[1] = 0.0  # yaw preserved
        changed = True
    if args.lock_vertical_up:
        rpy[0] = 0.0
        rpy[1] = 0.0  # yaw preserved
        changed = True
    if args.copy_orientation_from:
        src = (doc.get("poses", {}) or {}).get(args.copy_orientation_from)
        if src is None:
            print(
                f"--copy-orientation-from: pose {args.copy_orientation_from!r} not found",
                file=sys.stderr,
            )
            raise SystemExit(2)
        src_tcp = src["tcp_pose_world"]["values"]
        rpy = list(quat_to_rpy_deg(src_tcp[3], src_tcp[4], src_tcp[5], src_tcp[6]))
        changed = True

    if changed:
        quat = rpy_deg_to_quat(*rpy)
        entry["tcp_pose_world"]["values"] = pos + quat
    return changed


# ---------- with-robot recapture ----------


def _settle_via_robot(entry: dict, args, logger=None) -> None:
    """MovePTP to the new TCP, recapture joints + TCP from arm state."""
    from flexiv_helpers import (
        RobotSession,
        joints_to_jpos_deg,
        tcp_pose_to_coord_args,
    )

    tcp = entry["tcp_pose_world"]["values"]
    q_seed_deg = joints_to_jpos_deg(entry["q_rad"])
    coord_args = tcp_pose_to_coord_args(tcp, ref_joints_deg=q_seed_deg)

    with RobotSession(args.robot_sn, logger=logger) as session:
        flexivrdk = session.flexivrdk
        session.switch_mode("NRT_PRIMITIVE_EXECUTION")
        params = {
            "target": flexivrdk.Coord(*coord_args),
            "jntVelScale": int(args.jnt_vel_scale),
            "zoneRadius": "ZFine",
            "targetTolerLevel": 1,
            "jntAccMultiplier": 1.0,
            # enableFixRefJntPos=False so IK finds the closest joints to the
            # seed for the new TCP. The seed biases the solution toward the
            # captured arm shape but allows the small perturbation we want.
            "enableFixRefJntPos": False,
            "refJntPos": flexivrdk.JPos(q_seed_deg),
        }
        if logger is not None:
            logger.info("MovePTP -> tuned pose; will recapture from arm state")
        session.execute_primitive("MovePTP", params)
        session.wait_for_primitive("reachedTarget")
        _, state = session.selected_arm_state()
        q_rad_new = [float(v) for v in getattr(state, "q", [])]
        tcp_new = [float(v) for v in getattr(state, "tcp_pose", [])]
        if len(q_rad_new) != 7 or len(tcp_new) != 7:
            raise RuntimeError(
                f"arm state has q={len(q_rad_new)}, tcp={len(tcp_new)}; expected 7 each"
            )
        entry["q_rad"] = q_rad_new
        entry["q_deg"] = [math.degrees(v) for v in q_rad_new]
        entry["tcp_pose_world"]["values"] = tcp_new


# ---------- CLI ----------


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="tune_pose",
        description=(
            "Surgically edit one pose or path waypoint in pipeline_poses.yaml. "
            "Dry-run by default; pass --apply to commit."
        ),
    )
    p.add_argument("--pose-file", default=DEFAULT_POSE_FILE)

    sel = p.add_argument_group("target selection")
    sel.add_argument("--pose", default=None, help="Pose name (one of REQUIRED_POSES)")
    sel.add_argument("--path", default=None, help="Path name (for waypoint edits)")
    sel.add_argument(
        "--waypoint",
        type=int,
        default=None,
        help="1-indexed waypoint within --path",
    )

    pos = p.add_argument_group("position edits (millimeters)")
    pos.add_argument(
        "--translate-mm",
        type=float,
        nargs=3,
        metavar=("DX", "DY", "DZ"),
        default=None,
    )
    pos.add_argument("--set-x-mm", type=float, default=None)
    pos.add_argument("--set-y-mm", type=float, default=None)
    pos.add_argument("--set-z-mm", type=float, default=None)

    ori = p.add_argument_group("orientation edits (degrees, Tait-Bryan ZYX)")
    ori.add_argument(
        "--set-rpy",
        type=float,
        nargs=3,
        metavar=("ROLL", "PITCH", "YAW"),
        default=None,
    )
    ori.add_argument("--set-roll", type=float, default=None)
    ori.add_argument("--set-pitch", type=float, default=None)
    ori.add_argument("--set-yaw", type=float, default=None)
    ori.add_argument(
        "--snap-roll",
        type=float,
        default=None,
        help="Snap roll to nearest multiple of this many degrees (e.g. 1, 5, 90)",
    )
    ori.add_argument("--snap-pitch", type=float, default=None)
    ori.add_argument("--snap-yaw", type=float, default=None)
    ori.add_argument(
        "--lock-vertical-down",
        action="store_true",
        help="Set roll=180, pitch=0; keep yaw. Wrist TCP-Z points along world -Z.",
    )
    ori.add_argument(
        "--lock-vertical-up",
        action="store_true",
        help="Set roll=0, pitch=0; keep yaw. Wrist TCP-Z points along world +Z.",
    )
    ori.add_argument(
        "--copy-orientation-from",
        default=None,
        help="Use another pose's RPY; positions stay unchanged",
    )

    commit = p.add_argument_group("commit")
    commit.add_argument(
        "--apply",
        action="store_true",
        help="Write changes back to the pose file (otherwise dry-run)",
    )
    commit.add_argument(
        "--use-robot",
        action="store_true",
        help=(
            "Connect to the robot, MovePTP to the new TCP, and recapture both "
            "joint angles and TCP from the resulting arm state. Implies motion. "
            "Without this, q_rad/q_deg stay STALE relative to the edited TCP."
        ),
    )
    commit.add_argument("--robot-sn", default=DEFAULT_ROBOT_SN)
    commit.add_argument(
        "--jnt-vel-scale",
        type=int,
        default=DEFAULT_JNT_VEL_SCALE,
        help=f"MovePTP velocity scale when --use-robot, default {DEFAULT_JNT_VEL_SCALE}",
    )
    return p.parse_args(argv)


def run(args, logger=None) -> int:
    pose_file = _resolve_pose_file(args.pose_file)
    if not os.path.exists(pose_file):
        print(f"pose file not found: {pose_file}", file=sys.stderr)
        return 2

    doc = pose_schema.read_yaml(pose_file)
    entry, label = _get_entry(doc, args)
    _print_entry(entry, label, "BEFORE")

    changed = _apply_edits(entry, doc, args)
    if changed:
        print()
        _print_entry(entry, label, "AFTER ")

    # --use-robot without any new edits is a valid request: "resync the YAML
    # by moving the arm to the current TCP and recapturing q_rad to match."
    # That use case must still proceed even though _apply_edits reported no
    # changes.
    if not changed and not args.use_robot:
        print("\n(no edits specified — nothing to change)")
        return 0

    if not args.apply:
        print("\n(dry-run) pass --apply to save changes")
        return 0

    if args.use_robot:
        if logger is not None:
            logger.info(
                "--use-robot: settling via MovePTP + recapture"
                + ("" if changed else " (resync only; no TCP edits)")
            )
        _settle_via_robot(entry, args, logger=logger)
        print()
        _print_entry(entry, label, "POST-SETTLE")
    else:
        print()
        print(
            "WARNING: q_rad and tcp_pose_world are now inconsistent in the YAML."
        )
        print(
            "Playback will re-IK from tcp_pose_world (using q_rad as seed). "
            "Pass --use-robot to settle them together."
        )

    pose_schema.write_yaml(doc, pose_file)
    print(f"\nWrote {pose_file}")
    return 0


def main(argv=None) -> int:
    try:
        import spdlog  # type: ignore

        logger = spdlog.ConsoleLogger("TunePose")
    except Exception:
        logger = None
    args = parse_args(argv)
    return run(args, logger=logger)


if __name__ == "__main__":
    raise SystemExit(main())
