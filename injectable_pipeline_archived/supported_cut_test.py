"""Supported-cut experiment: cut a clamped sample with the robot holding the
top during the final HOLD_LAST_DEG of the second pass.

Sequence:
1. Pass 1 — full 360° at X_PASS1_MM (shallow), normal cut speed.
2. Pass 2A — rotate to (360 - HOLD_LAST_DEG)° at X_PASS2_MM (deeper) at the
   normal cut speed. Blade stays on; X stays at X_PASS2_MM.
3. Robot intervention — gripper open, MovePTP to ``cut_support_grip``,
   gripper closes on the top at high force.
4. Switch to ``NRT_CARTESIAN_MOTION_FORCE`` mode with a stiffness vector that
   is stiff in world Z + tilt (Krot_x, Krot_y), soft in world XY + rotation
   about world Z (which equals J7 since the grip is coaxial). The arm
   passively follows the rotary's last few degrees.
5. Pass 2B — rotate the rotary the remaining HOLD_LAST_DEG at a slow speed
   so the impedance loop can track without lag.
6. Restore stiff mode; arm lifts the top vertically by TOP_LIFT_MM.
7. Arduino cleanup — X back to staging, blade off, rotary unwind, X/Z home.

Pre-condition: vise is clamped on a part. Use ``close_vise.py`` to clamp.
Pose ``cut_support_grip`` must exist in ``pipeline_poses.yaml`` (capture it
with ``capture_pose.py cut_support_grip``).
"""

from __future__ import annotations

import os
import sys
import time

import pose_schema
from arduino_client import ArduinoClient, DEFAULT_READY_TIMEOUT_S
from flexiv_helpers import (
    RobotSession,
    gripper_set,
    move_ptp_joint,
    move_z_relative,
    wiggle_about_virtual_point,
)


# ---------- tuning constants ----------

# Arduino cut geometry / speeds
Z_CUT_MM             = 90.0
X_STAGE_MM           = 100.0
X_PASS1_MM           = 110.0     # shallow cut
X_PASS2_MM           = 110.8     # deeper cut
TOTAL_DEG            = 360.0
HOLD_LAST_DEG        = 5.0       # rotary held by the arm during this final segment

SLOW_CUT_FEED_MM_S   = 10        # X engage / retract while blade is on
FAST_TRAVEL_MM_S     = 100       # non-cut X/Z moves
ROT_CUT_SPEED        = 60      # firmware default (60 deg/s)
ROT_RETURN_SPEED     = 3000     # firmware default
ROT_SLOW_HOLD_SPEED  = 10.0      # deg/s for the held last segment

# Robot
ROBOT_SN             = "Rizon4-062930"
GRIPPER_NAME         = "Flexiv-GN01"
GRIPPER_OPEN_WIDTH_M = 0.030
GRIPPER_CLOSE_WIDTH_M    = 0.005
GRIPPER_VEL_M_S          = 0.05
GRIPPER_CLOSE_FORCE_N    = 40.0  # high — supports the top during the cut
GRIPPER_CLOSE_SETTLE_S   = 0.5

MOVE_JNT_VEL_SCALE   = 5         # joint velocity scale for MovePTP

# Approach to cut_support_grip: arm goes to (target.xy, target.z + APPROACH_MM)
# first, then descends pure -Z. Mirrors the vise-approach invariant — the
# last leg of the approach to the cut top is straight down.
CUT_SUPPORT_APPROACH_MM     = 100.0
CUT_SUPPORT_APPROACH_VEL_M_S = 0.03  # 3 cm/s for the vertical descent

# Compliance stiffness for the held segment.
# [Kx, Ky, Kz, Krot_x, Krot_y, Krot_z]
# Translations in N/m; rotations in N·m/rad.
#   Kx, Ky low      : let the gripper drift radially if cut isn't perfectly concentric
#   Kz high         : don't let the top spring upward when the bridge breaks
#   Krot_x/y high   : don't let the gripper tilt off-axis
#   Krot_z low      : allow J7 to rotate with the rotary
COMPLIANT_K = [50.0, 50.0, 3000.0, 40.0, 40.0, 5.0]

# Post-cut wiggle: oscillate the gripper about a virtual pivot to shake any
# remaining bridge free before the lift. Pivot offset is along the gripper's
# local +X axis (0 cm = rotate in place). Yaw oscillation = rotation about the
# gripper's TCP-Z which is aligned with the part axis for the coaxial grip.
WIGGLE_OFFSET_CM     = 0.0       # 0 = rotate in place
WIGGLE_ROLL_DEG      = 0.0
WIGGLE_PITCH_DEG     = 0.0
WIGGLE_YAW_DEG       = 5.0       # ± yaw amplitude
WIGGLE_REPEAT_COUNT  = 3         # number of ± cycles
WIGGLE_VEL_SCALE     = 10        # MovePTP joint velocity scale during wiggle

# Post-cut lift: arm goes straight up after the cut completes.
TOP_LIFT_MM          = 100.0
TOP_LIFT_VEL_M_S     = 0.03

POSE_FILE            = "pipeline_poses.yaml"


# ---------- the strategy ----------


def _pose_offset_z(pose_entry: dict, dz_m: float) -> dict:
    """Return a shallow copy of ``pose_entry`` with TCP Z shifted by dz_m.

    Used to derive an "above" approach pose from a captured pose without
    needing a separate YAML entry. The captured joint angles are passed
    through as the IK seed for the offset target (move_ptp_joint with
    joint_locked=False lets IK pick joints near the seed for the new TCP).
    """
    tcp_order = list(pose_entry["tcp_pose_world"]["order"])
    tcp_vals = list(pose_entry["tcp_pose_world"]["values"])
    tcp_vals[2] = float(tcp_vals[2]) + float(dz_m)
    return {
        "q_rad": list(pose_entry["q_rad"]),
        "tcp_pose_world": {"order": tcp_order, "values": tcp_vals},
    }


def supported_cut(session, arduino, poses, logger=None):
    """Run the supported-cut sequence end-to-end."""

    def log(msg):
        if logger is not None:
            logger.info(msg)

    flexivrdk = session.flexivrdk
    robot = session.robot

    # ---- Robot: park at safe_intermediate, gripper open ----
    # Get the arm out of the cutter's swing arc before any blade motion.
    # Open the gripper too, so the engage step starts from a known state and
    # any stray hardware in the fingers gets released.
    log("Parking arm at safe_intermediate; gripper open")
    gripper_set(session.gripper, GRIPPER_OPEN_WIDTH_M, vel_m_s=GRIPPER_VEL_M_S)
    move_ptp_joint(
        session,
        poses["safe_intermediate"],
        vel_scale=MOVE_JNT_VEL_SCALE,
        joint_locked=True,
    )

    # ---- Arduino: Pass 1 (full 360° at shallow X) ----
    log("Pass 1: full 360° at shallow X")
    arduino.rotate_abs(0.0, speed=ROT_RETURN_SPEED)
    arduino.move_z_abs(Z_CUT_MM, feed_mm_s=FAST_TRAVEL_MM_S)
    arduino.move_x_abs(X_STAGE_MM, feed_mm_s=FAST_TRAVEL_MM_S)
    arduino.set_blade(True)
    arduino.move_x_abs(X_PASS1_MM, feed_mm_s=SLOW_CUT_FEED_MM_S)
    arduino.rotate_abs(TOTAL_DEG, speed=ROT_CUT_SPEED)
    arduino.move_x_abs(X_STAGE_MM, feed_mm_s=SLOW_CUT_FEED_MM_S)
    arduino.rotate_abs(0.0, speed=ROT_RETURN_SPEED)

    # ---- Arduino: Pass 2A (deeper, stop HOLD_LAST_DEG short) ----
    log(f"Pass 2A: deeper X, rotate to {TOTAL_DEG - HOLD_LAST_DEG}°")
    arduino.move_x_abs(X_PASS2_MM, feed_mm_s=SLOW_CUT_FEED_MM_S)
    arduino.rotate_abs(TOTAL_DEG - HOLD_LAST_DEG, speed=ROT_CUT_SPEED)
    # Blade STAYS ON. X STAYS at X_PASS2_MM. Pause for the robot.

    # ---- Robot: engage on the top ----
    # Approach in two segments to match the vise-approach invariant: first a
    # joint-space MovePTP to CUT_SUPPORT_APPROACH_MM above the target pose,
    # then a pure -Z Cartesian descent for the final 100 mm onto the part.
    # Gripper was opened at the safe-park step.
    log(
        f"Robot intervention: approach (target + {CUT_SUPPORT_APPROACH_MM} mm) "
        f"→ pure -Z descent → grip"
    )
    above_grip = _pose_offset_z(
        poses["cut_support_grip"], +CUT_SUPPORT_APPROACH_MM / 1000.0
    )
    move_ptp_joint(
        session,
        above_grip,
        vel_scale=MOVE_JNT_VEL_SCALE,
        joint_locked=False,    # let IK find joints for the offset TCP near the seed
    )
    move_z_relative(
        session,
        -CUT_SUPPORT_APPROACH_MM / 1000.0,
        vel_m_s=CUT_SUPPORT_APPROACH_VEL_M_S,
    )
    gripper_set(
        session.gripper,
        GRIPPER_CLOSE_WIDTH_M,
        vel_m_s=GRIPPER_VEL_M_S,
        force_n=GRIPPER_CLOSE_FORCE_N,
        settle_after_s=GRIPPER_CLOSE_SETTLE_S,
    )

    # ---- Switch to compliant motion-force mode for the held segment ----
    log(f"Switching to NRT_CARTESIAN_MOTION_FORCE; impedance K={COMPLIANT_K}")
    session.switch_mode("NRT_CARTESIAN_MOTION_FORCE")
    try:
        # All axes motion-controlled (no force-control axis); impedance set
        # via SetCartesianImpedance.
        robot.SetForceControlAxis([False] * 6)
        robot.SetCartesianImpedance(COMPLIANT_K)
        # Anchor at current TCP — controller holds against any deviation
        # according to the stiffness. The low Krot_z lets J7 rotate freely
        # under torque from the rotary.
        _, arm = session.selected_arm_state()
        anchor_tcp = list(getattr(arm, "tcp_pose", []))
        if len(anchor_tcp) != 7:
            raise RuntimeError(
                f"tcp_pose has {len(anchor_tcp)} values; expected 7"
            )
        robot.SendCartesianMotionForce(anchor_tcp, [0.0] * 6, [0.0] * 6, 0.05)
        # Let the controller settle into the new mode briefly before commanding
        # the rotary to move (avoids any initial transient kicking the arm).
        time.sleep(0.2)

        # ---- Arduino: Pass 2B (the held last HOLD_LAST_DEG, slow) ----
        log(
            f"Pass 2B: rotate {TOTAL_DEG - HOLD_LAST_DEG}° → {TOTAL_DEG}° "
            f"@ {ROT_SLOW_HOLD_SPEED} deg/s (arm passively follows)"
        )
        arduino.rotate_abs(TOTAL_DEG, speed=ROT_SLOW_HOLD_SPEED)
        # Small dwell so the arm settles after the rotary stops.
        time.sleep(0.3)
    finally:
        # Restore nominal stiffness so the next motion doesn't carry our
        # low-Krot_z setting forward, then return to primitive mode.
        try:
            robot.SetCartesianImpedance(robot.info().K_x_nom)
        except Exception as exc:  # noqa: BLE001
            log(f"warn: failed to restore nominal impedance: {exc}")
        session.switch_mode("NRT_PRIMITIVE_EXECUTION")

    # ---- Robot: wiggle to break any remaining bridge ----
    log(
        f"Wiggle: yaw=±{WIGGLE_YAW_DEG}° about virtual pivot "
        f"(offset={WIGGLE_OFFSET_CM}cm) × {WIGGLE_REPEAT_COUNT} cycles"
    )
    wiggle_about_virtual_point(
        session,
        pivot_offset_cm=WIGGLE_OFFSET_CM,
        roll_deg=WIGGLE_ROLL_DEG,
        pitch_deg=WIGGLE_PITCH_DEG,
        yaw_deg=WIGGLE_YAW_DEG,
        repeat_count=WIGGLE_REPEAT_COUNT,
        vel_scale=WIGGLE_VEL_SCALE,
    )

    # ---- Robot: lift the top straight up ----
    log(f"Top lift: +{TOP_LIFT_MM} mm in world Z")
    move_z_relative(
        session,
        +TOP_LIFT_MM / 1000.0,
        vel_m_s=TOP_LIFT_VEL_M_S,
    )

    # ---- Arduino: cleanup ----
    log("Arduino cleanup: X retract → blade off → rotary unwind → X/Z home")
    arduino.move_x_abs(X_STAGE_MM, feed_mm_s=SLOW_CUT_FEED_MM_S)
    arduino.set_blade(False)
    arduino.rotate_abs(0.0, speed=ROT_RETURN_SPEED)
    arduino.move_x_abs(0.0, feed_mm_s=FAST_TRAVEL_MM_S)
    arduino.move_z_abs(0.0, feed_mm_s=FAST_TRAVEL_MM_S)
    log("Supported cut complete.")


# ---------- harness ----------


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    pose_file = os.path.join(here, POSE_FILE)
    doc = pose_schema.read_yaml(pose_file)
    poses = doc.get("poses", {}) or {}
    if "cut_support_grip" not in poses:
        print(
            "cut_support_grip not found in pose file. Capture it first:\n"
            "    python capture_pose.py cut_support_grip",
            file=sys.stderr,
        )
        return 2

    try:
        import spdlog  # type: ignore

        logger = spdlog.ConsoleLogger("SupportedCut")
    except Exception:
        logger = None

    arduino = ArduinoClient(
        ready_timeout_s=DEFAULT_READY_TIMEOUT_S, done_timeout_s=60.0, logger=logger
    )
    arduino.connect()
    try:
        if not arduino.get_status().get("homed"):
            arduino.home_all()

        with RobotSession(ROBOT_SN, logger=logger) as session:
            # init=False — assume gripper was initialized in a prior session
            # this lab day. Pass --init by editing if needed.
            session.setup_gripper(GRIPPER_NAME, init=False)
            session.switch_mode("NRT_PRIMITIVE_EXECUTION")
            supported_cut(session, arduino, poses, logger=logger)
        return 0
    finally:
        try:
            arduino.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
