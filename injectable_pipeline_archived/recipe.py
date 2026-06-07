"""Injectable pipeline recipe — top-to-bottom, built incrementally.

Read this file straight through to understand the full cycle. Each phase is
one function; phases are called in cycle order from ``main()``. Tuning lives
in module-level constants near the top.

This file is being built one motion at a time. When a motion has been
validated on hardware, the next motion is appended below it. Pose / path
data lives in ``pipeline_poses.yaml`` — open that file to see the actual
joint values, TCP poses, and gripper widths.

Run:
    python recipe.py

The first thing the recipe does at every launch is move the arm to the
captured ``home`` pose. If the arm is far from home, watch for fixture
collisions on the way in. Hand on E-stop.
"""

from __future__ import annotations

import os

import pose_schema
from arduino_client import ArduinoClient, DEFAULT_READY_TIMEOUT_S
from flexiv_helpers import (
    RobotSession,
    descend_until_force_or_depth,
    gripper_set,
    move_ptp_joint,
    move_z_relative,
    pose_offset_z,
    walk_path,
    wiggle_about_virtual_point,
    zero_ft_sensor,
)


# ---------- tuning constants ----------

ROBOT_SN = "Rizon4-062930"
POSE_FILE = "pipeline_poses.yaml"

# Conservative velocity during the recipe build phase. Bump back to 10 (the
# orchestrator default) once each motion is trusted on hardware.
MOVE_JNT_VEL_SCALE = 5

# Tray pre-grasp -> grasp: pure -Z Cartesian descent. Tune as the tray
# geometry settles. 50 mm = 5 cm.
PICKUP_DESCENT_MM = 50
PICKUP_DESCENT_VEL_M_S = 0.03  # 3 cm/s; gentle for the build phase

# Post-grasp +Z lift to clear the tray with the part held. Default puts the
# wrist back at roughly pre_grasp height; raise for extra clearance.
PICKUP_LIFT_MM = 50
PICKUP_LIFT_VEL_M_S = 0.03

# Gripper (Flexiv GN01).
GRIPPER_NAME = "Flexiv-GN01"
INIT_GRIPPER_AT_STARTUP = True   # Set False to skip the ~4s calibration motion
                                 # on subsequent runs (once you trust the build).
GRIPPER_CLOSE_WIDTH_M = 0.01    # Almost-closed; tune to ≈ part diameter
GRIPPER_OPEN_WIDTH_M  = 0.030    # Fully open
GRIPPER_VEL_M_S       = 0.05
GRIPPER_CLOSE_FORCE_N = 20.0     # Gentle for the build phase; bump later
GRIPPER_CLOSE_SETTLE_S = 0.5     # Extra wait after close so the force-stall
                                 # settles before subsequent arm motion. The
                                 # controller's is_moving clears the moment
                                 # stall is detected, which is too eager for
                                 # downstream lift.

# Phase 2 LOAD descent into the vise. Continuous Cartesian Motion-Force mode
# with hardware-regulated soft contact (SetMaxContactWrench) + 1 kHz software
# wrench polling. Force threshold is the |F| (xyz norm) at which the descent
# aborts. Bump up to 15 N (spec INSERT_FORCE_THRESHOLD_N) once trusted.
LOAD_DESCENT_MAX_MM       = 70      # safety budget for downward travel
LOAD_DESCENT_VEL_M_S      = 0.005   # 5 mm/s
LOAD_DESCENT_POLL_S       = 0.001   # 1 kHz wrench check
LOAD_FORCE_THRESHOLD_N    = 5.0     # |F_xyz| threshold for abort

# Vise clamp force when loading the injectable. Spec default is 4.0 kg.
VISE_TARGET_FORCE_KG = 6.0

# Post-clamp retreat from the slot region. Vise-approach invariant: the first
# VISE_VERTICAL_APPROACH_MM of motion when exiting the slot must be pure +Z
# before any lateral travel is permitted. 100 mm matches the invariant.
LOAD_RETREAT_MM      = 100
LOAD_RETREAT_VEL_M_S = 0.03

# Phase 3 CUT recipe. Recipe1 second cut (frees the plastic top) — matches
# the orchestrator's PARAMS defaults from specifications.md.
CUT_Z_MM  = 91.0
CUT_X_MM  = 111.0
CUT_DEG   = 359.0

# Mimed-cut mode: blade still fires and the cutting-machine motion still
# runs, but X is shifted further out by CUT_DRY_RUN_X_OFFSET_MM so the
# blade clears the part. Use to validate the cut motion / sequencing on
# hardware without actually cutting anything. Flip to False for a real cut.
CUT_DRY_RUN              = False
CUT_DRY_RUN_X_OFFSET_MM  = 10.0

# Phase 3.5 BREAK_OFF: after the cut, grip the cut-off top and wiggle it about
# a virtual pivot to shear off any remaining bridge.
BREAK_OFF_APPROACH_MM            = 100    # arm goes (above_vise + this) then descends pure -Z by this
BREAK_OFF_APPROACH_VEL_M_S       = 0.03   # 3 cm/s for the pure-Z descent to above_vise
BREAK_OFF_GRIPPER_CLOSE_FORCE_N  = 80.0   # firm grip — supports the shear during wiggle
BREAK_OFF_PIVOT_OFFSET_CM        = 6.0    # along EE local +X. See note in this phase's docstring.
BREAK_OFF_WIGGLE_YAW_DEG         = 15.0   # ±15° per cycle
BREAK_OFF_WIGGLE_REPEAT_COUNT    = 10     # 10 cycles
BREAK_OFF_WIGGLE_VEL_SCALE       = 10     # MovePTP velocity scale during wiggle


# ---------- phases ----------


def phase_pickup(session, poses: dict, paths: dict) -> None:
    """Phase 1 — tray to above_vise.

    Step 1: jog to ``home`` as a known clear starting point.
    Step 2: stage above the tray at ``pickup_pre_grasp`` (gripper open, wrist
            rolled so fingers will close perpendicular to the part's long axis).
    Step 3: pure -Z Cartesian descent by PICKUP_DESCENT_MM. Orientation held
            constant — gripper drops straight onto the part. No compliance
            yet; if the part is misaligned by more than wrist clearance allows,
            we'll add Cartesian impedance here later.
    Step 4: close gripper on the part to GRIPPER_CLOSE_WIDTH_M at gentle force.
            Blocks until the gripper stops moving (either reaches target width
            or stalls against the part at the force limit). The gripper was
            forced fully open at phase entry, so this close always starts
            from a known state.
    Step 5: pure +Z Cartesian lift by PICKUP_LIFT_MM with the part held.
            Orientation locked; the wrist comes straight up out of the tray.
    Step 6: walk the captured ``lifted_to_above_vise`` path (joint-locked
            intermediate waypoints — reproduces the demonstrated arm shape
            so the elbow doesn't swing into fixtures) and arrive at
            ``above_vise``. End state: gripper holds part, wrist staged
            above the vise slot.
    """
   
    # step 0, move to home
    move_ptp_joint(session, poses["home"], vel_scale=MOVE_JNT_VEL_SCALE)
    # step 1, ensure the gripper is fully open before any approach motion.
    # Recovers from a previous run that ended mid-grip, and guarantees the
    # fingers won't collide with the part during the descent.
    gripper_set(session.gripper, GRIPPER_OPEN_WIDTH_M, vel_m_s=GRIPPER_VEL_M_S)
    # step 2, move above grasp point on tray
    move_ptp_joint(session, poses["pickup_pre_grasp"], vel_scale=MOVE_JNT_VEL_SCALE)
    # step 3, lower down
    move_z_relative(
        session,
        -PICKUP_DESCENT_MM / 1000.0,
        vel_m_s=PICKUP_DESCENT_VEL_M_S,
    )
    # step 4, close gripper on the part
    gripper_set(
        session.gripper,
        GRIPPER_CLOSE_WIDTH_M,
        vel_m_s=GRIPPER_VEL_M_S,
        force_n=GRIPPER_CLOSE_FORCE_N,
        settle_after_s=GRIPPER_CLOSE_SETTLE_S,
    )
    # step 5, lift the part straight up out of the tray
    move_z_relative(
        session,
        +PICKUP_LIFT_MM / 1000.0,
        vel_m_s=PICKUP_LIFT_VEL_M_S,
    )
    # step 6, walk captured path through its waypoints, arrive at above_vise
    walk_path(
        session,
        paths["lifted_to_above_vise"],
        end_pose=poses["above_vise"],
        vel_scale=MOVE_JNT_VEL_SCALE,
    )
    # next motion goes below this line as we add steps.


def phase_load(session, poses: dict, paths: dict, arduino, logger=None) -> None:
    """Phase 2 — above_vise to vise.

    Step 7: zero the F/T sensor (with the part in the gripper so its weight
            is in the baseline), then a force-aborted Cartesian descent in
            ``NRT_CARTESIAN_MOTION_FORCE`` mode. The arm arrives at this
            phase already staged at above_vise with the part held; the
            descent drops the part straight into the slot until the wrist
            feels resistance (|F_xyz| > LOAD_FORCE_THRESHOLD_N) or until
            LOAD_DESCENT_MAX_MM of travel.
    Step 8: clamp the vise on the part at VISE_TARGET_FORCE_KG. Blocks on
            the firmware DONE — vise reaches target force or stalls.
    Step 9: open the gripper to release the part now that the vise holds it.
            The arm can let go cleanly.
    Step 10: pure +Z retreat by LOAD_RETREAT_MM. Vise-approach invariant
             requires the first 100 mm out of the slot region to be straight
             vertical before any lateral motion. After this step the arm is
             clear and may walk to the next phase's anchor pose.
    """
    # step 7a, zero the F/T sensor so contact detection is unbiased
    zero_ft_sensor(session)
    # step 7b, force-aborted descent into the vise (continuous 1 kHz poll)
    descend_until_force_or_depth(
        session,
        max_depth_m=LOAD_DESCENT_MAX_MM / 1000.0,
        force_threshold_n=LOAD_FORCE_THRESHOLD_N,
        vel_m_s=LOAD_DESCENT_VEL_M_S,
        poll_dt_s=LOAD_DESCENT_POLL_S,
        logger=logger,
    )
    # step 8, clamp the vise on the part
    arduino.close_vise(target_force_kg=VISE_TARGET_FORCE_KG)
    # step 9, release the part from the gripper (vise now holds it)
    gripper_set(
        session.gripper,
        GRIPPER_OPEN_WIDTH_M,
        vel_m_s=GRIPPER_VEL_M_S,
    )
    # step 10, pure +Z retreat out of the slot region (vise-approach invariant)
    move_z_relative(
        session,
        +LOAD_RETREAT_MM / 1000.0,
        vel_m_s=LOAD_RETREAT_VEL_M_S,
    )
    # next motion goes below this line as we add steps.


def phase_cut(session, poses: dict, paths: dict, arduino, logger=None) -> None:
    """Phase 3 — park + fire CUT_HEIGHT.

    Step 11: MovePTP (joint-locked) to ``safe_intermediate``. This pose was
             captured outside the blade's swing arc so the cut can fire
             without the arm in the way.
    Step 12: rotate the cutting-machine rotary stage to 0 deg (firmware
             precondition for CUT_HEIGHT — rejects with INVALID_STATE
             otherwise), then ``arduino.cut_height(z, x, deg)`` fires the
             blade. Blocks on Arduino DONE. Robot does not move.

    Pre-conditions (asserted implicitly by the recipe ordering): vise is
    clamped on the part (phase_load step 8), gripper has released (step 9),
    arm is clear of the slot region (step 10's +Z retreat).
    """
    # step 11, park at safe_intermediate well outside the blade swing
    move_ptp_joint(
        session,
        poses["safe_intermediate"],
        vel_scale=MOVE_JNT_VEL_SCALE,
        joint_locked=True,
    )
    # step 12a, ensure rotary is at 0 (firmware precondition for CUT_HEIGHT).
    # A previous CUT_HEIGHT sweep leaves the rotary at CUT_DEG, so the next
    # cut would be rejected without this defensive reset.
    arduino.rotate_abs(0.0)
    # step 12b, fire the cut (arduino blocks until the cutting machine returns).
    # When CUT_DRY_RUN is set we mime: blade still fires and the full motion
    # runs, but X is nudged out by CUT_DRY_RUN_X_OFFSET_MM so the blade
    # passes clear of the part.
    cut_x_mm = CUT_X_MM - (CUT_DRY_RUN_X_OFFSET_MM if CUT_DRY_RUN else 0.0)
    if logger is not None:
        tag = "MIMED CUT (dry-run)" if CUT_DRY_RUN else "CUT"
        logger.info(
            f"{tag}: z={CUT_Z_MM}mm x={cut_x_mm}mm deg={CUT_DEG} — BLADE FIRES"
        )
    arduino.cut_height(z_mm=140, x_mm=110, deg=360)
    arduino.cut_height(z_mm=140, x_mm=110.8, deg=360)


def phase_break_off(
    session, poses: dict, paths: dict, arduino, logger=None
) -> None:
    """Phase 3.5 — break the cut-off top free via grip + wiggle.

    After phase_cut the cut may not fully sever the top; a thin bridge
    sometimes remains. This phase brings the robot in, grips the top
    firmly, and wiggles it about a virtual pivot to shear the bridge.

    Step A: Arduino HOME_ALL (defensive — CUT_HEIGHT already tails into a
            home, but ensures clean state for the robot's approach).
    Step B: Robot → home, gripper open.
    Step C: Robot → above_vise (staging above the slot).
    Step D: F/T zero + force-aborted -Z descent (same primitive used in
            phase_load) until the gripper contacts the top.
    Step E: Gripper closes at BREAK_OFF_GRIPPER_CLOSE_FORCE_N (high force
            so the wiggle doesn't slip in the fingers).
    Step F: Wiggle: ±BREAK_OFF_WIGGLE_YAW_DEG yaw about a virtual pivot
            BREAK_OFF_PIVOT_OFFSET_CM along the gripper's TCP +X axis,
            BREAK_OFF_WIGGLE_REPEAT_COUNT cycles. With the coaxial-grip
            wrist orientation, TCP +X is *horizontal* (= world -Y) — so
            the gripper sweeps in an arc, applying combined twist + lateral
            shear to the bridge. Set BREAK_OFF_PIVOT_OFFSET_CM = 0 for
            pure in-place twist instead.

    Post: arm is sitting at above_vise with the broken-off top in the
    gripper. Cleanup (retreat / disposal) is left for a later phase.
    """

    """What I think the robot should do here: 
    1. arduino home all
    2. robot is at safe_intermediate
    3. robot moves to a point 150 mm above above_vise
    4. robot moves down in z to reach above_vise
    5. robot grips with 80N force 
    6. wiggle to break-off 
    """

    # 1. arduino home — defensive
    arduino.home_all()

    # 3. vertical approach: go to (above_vise.xy, above_vise.z + BREAK_OFF_APPROACH_MM).
    #    joint_locked=False lets IK pick joints near the captured seed for
    #    the offset TCP. From safe_intermediate, this is a safe move — the
    #    anchor is high in the air above the slot.
    above_anchor = pose_offset_z(
        poses["above_vise"], +BREAK_OFF_APPROACH_MM / 1000.0
    )
    move_ptp_joint(
        session,
        above_anchor,
        vel_scale=MOVE_JNT_VEL_SCALE,
        joint_locked=False,
    )

    # 4. pure -Z Cartesian descent BREAK_OFF_APPROACH_MM to land exactly at above_vise.
    move_z_relative(
        session,
        -BREAK_OFF_APPROACH_MM / 1000.0,
        vel_m_s=BREAK_OFF_APPROACH_VEL_M_S,
    )

    # 5. grip firmly at 80N — supports the shear during the wiggle.
    gripper_set(
        session.gripper,
        GRIPPER_CLOSE_WIDTH_M,
        vel_m_s=GRIPPER_VEL_M_S,
        force_n=BREAK_OFF_GRIPPER_CLOSE_FORCE_N,
        settle_after_s=GRIPPER_CLOSE_SETTLE_S,
    )

    # 6. wiggle to shear off the remaining bridge.
    if logger is not None:
        logger.info(
            f"BREAK_OFF wiggle: ±{BREAK_OFF_WIGGLE_YAW_DEG}° yaw × "
            f"{BREAK_OFF_WIGGLE_REPEAT_COUNT} cycles, "
            f"pivot offset {BREAK_OFF_PIVOT_OFFSET_CM} cm along TCP +X"
        )
    wiggle_about_virtual_point(
        session,
        pivot_offset_cm=BREAK_OFF_PIVOT_OFFSET_CM,
        yaw_deg=BREAK_OFF_WIGGLE_YAW_DEG,
        repeat_count=BREAK_OFF_WIGGLE_REPEAT_COUNT,
        vel_scale=BREAK_OFF_WIGGLE_VEL_SCALE,
    )


# def phase_remove_top(...): ...
# def phase_remove_spring(...): ...
# def phase_remove_body(...): ...


# ---------- main ----------


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    pose_file = os.path.join(here, POSE_FILE)
    doc = pose_schema.read_yaml(pose_file)
    # The recipe is built incrementally; full-document validation
    # (every required pose + path captured) would block partial runs. Each
    # phase function instead pulls entries by name and surfaces a clear
    # KeyError if anything it touches is missing.
    poses = doc.get("poses", {}) or {}
    paths = doc.get("paths", {}) or {}

    try:
        import spdlog  # type: ignore

        logger = spdlog.ConsoleLogger("Recipe")
    except Exception:
        logger = None

    # Arduino is shared across phases (vise, blade, etc.). Connecting at the
    # top of main() means a single DTR-reset for the whole session — the
    # firmware boots with homed=False, so we auto-home before any phase that
    # uses vise/cut primitives.
    arduino = ArduinoClient(
        ready_timeout_s=DEFAULT_READY_TIMEOUT_S,
        done_timeout_s=60.0,
        logger=logger,
    )
    try:
        arduino.connect()
        status = arduino.get_status()
        if not status.get("homed"):
            if logger is not None:
                logger.info("Arduino not homed; sending HOME_ALL")
            arduino.home_all()
        elif logger is not None:
            logger.info("Arduino already homed")

        with RobotSession(ROBOT_SN, logger=logger) as session:
            session.setup_gripper(GRIPPER_NAME, init=INIT_GRIPPER_AT_STARTUP)
            session.switch_mode("NRT_PRIMITIVE_EXECUTION")
            phase_pickup(session, poses, paths)
            phase_load(session, poses, paths, arduino, logger=logger)
            phase_cut(session, poses, paths, arduino, logger=logger)
            phase_break_off(session, poses, paths, arduino, logger=logger)
            # phase_remove_spring(...)
            # phase_remove_body(...)

        if logger is not None:
            logger.info("Recipe complete.")
        return 0
    finally:
        try:
            arduino.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
