"""Flexiv RDK convenience helpers used by the injectable pipeline.

Wraps the boilerplate that the existing ``project/record_robot_waypoints.py`` and
``project/play_recorded_waypoints.py`` scripts duplicate: robot init + fault clear +
``Enable`` + ``operational()`` polling, primitive dispatch with the v2 joint-group API,
named mode resolution, and Cartesian-impedance configuration.

The existing scripts are intentionally left alone (see ``planning/milestones.md`` M2).
This module is consumed only by the new ``injectable_pipeline`` code.
"""

from __future__ import annotations

import math
import time
from typing import Any, Iterable


# Fallback integer values for ``flexivrdk.Mode`` when the installed RDK build does not
# expose the named attribute. Only modes whose integers we trust against the documented
# v1.9 declaration order are listed; Cartesian modes are intentionally omitted so a
# missing attribute raises loudly rather than picking the wrong enum value.
MODE_VALUES: dict[str, int] = {
    "NRT_PRIMITIVE_EXECUTION": 8,
    "NRT_JOINT_IMPEDANCE": 4,
}


def _load_flexivrdk():
    """Lazy import so tests can run without the SDK installed."""
    import flexivrdk  # type: ignore

    return flexivrdk


def rdk_mode(name: str, flexivrdk_module=None):
    """Resolve a mode name to ``flexivrdk.Mode`` via attribute lookup or integer map."""
    flexivrdk = flexivrdk_module or _load_flexivrdk()
    if hasattr(flexivrdk.Mode, name):
        return getattr(flexivrdk.Mode, name)
    if name in MODE_VALUES:
        try:
            return flexivrdk.Mode(MODE_VALUES[name])
        except Exception as exc:  # pragma: no cover - depends on RDK build
            raise RuntimeError(
                f"Cannot resolve flexivrdk.Mode.{name} via attribute lookup or integer map"
            ) from exc
    raise RuntimeError(f"Unknown mode name: {name}")


def _vector(values) -> list[float]:
    return [float(v) for v in values]


def selected_arm_state(robot) -> tuple[str | None, Any]:
    """Pick the ARM joint group's state from ``robot.states()`` when present."""
    states = robot.states()
    if not isinstance(states, dict):
        return None, states
    for group, state in states.items():
        if "ARM" in str(group).upper():
            return str(group), state
    if not states:
        return None, None
    group, state = next(iter(states.items()))
    return str(group), state


def execute_primitive(robot, name: str, params: dict, flexivrdk_module=None) -> None:
    """Submit a primitive, using the v2 joint-group API when available."""
    flexivrdk = flexivrdk_module
    if hasattr(robot, "groups"):
        if flexivrdk is None:
            flexivrdk = _load_flexivrdk()
        if hasattr(flexivrdk, "PrimitiveArgs"):
            joint_groups = robot.groups()
            robot.ExecutePrimitive(
                {group: flexivrdk.PrimitiveArgs(name, params) for group in joint_groups}
            )
            return
    # Legacy single-argument form for older RDK versions.
    robot.ExecutePrimitive(name, params)


def _primitive_state_value(state, key: str):
    """Look ``key`` up in a primitive-state container.

    Handles both v2 nested form (``state.names_and_values`` is the dict) and
    v1.9 flat form (``state`` itself is the dict). Returns ``None`` when the
    value isn't a subscriptable mapping — this happens when the robot is in
    a non-operational state (e.g. after E-stop) and ``primitive_states()``
    returns scalar values where a state object is expected.
    """
    values = getattr(state, "names_and_values", state)
    if not isinstance(values, dict):
        return None
    return values.get(key)


def _primitive_state_reached(robot, state_key: str) -> bool:
    states = robot.primitive_states()
    if isinstance(states, dict):
        if state_key in states:
            value = states[state_key]
            if isinstance(value, (list, tuple)):
                return all(bool(v) for v in value)
            return bool(value)
        if not states:
            return False
        for state in states.values():
            value = _primitive_state_value(state, state_key)
            # value is None when this entry isn't a proper state container
            # (e.g. a scalar in a degraded primitive_states response). Treat
            # as "no info" and don't let it crash the poll.
            if value is None:
                return False
            if isinstance(value, (list, tuple)):
                if not all(bool(v) for v in value):
                    return False
            elif not bool(value):
                return False
        return True
    return False


def wait_for_primitive(
    robot,
    state_key: str = "reachedTarget",
    dt: float = 0.2,
    timeout_s: float | None = None,
) -> None:
    """Poll ``robot.primitive_states()`` until the named state goes truthy."""
    deadline = None if timeout_s is None else time.monotonic() + max(0.0, timeout_s)
    while not _primitive_state_reached(robot, state_key):
        if deadline is not None and time.monotonic() >= deadline:
            raise TimeoutError(
                f"Primitive did not reach {state_key!r} within {timeout_s:.3f}s"
            )
        time.sleep(dt)


def read_external_wrench(robot) -> dict:
    """Return the wrist external wrench as a {fx,fy,fz,mx,my,mz} dict."""
    _, state = selected_arm_state(robot)
    wrench = list(getattr(state, "ext_wrench_in_tcp", []) or [])
    if len(wrench) < 6:
        wrench = wrench + [0.0] * (6 - len(wrench))
    return {
        "fx": float(wrench[0]),
        "fy": float(wrench[1]),
        "fz": float(wrench[2]),
        "mx": float(wrench[3]),
        "my": float(wrench[4]),
        "mz": float(wrench[5]),
    }


def joints_to_jpos_deg(q_rad: Iterable[float]) -> list[float]:
    """Convert a 7-element radians vector to degrees, ready for ``flexivrdk.JPos``."""
    values = [float(q) for q in q_rad]
    if len(values) != 7:
        raise ValueError(f"joints_to_jpos_deg expects 7 joints, got {len(values)}")
    return [math.degrees(q) for q in values]


def rpy_deg_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> list[float]:
    """Inverse of :func:`quat_to_rpy_deg`. Returns ``[qw, qx, qy, qz]`` unit quat.

    Uses the same Tait-Bryan ZYX intrinsic convention so RPY -> quat -> RPY
    round-trips on values produced by ``quat_to_rpy_deg``.
    """
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
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    return [qw / norm, qx / norm, qy / norm, qz / norm]


def quat_to_rpy_deg(qw: float, qx: float, qy: float, qz: float) -> list[float]:
    """Convert a unit quaternion to roll/pitch/yaw in degrees.

    Lifted verbatim from ``project/play_recorded_waypoints.py`` so the pipeline and
    the existing playback script agree on the conversion bit-for-bit.
    """
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


def zero_ft_sensor(session) -> None:
    """Zero the force/torque sensor at the current configuration.

    Wraps the ``ZeroFTSensor`` primitive. The caller is responsible for
    ensuring the arm is in the configuration whose contact state should
    become the new zero baseline — typically called right after the grasp
    so the held part's weight is included in the baseline (subsequent
    ``ext_wrench_in_world`` readings then reflect *additional* contact only).

    Forces the session into ``NRT_PRIMITIVE_EXECUTION`` mode since
    ``ZeroFTSensor`` is a primitive. Does not restore the prior mode.
    """
    session.switch_mode("NRT_PRIMITIVE_EXECUTION")
    session.execute_primitive("ZeroFTSensor", {})
    # ZeroFTSensor is not a motion primitive; the completion flag is
    # ``terminated`` rather than ``reachedTarget``.
    session.wait_for_primitive("terminated")


def descend_until_force_or_depth(
    session,
    *,
    max_depth_m: float,
    force_threshold_n: float,
    vel_m_s: float = 0.005,
    poll_dt_s: float = 0.001,
    max_contact_wrench: list[float] | None = None,
    settle_after_s: float = 0.1,
    logger=None,
) -> dict:
    """Cartesian Motion-Force descent: hardware-regulated soft contact +
    high-frequency software polling for the contact-detection abort.

    Switches the session into ``NRT_CARTESIAN_MOTION_FORCE`` for the descent
    and restores ``NRT_PRIMITIVE_EXECUTION`` on exit (including on error).
    The controller is given a max-contact-wrench limit (defense in depth;
    the arm will soften its motion at that limit even if our software polling
    lags). We poll ``ext_wrench_in_world`` every ``poll_dt_s`` and abort when
    ``|F| = sqrt(fx² + fy² + fz²) > force_threshold_n``; the abort itself is
    a fresh ``SendCartesianMotionForce`` to the current TCP, which decelerates
    the arm and holds it in place.

    Call ``zero_ft_sensor(session)`` first — this helper does NOT zero the
    sensor on its own, so the contact baseline is whatever you choose.

    Returns ``{"reason": "force"|"depth"|"timeout", "fz_at_stop": float, "descent_m": float}``.
    """
    flexivrdk = session.flexivrdk
    robot = session.robot

    _, arm = session.selected_arm_state()
    start_tcp = list(getattr(arm, "tcp_pose", []))
    if len(start_tcp) != 7:
        raise RuntimeError(
            f"arm tcp_pose has {len(start_tcp)} values; expected 7"
        )
    start_z = float(start_tcp[2])

    if max_contact_wrench is None:
        # Hardware-level soft-contact regulation kicks in at 2x the software
        # threshold, with a floor of 10 N. Torques 3 N·m (per vendor example).
        soft = max(float(force_threshold_n) * 2.0, 10.0)
        max_contact_wrench = [soft, soft, soft, 3.0, 3.0, 3.0]

    target_pose = list(start_tcp)
    target_pose[2] = start_z - float(max_depth_m)

    if logger is not None:
        logger.info(
            f"descend: start_z={start_z*1000:.1f}mm "
            f"max_depth={max_depth_m*1000:.0f}mm "
            f"force_threshold={force_threshold_n:.1f}N "
            f"vel={vel_m_s*1000:.1f}mm/s poll={poll_dt_s*1000:.1f}ms"
        )

    session.switch_mode("NRT_CARTESIAN_MOTION_FORCE")
    reason = None
    fz_at_stop = 0.0
    descent_m = 0.0
    try:
        robot.SetMaxContactWrench(list(max_contact_wrench))
        robot.SendCartesianMotionForce(
            target_pose, [0.0] * 6, [0.0] * 6, float(vel_m_s)
        )

        # Safety budget: nominal travel time + 5 s buffer.
        deadline = time.monotonic() + (max_depth_m / max(vel_m_s, 1e-3)) + 5.0
        while True:
            time.sleep(poll_dt_s)
            _, arm = session.selected_arm_state()
            wrench = list(getattr(arm, "ext_wrench_in_world", [0.0] * 6))
            tcp_now = list(getattr(arm, "tcp_pose", start_tcp))
            if len(wrench) >= 3:
                fx, fy, fz = float(wrench[0]), float(wrench[1]), float(wrench[2])
            else:
                fx = fy = fz = 0.0
            force_norm = math.sqrt(fx * fx + fy * fy + fz * fz)
            if len(tcp_now) >= 3:
                descent_m = start_z - float(tcp_now[2])
            else:
                descent_m = 0.0

            if force_norm > force_threshold_n:
                reason = "force"
                fz_at_stop = fz
                # Halt-in-place: new target = current TCP.
                halt_target = list(tcp_now) if len(tcp_now) == 7 else list(start_tcp)
                robot.SendCartesianMotionForce(
                    halt_target, [0.0] * 6, [0.0] * 6, float(vel_m_s)
                )
                break
            if descent_m >= max_depth_m - 1e-4:
                reason = "depth"
                break
            if time.monotonic() > deadline:
                reason = "timeout"
                break

        if logger is not None:
            logger.info(
                f"descend: reason={reason} fz={fz_at_stop:.2f}N "
                f"descent={descent_m*1000:.2f}mm"
            )
        if settle_after_s > 0:
            time.sleep(settle_after_s)
        return {"reason": reason, "fz_at_stop": fz_at_stop, "descent_m": descent_m}
    finally:
        session.switch_mode("NRT_PRIMITIVE_EXECUTION")


def walk_path(
    session,
    path_entry: dict,
    *,
    end_pose: dict | None = None,
    vel_scale: int = 10,
    zone_radius: str = "ZFine",
    target_toler_level: int = 1,
    jnt_acc_multiplier: float = 1.0,
    joint_locked: bool = True,
) -> None:
    """MovePTP through each captured waypoint of ``path_entry`` in order.

    If ``end_pose`` is given, a final MovePTP to it caps the walk. Intermediate
    waypoints default to ``joint_locked=True`` so the arm reproduces the
    operator-demonstrated arm shape exactly (paths typically encode
    collision-safe joint trajectories — letting IK drift could swing the
    elbow through a fixture).
    """
    waypoints = path_entry.get("waypoints", []) or []
    for wp in waypoints:
        move_ptp_joint(
            session,
            wp,
            vel_scale=vel_scale,
            zone_radius=zone_radius,
            target_toler_level=target_toler_level,
            jnt_acc_multiplier=jnt_acc_multiplier,
            joint_locked=joint_locked,
        )
    if end_pose is not None:
        move_ptp_joint(
            session,
            end_pose,
            vel_scale=vel_scale,
            zone_radius=zone_radius,
            target_toler_level=target_toler_level,
            jnt_acc_multiplier=jnt_acc_multiplier,
            joint_locked=joint_locked,
        )


def gripper_set(
    gripper,
    width_m: float,
    *,
    vel_m_s: float = 0.05,
    force_n: float = 10.0,
    wait: bool = True,
    timeout_s: float = 15.0,
    poll_dt_s: float = 0.1,
    settle_after_s: float = 0.0,
) -> None:
    """Dispatch ``gripper.Move(width, vel, force)`` and (by default) wait until
    the gripper reports it's no longer moving.

    The timeout is forgiving — when the gripper jams against a part the
    controller flags ``is_moving=False`` once the force limit is hit, so the
    poll naturally exits. The timeout only matters if the gripper hangs.

    ``settle_after_s`` is an extra sleep applied *after* the poll completes.
    Useful for force-controlled closes where ``is_moving`` clears the instant
    the controller registers stall, before the grip has actually steadied.
    Default 0 (no extra wait).
    """
    gripper.Move(float(width_m), float(vel_m_s), float(force_n))
    if not wait:
        return
    deadline = time.monotonic() + max(0.0, timeout_s)
    # Give the controller a tick to register the new motion before polling.
    time.sleep(poll_dt_s)
    while True:
        try:
            moving = bool(gripper.states().is_moving)
        except Exception:
            moving = False
        if not moving:
            break
        if time.monotonic() >= deadline:
            break
        time.sleep(poll_dt_s)
    if settle_after_s > 0:
        time.sleep(settle_after_s)


def _quat_normalize(quat):
    qw, qx, qy, qz = [float(v) for v in quat]
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm == 0.0:
        raise ValueError("zero quaternion")
    return [qw / norm, qx / norm, qy / norm, qz / norm]


def _quat_multiply(a, b):
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


def _quat_to_matrix(quat):
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


def _mat_vec_mul(matrix, vec):
    return [
        sum(matrix[row][col] * vec[col] for col in range(3)) for row in range(3)
    ]


def rotate_tcp_about_virtual_point(
    tcp_pose,
    offset_cm: float,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> list[float]:
    """Return TCP pose ``[x, y, z, qw, qx, qy, qz]`` after a local RPY rotation
    about a virtual pivot at ``offset_cm`` along the EE's local +X axis.

    Ported from ``project/record_robot_waypoints.py:rotate_about_virtual_point``
    — same math, lifted here so it's reusable. Used by ``wiggle_about_virtual_point``.
    """
    pose = [float(v) for v in tcp_pose]
    if len(pose) != 7:
        raise ValueError(f"tcp_pose must have 7 values, got {len(pose)}")
    position = pose[:3]
    current_quat = _quat_normalize(pose[3:])
    offset = [float(offset_cm) / 100.0, 0.0, 0.0]
    current_matrix = _quat_to_matrix(current_quat)
    co = _mat_vec_mul(current_matrix, offset)
    pivot_world = [position[i] + co[i] for i in range(3)]
    delta_quat = rpy_deg_to_quat(roll_deg, pitch_deg, yaw_deg)
    target_quat = _quat_multiply(current_quat, delta_quat)
    target_matrix = _quat_to_matrix(target_quat)
    to = _mat_vec_mul(target_matrix, offset)
    target_position = [pivot_world[i] - to[i] for i in range(3)]
    return target_position + list(target_quat)


def wiggle_about_virtual_point(
    session,
    *,
    pivot_offset_cm: float = 0.0,
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
    yaw_deg: float = 5.0,
    repeat_count: int = 3,
    vel_scale: int = 10,
    zone_radius: str = "ZFine",
) -> None:
    """Oscillate the TCP about a virtual pivot, ending back at the start pose.

    Reads the current TCP, generates RPY keyframes
    ``[0, +Δ, -Δ, +Δ, -Δ, ..., 0]`` (``repeat_count`` ± cycles, then back to
    zero), and dispatches a ``MovePTP`` for each transition. Each MovePTP uses
    the captured joint state as an IK seed (``enableFixRefJntPos=False``) so
    the arm stays close to the original configuration.

    With ``pivot_offset_cm = 0`` the wiggle is pure rotation about the current
    TCP origin (useful for shaking a gripped part in place). With a positive
    offset, the rotation pivots at a point along the EE's local +X axis —
    e.g. set offset to a few cm to swing the gripper tip around a point near
    the part's center of mass.
    """
    flexivrdk = session.flexivrdk
    _, arm = session.selected_arm_state()
    start_tcp = list(getattr(arm, "tcp_pose", []))
    if len(start_tcp) != 7:
        raise RuntimeError(
            f"tcp_pose has {len(start_tcp)} values; expected 7"
        )
    seed_q_deg = joints_to_jpos_deg(
        [float(v) for v in getattr(arm, "q", [])]
    )

    zero = (0.0, 0.0, 0.0)
    positive = (float(roll_deg), float(pitch_deg), float(yaw_deg))
    keyframes: list[tuple[float, float, float]] = [zero]
    for _ in range(int(repeat_count)):
        keyframes.append(positive)
        keyframes.append(tuple(-v for v in positive))
    keyframes.append(zero)

    # Execute MovePTP between consecutive keyframes; we always rotate relative
    # to the *start* TCP, so accumulated drift can't grow across iterations.
    for r, p, y in keyframes[1:]:
        target_tcp = rotate_tcp_about_virtual_point(
            start_tcp, pivot_offset_cm, r, p, y
        )
        coord_args = tcp_pose_to_coord_args(target_tcp, ref_joints_deg=seed_q_deg)
        session.execute_primitive(
            "MovePTP",
            {
                "target": flexivrdk.Coord(*coord_args),
                "jntVelScale": int(vel_scale),
                "zoneRadius": zone_radius,
                "targetTolerLevel": 1,
                "jntAccMultiplier": 1.0,
                "enableFixRefJntPos": False,
                "refJntPos": flexivrdk.JPos(seed_q_deg),
            },
        )
        session.wait_for_primitive("reachedTarget")


def pose_offset_z(pose_entry: dict, dz_m: float) -> dict:
    """Return a copy of ``pose_entry`` with TCP Z shifted by ``dz_m`` (meters).

    Useful for synthesizing an "above" anchor pose from a captured target
    without a separate YAML entry. The captured joint angles are passed
    through as the IK seed; callers should pass ``joint_locked=False`` to
    ``move_ptp_joint`` so IK finds joints near the seed for the offset TCP.
    """
    tcp_order = list(pose_entry["tcp_pose_world"]["order"])
    tcp_vals = list(pose_entry["tcp_pose_world"]["values"])
    tcp_vals[2] = float(tcp_vals[2]) + float(dz_m)
    return {
        "q_rad": list(pose_entry["q_rad"]),
        "tcp_pose_world": {"order": tcp_order, "values": tcp_vals},
    }


def move_z_relative(
    session,
    delta_m: float,
    *,
    vel_m_s: float = 0.05,
    zone_radius: str = "ZFine",
) -> None:
    """Cartesian-linear translation along world Z by ``delta_m`` meters.

    Reads the current TCP via ``session.selected_arm_state()``, builds a
    target Coord identical to it but with ``z += delta_m``, and dispatches a
    ``MoveL`` primitive in the WORLD frame. Orientation is held constant —
    this is the canonical "drop straight down" primitive used at the tray
    (pre-grasp -> grasp) and at the vise (above_vise -> slot insert).

    ``delta_m`` is signed: negative = descend, positive = ascend.

    ``vel_m_s`` is the TCP linear velocity. Defaults to 0.05 m/s (5 cm/s) for
    conservative behavior; insert / approach phases typically run slower.
    """
    flexivrdk = session.flexivrdk
    _, state = session.selected_arm_state()
    tcp = list(getattr(state, "tcp_pose", []))
    if len(tcp) != 7:
        raise RuntimeError(
            f"arm tcp_pose has {len(tcp)} values; expected 7 (x,y,z,qw,qx,qy,qz)"
        )
    target_tcp = list(tcp)
    target_tcp[2] = target_tcp[2] + delta_m
    coord_args = tcp_pose_to_coord_args(target_tcp)
    session.execute_primitive(
        "MoveL",
        {
            "target": flexivrdk.Coord(*coord_args),
            "vel": float(vel_m_s),
            "zoneRadius": zone_radius,
        },
    )
    session.wait_for_primitive("reachedTarget")


def move_ptp_joint(
    session,
    pose_entry: dict,
    *,
    vel_scale: int = 10,
    zone_radius: str = "ZFine",
    target_toler_level: int = 1,
    jnt_acc_multiplier: float = 1.0,
    joint_locked: bool = False,
) -> None:
    """MovePTP to a captured pose with the captured joints as an IK seed.

    Reads ``q_rad`` and ``tcp_pose_world`` from ``pose_entry``. The TCP pose
    is the target; the joint angles are passed as ``refJntPos`` so IK picks a
    configuration near the operator's demonstrated arm shape — that lets
    paper tunes to ``tcp_pose_world`` (via ``tune_pose.py``) actually move
    the arm to the new location.

    If ``joint_locked=True``, ``enableFixRefJntPos`` is set and the motion
    reaches ``refJntPos`` exactly, ignoring TCP-only edits. Useful for strict
    replay of captured arm configurations (e.g. ``replay_phase.py``); not
    what you want during an iterative tuning loop.

    Blocks on ``reachedTarget`` before returning.
    """
    flexivrdk = session.flexivrdk
    tcp = pose_entry["tcp_pose_world"]["values"]
    joints_deg = joints_to_jpos_deg(pose_entry["q_rad"])
    coord_args = tcp_pose_to_coord_args(tcp, ref_joints_deg=joints_deg)
    session.execute_primitive(
        "MovePTP",
        {
            "target": flexivrdk.Coord(*coord_args),
            "jntVelScale": int(vel_scale),
            "zoneRadius": zone_radius,
            "targetTolerLevel": int(target_toler_level),
            "jntAccMultiplier": float(jnt_acc_multiplier),
            "enableFixRefJntPos": bool(joint_locked),
            "refJntPos": flexivrdk.JPos(joints_deg),
        },
    )
    session.wait_for_primitive("reachedTarget")


def tcp_pose_to_coord_args(
    tcp_pose: Iterable[float],
    ref_frame: tuple[str, str] = ("WORLD", "WORLD_ORIGIN"),
    ref_joints_deg: Iterable[float] | None = None,
    ref_external: Iterable[float] | None = None,
) -> tuple:
    """Build the 5 positional arguments for ``flexivrdk.Coord``.

    ``tcp_pose`` is the 7-vector ``[x, y, z, qw, qx, qy, qz]`` in meters/quaternion.
    Position is passed through unchanged; orientation is converted to roll/pitch/yaw
    degrees via ``quat_to_rpy_deg``. ``ref_joints_deg`` defaults to a 7-element
    zero vector (matches the RDK's own default IK seed) and ``ref_external``
    defaults to a 6-element zero vector (the binding enforces FixedSize(6) on
    that argument; an empty list raises TypeError).
    """
    pose = [float(v) for v in tcp_pose]
    if len(pose) != 7:
        raise ValueError(f"tcp_pose must have 7 values, got {len(pose)}")
    position = pose[:3]
    qw, qx, qy, qz = pose[3:]
    orientation = quat_to_rpy_deg(qw, qx, qy, qz)
    if ref_joints_deg is None:
        ref_q_m = [0.0] * 7
    else:
        ref_q_m = [float(v) for v in ref_joints_deg]
        if len(ref_q_m) != 7:
            raise ValueError(
                f"ref_joints_deg must have 7 values, got {len(ref_q_m)}"
            )
    if ref_external is None:
        ref_q_e = [0.0] * 6
    else:
        ref_q_e = [float(v) for v in ref_external]
        if len(ref_q_e) != 6:
            raise ValueError(
                f"ref_external must have 6 values, got {len(ref_q_e)}"
            )
    return (position, orientation, list(ref_frame), ref_q_m, ref_q_e)


class RobotSession:
    """Context manager around ``flexivrdk.Robot`` plus gripper convenience.

    Construct, enter the ``with`` block to enable and reach ``operational()``,
    exit to ``Stop()`` the robot. Exit never raises; any cleanup exceptions are
    logged best-effort.
    """

    GRIPPER_INIT_WAIT_S = 4.0
    DEFAULT_OPERATIONAL_TIMEOUT_S = 30.0

    def __init__(
        self,
        robot_sn: str,
        logger=None,
        *,
        operational_timeout_s: float = DEFAULT_OPERATIONAL_TIMEOUT_S,
        flexivrdk_module=None,
    ) -> None:
        self._robot_sn = robot_sn
        self._logger = logger
        self._operational_timeout_s = operational_timeout_s
        self._flexivrdk = flexivrdk_module
        self._robot = None
        self._gripper = None
        self._tool = None

    # ----- lifecycle -----

    def __enter__(self) -> "RobotSession":
        flexivrdk = self._flexivrdk or _load_flexivrdk()
        self._flexivrdk = flexivrdk
        self._info(f"Connecting to robot {self._robot_sn}")
        robot = flexivrdk.Robot(self._robot_sn)

        if robot.fault():
            self._warn("Fault detected on robot, trying to clear")
            if not robot.ClearFault():
                raise RuntimeError("Failed to clear fault on robot")
            self._info("Fault cleared")

        robot.Enable()
        deadline = time.monotonic() + self._operational_timeout_s
        while not robot.operational():
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"Robot did not become operational within "
                    f"{self._operational_timeout_s:.1f}s"
                )
            time.sleep(0.5)
        self._info("Robot operational")
        if hasattr(robot, "groups") and hasattr(flexivrdk, "PrimitiveArgs"):
            self._info("RDK API surface: joint-group (v2.0 path)")
        else:
            self._info("RDK API surface: single-group (v1.9 path)")
        self._robot = robot
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._robot is None:
            return
        try:
            self._robot.Stop()
        except Exception as err:  # noqa: BLE001 - best-effort cleanup
            self._warn(f"robot.Stop() during shutdown raised: {err}")

    # ----- accessors -----

    @property
    def robot(self):
        if self._robot is None:
            raise RuntimeError("RobotSession is not entered; call inside `with`")
        return self._robot

    @property
    def gripper(self):
        return self._gripper

    @property
    def flexivrdk(self):
        return self._flexivrdk

    # ----- gripper -----

    def setup_gripper(self, gripper_name: str, init: bool = True):
        flexivrdk = self._flexivrdk or _load_flexivrdk()
        gripper = flexivrdk.Gripper(self.robot)
        tool = flexivrdk.Tool(self.robot)
        self._info(f"Enabling gripper [{gripper_name}]")
        gripper.Enable(gripper_name)
        self._info(f"Switching tool to [{gripper_name}]")
        tool.Switch(gripper_name)
        if init:
            self._info("Initializing gripper ...")
            gripper.Init()
            time.sleep(self.GRIPPER_INIT_WAIT_S)
        self._gripper = gripper
        self._tool = tool
        return gripper

    # ----- thin pass-throughs -----

    def selected_arm_state(self) -> tuple[str | None, Any]:
        return selected_arm_state(self.robot)

    def execute_primitive(self, name: str, params: dict) -> None:
        execute_primitive(self.robot, name, params, flexivrdk_module=self._flexivrdk)

    def wait_for_primitive(
        self,
        state_key: str = "reachedTarget",
        dt: float = 0.2,
        timeout_s: float | None = None,
    ) -> None:
        wait_for_primitive(self.robot, state_key=state_key, dt=dt, timeout_s=timeout_s)

    def switch_mode(self, mode_name: str) -> None:
        mode = rdk_mode(mode_name, flexivrdk_module=self._flexivrdk)
        self.robot.SwitchMode(mode)

    def set_cartesian_impedance(
        self,
        kx: float,
        ky: float,
        kz: float,
        k_rot: float,
        damping_ratio: list[float] | None = None,
    ) -> None:
        """Set the Cartesian impedance stiffness and optional damping ratio.

        Stiffness vector ordering follows the v1.9 SDK convention:
        ``[kx, ky, kz, krx, kry, krz]``. ``damping_ratio`` is the optional
        per-axis ``Z_x`` vector (same length 6); when ``None`` the SDK default
        is used.
        """
        stiffness = [float(kx), float(ky), float(kz),
                     float(k_rot), float(k_rot), float(k_rot)]
        for value in stiffness:
            if value < 0:
                raise ValueError(f"Stiffness components must be non-negative: {stiffness}")
        if damping_ratio is not None:
            damping_ratio = [float(z) for z in damping_ratio]
            if len(damping_ratio) != 6:
                raise ValueError(
                    f"damping_ratio must have length 6, got {len(damping_ratio)}"
                )
            for z in damping_ratio:
                if z < 0:
                    raise ValueError(
                        f"damping_ratio components must be non-negative: {damping_ratio}"
                    )
        setter = getattr(self.robot, "SetCartesianImpedance", None)
        if not callable(setter):
            raise RuntimeError(
                "flexivrdk.Robot has no SetCartesianImpedance method. "
                "Update flexiv_helpers.set_cartesian_impedance for this RDK build."
            )
        if damping_ratio is None:
            setter(stiffness)
        else:
            setter(stiffness, damping_ratio)

    def read_external_wrench(self) -> dict:
        return read_external_wrench(self.robot)

    # ----- logging helpers -----

    def _info(self, msg: str) -> None:
        if self._logger is not None and hasattr(self._logger, "info"):
            self._logger.info(msg)

    def _warn(self, msg: str) -> None:
        if self._logger is not None and hasattr(self._logger, "warn"):
            self._logger.warn(msg)
        elif self._logger is not None and hasattr(self._logger, "warning"):
            self._logger.warning(msg)
