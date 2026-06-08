"""Robot + gripper control and live status, reusing project/helper/flexiv_helpers.

Owns one ``RobotSession`` for the process lifetime. Status reads are lock-free
(``robot.states()`` is a cheap snapshot); mutating ops go through the shared
``OperationExecutor`` so only one motion runs at a time.
"""

from __future__ import annotations

import json
import math

import config

from helper.flexiv_helpers import (
    RobotSession,
    joints_to_jpos_deg,
    move_ptp_joint,
    quat_to_rpy_deg,
    read_external_wrench,
    rotate_tcp_about_tool_axis,
    tcp_pose_to_coord_args,
    zero_ft_sensor,
)

# Jog axis -> how it maps onto a TCP move. x/y/z translate in WORLD; rx/ry/rz
# rotate about the tool frame (roll/pitch/yaw).
_JOG_TRANSLATION = {"x": 0, "y": 1, "z": 2}
_JOG_ROTATION = {"rx": "roll_deg", "ry": "pitch_deg", "rz": "yaw_deg"}

# FloatingJoint primitive defaults (mirror record_robot_waypoints.py).
_FLOAT_RESPONSE_TORQUE = [1.5, 1.5, 1.5, 1.5, 0.5, 0.5, 0.3]


class RobotController:
    def __init__(self, logger, executor) -> None:
        self._logger = logger
        self._executor = executor
        self._session_cm: RobotSession | None = None
        self.session: RobotSession | None = None
        self.gripper = None
        self._floating = {"mode": None, "selection": []}
        self._last_gripper_cmd = "unknown"

    # ----- lifecycle -----

    def connect(self) -> None:
        self._session_cm = RobotSession(config.ROBOT_SN, logger=self._logger)
        self.session = self._session_cm.__enter__()
        self.gripper = self.session.setup_gripper(
            config.GRIPPER_NAME, init=config.GRIPPER_INIT
        )

    def close(self) -> None:
        if self._session_cm is not None:
            self._session_cm.__exit__(None, None, None)
            self._session_cm = None
            self.session = None

    # ----- status (lock-free) -----

    def status(self) -> dict:
        robot = self.session.robot
        _, state = self.session.selected_arm_state()
        q_rad = [float(v) for v in getattr(state, "q", [])]
        tcp = [float(v) for v in getattr(state, "tcp_pose", [])]

        position = tcp[:3] if len(tcp) >= 3 else [0.0, 0.0, 0.0]
        if len(tcp) == 7:
            rpy = quat_to_rpy_deg(tcp[3], tcp[4], tcp[5], tcp[6])
        else:
            rpy = [0.0, 0.0, 0.0]

        gripper_status = {"width_m": None, "force_n": None, "is_moving": None}
        try:
            gs = self.gripper.states()
            gripper_status = {
                "name": config.GRIPPER_NAME,
                "width_m": float(gs.width),
                "force_n": float(gs.force),
                "is_moving": bool(gs.is_moving),
                "last_command": self._last_gripper_cmd,
            }
        except Exception:  # noqa: BLE001 - gripper read is best-effort
            pass

        return {
            "connected": True,
            "fault": bool(robot.fault()),
            "operational": bool(robot.operational()),
            "joint_angles_deg": [math.degrees(v) for v in q_rad],
            "joint_angles_rad": q_rad,
            "tcp_position_m": position,
            "tcp_orientation_deg": rpy,
            "wrench": read_external_wrench(robot),
            "gripper": gripper_status,
            "floating": dict(self._floating),
        }

    # ----- saved waypoints -----

    def list_waypoints(self) -> list[str]:
        index = config.KEY_POSITIONS_DIR / "index.json"
        if index.exists():
            data = json.loads(index.read_text())
            return sorted(data.get("positions", {}).keys())
        return sorted(p.stem for p in config.KEY_POSITIONS_DIR.glob("*.json")
                      if p.name != "index.json")

    def _load_pose_entry(self, name: str) -> dict:
        index = config.KEY_POSITIONS_DIR / "index.json"
        filename = f"{name}.json"
        if index.exists():
            positions = json.loads(index.read_text()).get("positions", {})
            entry = positions.get(name)
            if entry and entry.get("file"):
                filename = entry["file"]
        path = config.KEY_POSITIONS_DIR / filename
        data = json.loads(path.read_text())
        # move_ptp_joint expects {"q_rad", "tcp_pose_world"}; waypoint files store
        # joint angles under "joint_angles_rad".
        return {
            "q_rad": data["joint_angles_rad"],
            "tcp_pose_world": data["tcp_pose_world"],
        }

    # ----- control ops (via executor) -----

    def home(self) -> bool:
        def op():
            self.session.switch_mode("NRT_PRIMITIVE_EXECUTION")
            self.session.execute_primitive("Home", {})
            self.session.wait_for_primitive("reachedTarget", timeout_s=60.0)
            self._floating = {"mode": None, "selection": []}
        return self._executor.submit("robot.home", op)

    def move_to(self, name: str) -> bool:
        entry = self._load_pose_entry(name)

        def op():
            self.session.switch_mode("NRT_PRIMITIVE_EXECUTION")
            move_ptp_joint(self.session, entry, vel_scale=config.MOVE_JNT_VEL_SCALE)
            self._floating = {"mode": None, "selection": []}
        return self._executor.submit(f"robot.move_to:{name}", op)

    def jog(self, axis: str, delta: float) -> bool:
        """Incremental TCP move. delta is mm for x/y/z, degrees for rx/ry/rz."""
        axis = str(axis).lower()
        if axis not in _JOG_TRANSLATION and axis not in _JOG_ROTATION:
            raise ValueError(f"unknown jog axis: {axis}")

        def op():
            flexivrdk = self.session.flexivrdk
            _, state = self.session.selected_arm_state()
            tcp = [float(v) for v in getattr(state, "tcp_pose", [])]
            if len(tcp) != 7:
                raise RuntimeError("TCP pose unavailable for jog")
            if axis in _JOG_TRANSLATION:
                target = list(tcp)
                target[_JOG_TRANSLATION[axis]] += float(delta) / 1000.0  # mm -> m
            else:
                target = rotate_tcp_about_tool_axis(
                    tcp, **{_JOG_ROTATION[axis]: float(delta)}
                )
            coord_args = tcp_pose_to_coord_args(target)
            self.session.switch_mode("NRT_PRIMITIVE_EXECUTION")
            self.session.execute_primitive("MoveL", {
                "target": flexivrdk.Coord(*coord_args),
                "vel": config.ROBOT_JOG_VEL_M_S,
                "zoneRadius": "ZFine",
            })
            self.session.wait_for_primitive("reachedTarget", timeout_s=30.0)
            self._floating = {"mode": None, "selection": []}
        return self._executor.submit(f"robot.jog:{axis}", op)

    def gripper_set(self, action: str) -> bool:
        from helper.flexiv_helpers import gripper_set as _gripper_set

        if action == "open":
            width, force = config.GRIPPER_OPEN_WIDTH_M, config.GRIPPER_OPEN_FORCE_N
        else:
            width, force = config.GRIPPER_CLOSE_WIDTH_M, config.GRIPPER_CLOSE_FORCE_N

        def op():
            _gripper_set(
                self.gripper, width,
                vel_m_s=config.GRIPPER_VELOCITY_M_S, force_n=force,
            )
            self._last_gripper_cmd = action
        return self._executor.submit(f"robot.gripper:{action}", op)

    def floating(self, mode: str, on: bool, selection=None) -> bool:
        selection = selection or []

        def op():
            if not on:
                self.session.hold_current_joints()
                self._floating = {"mode": None, "selection": []}
                return
            if mode == "cartesian":
                self.session.start_cartesian_floating(selection or None)
            else:  # joint floating
                dof = len(self.session.selected_arm_state()[1].q)
                if selection:
                    mask = [1.0 if str(i + 1) in map(str, selection) else 0.0
                            for i in range(dof)]
                else:
                    mask = [1.0] * dof
                self.session.robot.Stop()
                self.session.switch_mode("NRT_PRIMITIVE_EXECUTION")
                self.session.execute_primitive("FloatingJoint", {
                    "floatingJoint": mask,
                    "dampingLevel": [0.0] * dof,
                    "responseTorque": _FLOAT_RESPONSE_TORQUE[:dof],
                    "diEnableFloating": "NONE",
                })
            self._floating = {"mode": mode, "selection": selection}
        return self._executor.submit(f"robot.floating:{mode}:{'on' if on else 'off'}", op)

    def zero_ft(self) -> bool:
        return self._executor.submit("robot.zero_ft", lambda: zero_ft_sensor(self.session))

    def clear_fault(self) -> dict:
        # Quick, non-motion: run inline rather than through the executor.
        ok = bool(self.session.robot.ClearFault())
        return {"cleared": ok}

    def stop(self) -> dict:
        self._executor.request_stop()
        self.session.robot.Stop()
        return {"stopped": True}
