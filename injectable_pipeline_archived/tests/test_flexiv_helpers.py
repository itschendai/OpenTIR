"""Unit tests for flexiv_helpers.

The Flexiv RDK is not available on CI, so the tests use a faked module that mirrors
the relevant slices of the v2 API: ``Robot``, ``Gripper``, ``Tool``, ``Mode``,
``PrimitiveArgs``, and a few state container classes.
"""

from __future__ import annotations

import sys
import types

import pytest

import flexiv_helpers as fh


# ----------------- fake flexivrdk module -----------------


class _FakeJointGroup:
    def __init__(self, name: str) -> None:
        self._name = name

    def __repr__(self) -> str:  # used by str() inside helpers
        return self._name

    def __hash__(self) -> int:
        return hash(self._name)

    def __eq__(self, other) -> bool:
        return isinstance(other, _FakeJointGroup) and self._name == other._name


class _FakeArmState:
    def __init__(self, q=None, tcp_pose=None, ext_wrench=None) -> None:
        self.q = q if q is not None else [0.0] * 7
        self.tcp_pose = tcp_pose if tcp_pose is not None else [0.0] * 7
        self.ext_wrench_in_tcp = ext_wrench if ext_wrench is not None else [0.0] * 6


class _FakePrimitiveState:
    def __init__(self, values: dict) -> None:
        self.names_and_values = values


class _FakePrimitiveArgs:
    def __init__(self, name: str, params: dict) -> None:
        self.name = name
        self.params = params

    def __repr__(self) -> str:
        return f"PrimitiveArgs({self.name!r}, {self.params!r})"


class _FakeMode:
    """Stand-in for ``flexivrdk.Mode``: named attribute access + int-by-construction."""

    NRT_PRIMITIVE_EXECUTION = "M_NRT_PRIMITIVE_EXECUTION"
    NRT_JOINT_IMPEDANCE = "M_NRT_JOINT_IMPEDANCE"

    def __init__(self, value: int) -> None:  # imitate flexivrdk.Mode(int)
        self.value = value

    def __repr__(self) -> str:
        return f"_FakeMode({self.value})"


class FakeRobot:
    """Test double matching the slice of ``flexivrdk.Robot`` the helpers touch."""

    def __init__(
        self,
        robot_sn: str | None = None,
        *,
        groups: list[str] | None = None,
        initial_fault: bool = False,
        clear_fault_succeeds: bool = True,
        operational_after: int = 1,
    ) -> None:
        self.robot_sn = robot_sn
        self._groups = [_FakeJointGroup(g) for g in (groups or ["ARM"])]
        self._fault = initial_fault
        self._clear_succeeds = clear_fault_succeeds
        self.cleared_faults = 0
        self.enabled = False
        self._operational_after = operational_after
        self._operational_polls = 0
        self.stopped = False
        self.last_primitive: dict | None = None
        self.last_mode = None
        self.last_stiffness: list[float] | None = None
        self.last_damping_ratio: list[float] | None = None
        self._states_payload: dict[_FakeJointGroup, _FakeArmState] = {
            self._groups[0]: _FakeArmState()
        }
        self._primitive_payloads: list[dict[_FakeJointGroup, _FakePrimitiveState]] = [
            {self._groups[0]: _FakePrimitiveState({"reachedTarget": True})}
        ]
        self._primitive_call_index = 0
        self._switch_mode_calls: list[Any] = []

    # API the helpers exercise

    def fault(self) -> bool:
        return self._fault

    def ClearFault(self) -> bool:
        self.cleared_faults += 1
        if self._clear_succeeds:
            self._fault = False
            return True
        return False

    def Enable(self) -> None:
        self.enabled = True

    def operational(self) -> bool:
        self._operational_polls += 1
        return self._operational_polls >= self._operational_after

    def Stop(self) -> None:
        self.stopped = True

    def groups(self):
        return list(self._groups)

    def ExecutePrimitive(self, payload):
        self.last_primitive = payload

    def primitive_states(self):
        if self._primitive_call_index < len(self._primitive_payloads):
            payload = self._primitive_payloads[self._primitive_call_index]
            self._primitive_call_index += 1
            return payload
        return self._primitive_payloads[-1]

    def states(self):
        return self._states_payload

    def SwitchMode(self, mode) -> None:
        self.last_mode = mode
        self._switch_mode_calls.append(mode)

    def SetCartesianImpedance(self, stiffness, damping_ratio=None) -> None:
        self.last_stiffness = list(stiffness)
        self.last_damping_ratio = (
            list(damping_ratio) if damping_ratio is not None else None
        )

    # test helpers
    def set_primitive_state_sequence(self, sequence):
        self._primitive_payloads = sequence
        self._primitive_call_index = 0

    def set_states_payload(self, payload):
        self._states_payload = payload


class FakeFlexivRdk(types.ModuleType):
    """Drop-in replacement for ``flexivrdk`` exposing just what the helpers need."""

    def __init__(self) -> None:
        super().__init__("flexivrdk")
        self.Mode = _FakeMode
        self.PrimitiveArgs = _FakePrimitiveArgs
        self.Robot = FakeRobot
        self.Gripper = _FakeGripper
        self.Tool = _FakeTool


class _FakeGripper:
    def __init__(self, robot):
        self.robot = robot
        self.enabled_name = None
        self.initialized = False

    def Enable(self, name):
        self.enabled_name = name

    def Init(self):
        self.initialized = True


class _FakeTool:
    def __init__(self, robot):
        self.robot = robot
        self.switched_name = None

    def Switch(self, name):
        self.switched_name = name


from typing import Any  # noqa: E402  (used in FakeRobot annotation)


@pytest.fixture
def fake_flexivrdk(monkeypatch):
    module = FakeFlexivRdk()
    monkeypatch.setitem(sys.modules, "flexivrdk", module)
    return module


# ----------------- rdk_mode -----------------


def test_rdk_mode_uses_attribute_when_present(fake_flexivrdk):
    assert fh.rdk_mode("NRT_PRIMITIVE_EXECUTION", fake_flexivrdk) == "M_NRT_PRIMITIVE_EXECUTION"


def test_rdk_mode_falls_back_to_integer_map(fake_flexivrdk):
    # Remove the attribute so the integer fallback is exercised.
    class _Mode:
        def __init__(self, value):
            self.value = value

    fake_flexivrdk.Mode = _Mode
    result = fh.rdk_mode("NRT_PRIMITIVE_EXECUTION", fake_flexivrdk)
    assert isinstance(result, _Mode)
    assert result.value == fh.MODE_VALUES["NRT_PRIMITIVE_EXECUTION"]


def test_rdk_mode_unknown_raises(fake_flexivrdk):
    fake_flexivrdk.Mode = type("Empty", (), {})
    with pytest.raises(RuntimeError):
        fh.rdk_mode("DOES_NOT_EXIST", fake_flexivrdk)


# ----------------- selected_arm_state -----------------


def test_selected_arm_state_picks_group_with_arm_substring():
    robot = FakeRobot(groups=["EXT", "ARM_RIGHT"])
    arm_state = _FakeArmState(q=[1.0] * 7)
    robot.set_states_payload(
        {robot._groups[0]: _FakeArmState(), robot._groups[1]: arm_state}
    )
    label, state = fh.selected_arm_state(robot)
    assert label == "ARM_RIGHT"
    assert state is arm_state


def test_selected_arm_state_falls_back_to_first_group():
    robot = FakeRobot(groups=["LEFT_GROUP", "RIGHT_GROUP"])
    first = _FakeArmState()
    robot.set_states_payload(
        {robot._groups[0]: first, robot._groups[1]: _FakeArmState()}
    )
    label, state = fh.selected_arm_state(robot)
    assert label == "LEFT_GROUP"
    assert state is first


# ----------------- execute_primitive -----------------


def test_execute_primitive_uses_joint_group_api_when_available(fake_flexivrdk):
    robot = FakeRobot(groups=["ARM"])
    fh.execute_primitive(robot, "MovePTP", {"foo": 1}, flexivrdk_module=fake_flexivrdk)
    assert isinstance(robot.last_primitive, dict)
    assert len(robot.last_primitive) == 1
    only_value = next(iter(robot.last_primitive.values()))
    assert isinstance(only_value, _FakePrimitiveArgs)
    assert only_value.name == "MovePTP"
    assert only_value.params == {"foo": 1}


def test_execute_primitive_legacy_fallback_when_no_groups(fake_flexivrdk):
    class LegacyRobot:
        def __init__(self):
            self.calls = []

        def ExecutePrimitive(self, name, params):  # legacy form
            self.calls.append((name, params))

    robot = LegacyRobot()
    fh.execute_primitive(robot, "Home", {"a": 1}, flexivrdk_module=fake_flexivrdk)
    assert robot.calls == [("Home", {"a": 1})]


# ----------------- wait_for_primitive -----------------


def test_wait_for_primitive_returns_when_state_truthy(fake_flexivrdk):
    robot = FakeRobot()
    robot.set_primitive_state_sequence(
        [
            {robot._groups[0]: _FakePrimitiveState({"reachedTarget": False})},
            {robot._groups[0]: _FakePrimitiveState({"reachedTarget": False})},
            {robot._groups[0]: _FakePrimitiveState({"reachedTarget": True})},
        ]
    )
    fh.wait_for_primitive(robot, dt=0.001)


def test_wait_for_primitive_times_out(fake_flexivrdk):
    robot = FakeRobot()
    robot.set_primitive_state_sequence(
        [{robot._groups[0]: _FakePrimitiveState({"reachedTarget": False})}]
    )
    with pytest.raises(TimeoutError):
        fh.wait_for_primitive(robot, dt=0.005, timeout_s=0.05)


# ----------------- read_external_wrench -----------------


def test_read_external_wrench_returns_six_components():
    robot = FakeRobot()
    robot.set_states_payload(
        {robot._groups[0]: _FakeArmState(ext_wrench=[1.0, -2.0, 9.81, 0.1, 0.2, 0.3])}
    )
    wrench = fh.read_external_wrench(robot)
    assert wrench == {
        "fx": 1.0, "fy": -2.0, "fz": 9.81,
        "mx": 0.1, "my": 0.2, "mz": 0.3,
    }


def test_read_external_wrench_pads_short_vector():
    robot = FakeRobot()
    robot.set_states_payload({robot._groups[0]: _FakeArmState(ext_wrench=[1.0, 2.0])})
    wrench = fh.read_external_wrench(robot)
    assert wrench["fx"] == 1.0
    assert wrench["fy"] == 2.0
    assert wrench["fz"] == 0.0
    assert wrench["mx"] == 0.0


# ----------------- RobotSession lifecycle -----------------


def test_robot_session_enter_handles_no_initial_fault(fake_flexivrdk):
    with fh.RobotSession("Rizon4-062930", flexivrdk_module=fake_flexivrdk) as session:
        assert session.robot.enabled is True
        assert session.robot.cleared_faults == 0


def test_robot_session_enter_clears_fault_if_present(fake_flexivrdk):
    fake_flexivrdk.Robot = lambda sn: FakeRobot(sn, initial_fault=True)
    with fh.RobotSession("Rizon4-062930", flexivrdk_module=fake_flexivrdk) as session:
        assert session.robot.cleared_faults == 1


def test_robot_session_enter_raises_if_clear_fault_fails(fake_flexivrdk):
    fake_flexivrdk.Robot = lambda sn: FakeRobot(
        sn, initial_fault=True, clear_fault_succeeds=False
    )
    with pytest.raises(RuntimeError):
        with fh.RobotSession("Rizon4-062930", flexivrdk_module=fake_flexivrdk):
            pass


def test_robot_session_enter_waits_until_operational(fake_flexivrdk):
    fake_flexivrdk.Robot = lambda sn: FakeRobot(sn, operational_after=3)
    with fh.RobotSession(
        "Rizon4-062930", flexivrdk_module=fake_flexivrdk, operational_timeout_s=5.0
    ) as session:
        assert session.robot._operational_polls >= 3


def test_robot_session_enter_times_out_when_not_operational(fake_flexivrdk):
    class StuckRobot(FakeRobot):
        def operational(self):
            return False

    fake_flexivrdk.Robot = lambda sn: StuckRobot(sn)
    with pytest.raises(RuntimeError):
        with fh.RobotSession(
            "Rizon4-062930", flexivrdk_module=fake_flexivrdk, operational_timeout_s=0.1
        ):
            pass


def test_robot_session_exit_calls_stop_and_swallows_exit_errors(fake_flexivrdk):
    class FlakyStop(FakeRobot):
        def Stop(self):
            raise RuntimeError("Stop failed at shutdown")

    fake_flexivrdk.Robot = lambda sn: FlakyStop(sn)
    # Exit should not propagate the inner Stop() exception.
    with fh.RobotSession("Rizon4-062930", flexivrdk_module=fake_flexivrdk):
        pass


def test_robot_session_exit_calls_stop_on_success(fake_flexivrdk):
    captured = {}

    class CapturingRobot(FakeRobot):
        def Stop(self):
            captured["stopped"] = True
            super().Stop()

    fake_flexivrdk.Robot = lambda sn: CapturingRobot(sn)
    with fh.RobotSession("Rizon4-062930", flexivrdk_module=fake_flexivrdk):
        pass
    assert captured.get("stopped") is True


def test_robot_session_switch_mode_routes_through_named_lookup(fake_flexivrdk):
    with fh.RobotSession("Rizon4-062930", flexivrdk_module=fake_flexivrdk) as session:
        session.switch_mode("NRT_PRIMITIVE_EXECUTION")
        assert session.robot.last_mode == "M_NRT_PRIMITIVE_EXECUTION"


def test_robot_session_set_cartesian_impedance_routes_to_setter(fake_flexivrdk):
    with fh.RobotSession("Rizon4-062930", flexivrdk_module=fake_flexivrdk) as session:
        session.set_cartesian_impedance(kx=50.0, ky=60.0, kz=3000.0, k_rot=40.0)
        assert session.robot.last_stiffness == [50.0, 60.0, 3000.0, 40.0, 40.0, 40.0]


def test_robot_session_set_cartesian_impedance_rejects_negative(fake_flexivrdk):
    with fh.RobotSession("Rizon4-062930", flexivrdk_module=fake_flexivrdk) as session:
        with pytest.raises(ValueError):
            session.set_cartesian_impedance(-1.0, 50.0, 3000.0, 40.0)


def test_robot_session_setup_gripper(fake_flexivrdk):
    with fh.RobotSession("Rizon4-062930", flexivrdk_module=fake_flexivrdk) as session:
        gripper = session.setup_gripper("Flexiv-GN01", init=False)
        assert isinstance(gripper, _FakeGripper)
        assert gripper.enabled_name == "Flexiv-GN01"
        assert gripper.initialized is False
        assert session.gripper is gripper


# ----------------- H1: MODE_VALUES no longer carries Cartesian fallbacks -----------------


def test_mode_values_omits_cartesian_entries():
    assert "NRT_CARTESIAN_MOTION_FORCE" not in fh.MODE_VALUES
    assert "RT_CARTESIAN_MOTION_FORCE" not in fh.MODE_VALUES
    assert fh.MODE_VALUES["NRT_PRIMITIVE_EXECUTION"] == 8
    assert fh.MODE_VALUES["NRT_JOINT_IMPEDANCE"] == 4


def test_rdk_mode_raises_for_cartesian_when_attribute_missing(fake_flexivrdk):
    # Healthy v1.9 builds expose the attribute. When the attribute is missing the
    # fallback table no longer rescues us — loud failure is the desired behavior.
    fake_flexivrdk.Mode = type("Empty", (), {})
    with pytest.raises(RuntimeError):
        fh.rdk_mode("NRT_CARTESIAN_MOTION_FORCE", fake_flexivrdk)


# ----------------- H2: damping_ratio kwarg + no SetCartesianStiffness probe ----------------


def test_set_cartesian_impedance_default_omits_damping_ratio(fake_flexivrdk):
    with fh.RobotSession("Rizon4-062930", flexivrdk_module=fake_flexivrdk) as session:
        session.set_cartesian_impedance(kx=50.0, ky=60.0, kz=3000.0, k_rot=40.0)
        assert session.robot.last_damping_ratio is None


def test_set_cartesian_impedance_threads_damping_ratio_when_provided(fake_flexivrdk):
    z = [0.7, 0.7, 0.7, 0.6, 0.6, 0.6]
    with fh.RobotSession("Rizon4-062930", flexivrdk_module=fake_flexivrdk) as session:
        session.set_cartesian_impedance(50.0, 50.0, 3000.0, 40.0, damping_ratio=z)
        assert session.robot.last_damping_ratio == z


def test_set_cartesian_impedance_rejects_bad_damping_ratio(fake_flexivrdk):
    with fh.RobotSession("Rizon4-062930", flexivrdk_module=fake_flexivrdk) as session:
        with pytest.raises(ValueError):
            session.set_cartesian_impedance(
                50.0, 50.0, 3000.0, 40.0, damping_ratio=[0.7, 0.7, 0.7]
            )
        with pytest.raises(ValueError):
            session.set_cartesian_impedance(
                50.0, 50.0, 3000.0, 40.0,
                damping_ratio=[0.7, 0.7, 0.7, 0.7, 0.7, -0.1],
            )


def test_set_cartesian_impedance_raises_when_setter_missing(fake_flexivrdk):
    # Shadow the parent's method with a non-callable so the setter probe fails.
    class NoSetterRobot(FakeRobot):
        SetCartesianImpedance = None

    fake_flexivrdk.Robot = lambda sn: NoSetterRobot(sn)
    with fh.RobotSession("Rizon4-062930", flexivrdk_module=fake_flexivrdk) as session:
        with pytest.raises(RuntimeError):
            session.set_cartesian_impedance(50.0, 50.0, 3000.0, 40.0)


# ----------------- H3: RDK API surface log line at __enter__ -----------------


class _CapturingLogger:
    def __init__(self) -> None:
        self.info_lines: list[str] = []
        self.warn_lines: list[str] = []

    def info(self, msg: str) -> None:
        self.info_lines.append(msg)

    def warn(self, msg: str) -> None:
        self.warn_lines.append(msg)

    def warning(self, msg: str) -> None:
        self.warn_lines.append(msg)


def test_robot_session_logs_joint_group_api_surface(fake_flexivrdk):
    logger = _CapturingLogger()
    with fh.RobotSession(
        "Rizon4-062930", logger=logger, flexivrdk_module=fake_flexivrdk
    ):
        pass
    matches = [line for line in logger.info_lines if "RDK API surface" in line]
    assert matches == ["RDK API surface: joint-group (v2.0 path)"]


def test_robot_session_logs_single_group_api_surface(fake_flexivrdk):
    # Strip PrimitiveArgs to force the v1.9 single-group path. Note that
    # ``hasattr(robot, "groups")`` alone isn't enough — both ``groups()`` AND
    # ``PrimitiveArgs`` must be present for the v2 path to register.
    del fake_flexivrdk.PrimitiveArgs

    logger = _CapturingLogger()
    with fh.RobotSession(
        "Rizon4-062930", logger=logger, flexivrdk_module=fake_flexivrdk
    ):
        pass
    matches = [line for line in logger.info_lines if "RDK API surface" in line]
    assert matches == ["RDK API surface: single-group (v1.9 path)"]


# ----------------- M1: unit-conversion helpers -----------------


def test_joints_to_jpos_deg_converts_radians_to_degrees():
    import math as _m

    result = fh.joints_to_jpos_deg([0.0, _m.pi / 2, _m.pi, -_m.pi / 4, 0.0, 0.0, 0.0])
    assert len(result) == 7
    assert result[0] == pytest.approx(0.0)
    assert result[1] == pytest.approx(90.0)
    assert result[2] == pytest.approx(180.0)
    assert result[3] == pytest.approx(-45.0)


def test_joints_to_jpos_deg_rejects_wrong_length():
    with pytest.raises(ValueError):
        fh.joints_to_jpos_deg([0.0, 0.0, 0.0])


def test_quat_to_rpy_deg_identity_is_zero():
    rpy = fh.quat_to_rpy_deg(1.0, 0.0, 0.0, 0.0)
    assert rpy == pytest.approx([0.0, 0.0, 0.0])


def test_quat_to_rpy_deg_90deg_about_z():
    import math as _m

    half = _m.sin(_m.pi / 4)  # sin(45°) for the half-angle
    rpy = fh.quat_to_rpy_deg(half, 0.0, 0.0, half)
    assert rpy[0] == pytest.approx(0.0, abs=1e-9)
    assert rpy[1] == pytest.approx(0.0, abs=1e-9)
    assert rpy[2] == pytest.approx(90.0, abs=1e-9)


def test_tcp_pose_to_coord_args_round_trip_through_fake_coord(fake_flexivrdk):
    captured: dict = {}

    class _FakeCoord:
        def __init__(self, position, orientation, ref_frame, ref_q_m, ref_q_e=None):
            captured["position"] = list(position)
            captured["orientation"] = list(orientation)
            captured["ref_frame"] = list(ref_frame)
            captured["ref_q_m"] = list(ref_q_m)
            captured["ref_q_e"] = list(ref_q_e) if ref_q_e is not None else []

    fake_flexivrdk.Coord = _FakeCoord

    tcp_pose = [0.123, -0.456, 0.789, 1.0, 0.0, 0.0, 0.0]
    ref_joints = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0]
    args = fh.tcp_pose_to_coord_args(
        tcp_pose,
        ref_frame=("WORLD", "WORLD_ORIGIN"),
        ref_joints_deg=ref_joints,
    )
    assert len(args) == 5
    coord = fake_flexivrdk.Coord(*args)  # noqa: F841
    assert captured["position"] == [0.123, -0.456, 0.789]
    assert captured["orientation"] == pytest.approx([0.0, 0.0, 0.0])
    assert captured["ref_frame"] == ["WORLD", "WORLD_ORIGIN"]
    assert captured["ref_q_m"] == ref_joints
    # flexivrdk.Coord enforces FixedSize(6) on _ref_q_e; an empty list raises
    # TypeError in the real binding, so the helper defaults to 6 zeros.
    assert captured["ref_q_e"] == [0.0] * 6


def test_tcp_pose_to_coord_args_rejects_short_pose():
    with pytest.raises(ValueError):
        fh.tcp_pose_to_coord_args([0.0, 0.0, 0.0])


def test_tcp_pose_to_coord_args_rejects_wrong_size_ref_joints():
    with pytest.raises(ValueError):
        fh.tcp_pose_to_coord_args(
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            ref_joints_deg=[0.0, 0.0, 0.0],
        )


def test_tcp_pose_to_coord_args_rejects_wrong_size_ref_external():
    with pytest.raises(ValueError):
        fh.tcp_pose_to_coord_args(
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            ref_external=[0.0, 0.0],
        )


def test_tcp_pose_to_coord_args_default_ref_joints_is_seven_zeros():
    args = fh.tcp_pose_to_coord_args([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    _pos, _ori, _ref, ref_q_m, ref_q_e = args
    assert ref_q_m == [0.0] * 7
    assert ref_q_e == [0.0] * 6


# ----------------- move_ptp_joint -----------------


class _RecorderSession:
    """Minimal RobotSession stand-in for move_ptp_joint tests."""

    def __init__(self, flexivrdk_module):
        self.flexivrdk = flexivrdk_module
        self.calls: list[tuple[str, object]] = []

    def execute_primitive(self, name, params):
        self.calls.append(("execute_primitive", (name, dict(params))))

    def wait_for_primitive(self, state_key="reachedTarget", dt=0.2, timeout_s=None):
        self.calls.append(("wait_for_primitive", state_key))

    def switch_mode(self, mode_name):
        self.calls.append(("switch_mode", mode_name))


def _flexivrdk_with_coord_and_jpos(fake_flexivrdk):
    """Attach minimal Coord/JPos stubs to the test fixture."""

    class _Coord:
        def __init__(self, position, orientation, ref_frame, ref_q_m, ref_q_e):
            self.position = list(position)
            self.orientation = list(orientation)
            self.ref_frame = list(ref_frame)
            self.ref_q_m = list(ref_q_m)
            self.ref_q_e = list(ref_q_e)

    class _JPos:
        def __init__(self, q_deg):
            self.q_deg = list(q_deg)

    fake_flexivrdk.Coord = _Coord
    fake_flexivrdk.JPos = _JPos
    return fake_flexivrdk


def _pose_entry(q_rad=None, tcp=None):
    q_rad = q_rad if q_rad is not None else [0.1 * (i + 1) for i in range(7)]
    tcp = tcp if tcp is not None else [0.5, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0]
    return {
        "q_rad": q_rad,
        "tcp_pose_world": {
            "order": ["x", "y", "z", "qw", "qx", "qy", "qz"],
            "values": tcp,
        },
    }


def test_move_ptp_joint_dispatches_movePTP_then_waits(fake_flexivrdk):
    flexivrdk = _flexivrdk_with_coord_and_jpos(fake_flexivrdk)
    session = _RecorderSession(flexivrdk)
    pose = _pose_entry()

    fh.move_ptp_joint(session, pose, vel_scale=5)

    assert len(session.calls) == 2
    assert session.calls[0][0] == "execute_primitive"
    name, params = session.calls[0][1]
    assert name == "MovePTP"
    assert params["jntVelScale"] == 5
    # Default is unlocked: captured joints are an IK seed, not a hard target.
    assert params["enableFixRefJntPos"] is False
    assert isinstance(params["target"], flexivrdk.Coord)
    assert isinstance(params["refJntPos"], flexivrdk.JPos)
    # The wait call follows.
    assert session.calls[1] == ("wait_for_primitive", "reachedTarget")


def test_move_ptp_joint_joint_locked_opts_into_fix_ref_jnt_pos(fake_flexivrdk):
    flexivrdk = _flexivrdk_with_coord_and_jpos(fake_flexivrdk)
    session = _RecorderSession(flexivrdk)
    pose = _pose_entry()

    fh.move_ptp_joint(session, pose, joint_locked=True)

    _, params = session.calls[0][1]
    assert params["enableFixRefJntPos"] is True


# ----------------- walk_path -----------------


def test_walk_path_moves_through_each_waypoint_then_end_pose(fake_flexivrdk):
    flexivrdk = _flexivrdk_with_coord_and_jpos(fake_flexivrdk)
    session = _RecorderSession(flexivrdk)
    wp1 = _pose_entry(q_rad=[0.11] * 7, tcp=[0.5, 0.0, 0.55, 1, 0, 0, 0])
    wp2 = _pose_entry(q_rad=[0.22] * 7, tcp=[0.5, 0.0, 0.50, 1, 0, 0, 0])
    end = _pose_entry(q_rad=[0.33] * 7, tcp=[0.49, -0.36, 0.33, 1, 0, 0, 0])
    path = {"waypoints": [wp1, wp2]}

    fh.walk_path(session, path, end_pose=end, vel_scale=5)

    moves = [c for c in session.calls if c[0] == "execute_primitive"]
    assert len(moves) == 3  # 2 waypoints + 1 end pose
    # Every move uses joint-locked + the conservative vel_scale.
    for _, payload in moves:
        name, params = payload
        assert name == "MovePTP"
        assert params["enableFixRefJntPos"] is True
        assert params["jntVelScale"] == 5


def test_walk_path_with_no_waypoints_only_moves_to_end_pose(fake_flexivrdk):
    flexivrdk = _flexivrdk_with_coord_and_jpos(fake_flexivrdk)
    session = _RecorderSession(flexivrdk)
    end = _pose_entry()
    fh.walk_path(session, {"waypoints": []}, end_pose=end)
    moves = [c for c in session.calls if c[0] == "execute_primitive"]
    assert len(moves) == 1


def test_walk_path_without_end_pose_just_walks_waypoints(fake_flexivrdk):
    flexivrdk = _flexivrdk_with_coord_and_jpos(fake_flexivrdk)
    session = _RecorderSession(flexivrdk)
    wp1 = _pose_entry()
    fh.walk_path(session, {"waypoints": [wp1]})
    moves = [c for c in session.calls if c[0] == "execute_primitive"]
    assert len(moves) == 1


# ----------------- descend_until_force_or_depth -----------------


class _MockRobot:
    """Minimal Robot stand-in exposing the Cartesian Motion-Force methods."""

    def __init__(self):
        self.contact_wrench_calls: list[list[float]] = []
        self.motion_force_calls: list[dict] = []

    def SetMaxContactWrench(self, w):  # noqa: N802
        self.contact_wrench_calls.append(list(w))

    def SendCartesianMotionForce(self, pose, wrench, ff, vel):  # noqa: N802
        self.motion_force_calls.append(
            {
                "pose": list(pose),
                "wrench": list(wrench),
                "ff": list(ff),
                "vel": float(vel),
            }
        )


class _DescendSession:
    """Session fake for descend_until_force_or_depth tests.

    Each ``selected_arm_state`` consumes one wrench from the sequence (last
    value repeats once exhausted) and advances the simulated TCP Z by
    ``advance_per_call_m``. Call 0 is the helper's start_tcp read — no
    advance happens before that, but it's counted toward the call index for
    subsequent reads.
    """

    def __init__(self, flexivrdk_module, wrench_sequence, advance_per_call_m=0.0):
        self.flexivrdk = flexivrdk_module
        self._wrench_seq = list(wrench_sequence)
        self._call_idx = 0
        self._start_z = 0.5
        self._advance = float(advance_per_call_m)
        self.mode_switches: list[str] = []
        self.robot = _MockRobot()

    def switch_mode(self, mode_name):
        self.mode_switches.append(mode_name)

    def selected_arm_state(self):
        i = min(self._call_idx, len(self._wrench_seq) - 1)
        wrench = self._wrench_seq[i]
        # TCP advances cumulatively each call past the first.
        z = self._start_z - self._call_idx * self._advance
        self._call_idx += 1
        state = _FakeArmState(
            q=[0.0] * 7,
            tcp_pose=[0.5, 0.0, z, 1.0, 0.0, 0.0, 0.0],
        )
        state.ext_wrench_in_world = list(wrench)
        return "ARM", state


def test_descend_switches_to_motion_force_mode_and_restores(fake_flexivrdk):
    flexivrdk = _flexivrdk_with_coord_and_jpos(fake_flexivrdk)
    # advance_per_call > max_depth so the loop terminates via "depth" on its
    # first poll. Otherwise we'd sit at zero force/zero descent until the 5 s
    # safety deadline expires, slowing the test suite.
    session = _DescendSession(
        flexivrdk, [[0, 0, 0, 0, 0, 0]] * 5, advance_per_call_m=0.002
    )
    fh.descend_until_force_or_depth(
        session,
        max_depth_m=0.001,
        force_threshold_n=5.0,
        vel_m_s=0.005,
        poll_dt_s=0.0,
        settle_after_s=0.0,
    )
    assert session.mode_switches[0] == "NRT_CARTESIAN_MOTION_FORCE"
    assert session.mode_switches[-1] == "NRT_PRIMITIVE_EXECUTION"


def test_descend_dispatches_set_max_contact_and_motion_force(fake_flexivrdk):
    flexivrdk = _flexivrdk_with_coord_and_jpos(fake_flexivrdk)
    session = _DescendSession(
        flexivrdk, [[0, 0, 0, 0, 0, 0]] * 5, advance_per_call_m=0.002
    )
    fh.descend_until_force_or_depth(
        session,
        max_depth_m=0.001,
        force_threshold_n=5.0,
        vel_m_s=0.005,
        poll_dt_s=0.0,
        settle_after_s=0.0,
    )
    assert len(session.robot.contact_wrench_calls) == 1
    # Default contact wrench: 2x threshold floored at 10 N per linear axis.
    assert session.robot.contact_wrench_calls[0][0] == pytest.approx(10.0)
    # Initial motion-force command targets max_depth below start.
    initial = session.robot.motion_force_calls[0]
    assert initial["pose"][2] == pytest.approx(0.5 - 0.001)
    assert initial["wrench"] == [0.0] * 6
    assert initial["vel"] == pytest.approx(0.005)


def test_descend_force_trigger_sends_halt_in_place(fake_flexivrdk):
    flexivrdk = _flexivrdk_with_coord_and_jpos(fake_flexivrdk)
    # First wrench is consumed by start_tcp read; force exceeds threshold on
    # the very first poll iteration.
    wrench_seq = [
        [0, 0, 0, 0, 0, 0],     # idx 0 — start_tcp
        [0, 0, 12.0, 0, 0, 0],  # idx 1 — first poll: |F| = 12 N > 5 N -> abort
    ]
    session = _DescendSession(flexivrdk, wrench_seq, advance_per_call_m=0.001)
    result = fh.descend_until_force_or_depth(
        session,
        max_depth_m=0.05,
        force_threshold_n=5.0,
        vel_m_s=0.005,
        poll_dt_s=0.0,
        settle_after_s=0.0,
    )
    assert result["reason"] == "force"
    assert result["fz_at_stop"] == pytest.approx(12.0)
    # Two motion-force commands: initial descent + halt-in-place.
    assert len(session.robot.motion_force_calls) == 2
    # Halt's pose was the TCP read at the moment of force detection.
    halt = session.robot.motion_force_calls[1]
    # call_idx was 1 (start_tcp) then 2 (force read) when halt was issued;
    # advance was applied at call_idx=1, so z = 0.5 - 1*0.001 = 0.499.
    assert halt["pose"][2] == pytest.approx(0.499)


def test_descend_reaches_max_depth_when_no_contact(fake_flexivrdk):
    flexivrdk = _flexivrdk_with_coord_and_jpos(fake_flexivrdk)
    # Wrench always quiet. Advance per poll = 2 mm; max_depth = 10 mm.
    # Helper reads tcp after each iteration's sleep; descent_m at iter N
    # equals N * advance (since call_idx grows by 1 per state read).
    session = _DescendSession(
        flexivrdk,
        [[0, 0, 0.1, 0, 0, 0]] * 100,
        advance_per_call_m=0.002,
    )
    result = fh.descend_until_force_or_depth(
        session,
        max_depth_m=0.010,
        force_threshold_n=5.0,
        vel_m_s=0.005,
        poll_dt_s=0.0,
        settle_after_s=0.0,
    )
    assert result["reason"] == "depth"
    assert result["descent_m"] >= 0.010
    # Only the initial SendCartesianMotionForce; no halt issued.
    assert len(session.robot.motion_force_calls) == 1


def test_descend_force_norm_triggers_on_non_z_axis(fake_flexivrdk):
    """The threshold is on |F| = sqrt(fx²+fy²+fz²), not just fz."""
    flexivrdk = _flexivrdk_with_coord_and_jpos(fake_flexivrdk)
    wrench_seq = [
        [0, 0, 0, 0, 0, 0],     # start_tcp
        [10.0, 0, 0, 0, 0, 0],  # |F| = 10 in X direction
    ]
    session = _DescendSession(flexivrdk, wrench_seq, advance_per_call_m=0.001)
    result = fh.descend_until_force_or_depth(
        session,
        max_depth_m=0.05,
        force_threshold_n=5.0,
        vel_m_s=0.005,
        poll_dt_s=0.0,
        settle_after_s=0.0,
    )
    assert result["reason"] == "force"


def test_descend_restores_mode_on_exception(fake_flexivrdk):
    """If the controller raises mid-descent, the mode is still restored."""
    flexivrdk = _flexivrdk_with_coord_and_jpos(fake_flexivrdk)
    session = _DescendSession(flexivrdk, [[0, 0, 0, 0, 0, 0]] * 100)

    def boom(*a, **kw):
        raise RuntimeError("simulated controller failure")

    session.robot.SetMaxContactWrench = boom
    with pytest.raises(RuntimeError):
        fh.descend_until_force_or_depth(
            session,
            max_depth_m=0.01,
            force_threshold_n=5.0,
            poll_dt_s=0.0,
            settle_after_s=0.0,
        )
    # Mode was switched in, and the finally clause restored it.
    assert session.mode_switches[0] == "NRT_CARTESIAN_MOTION_FORCE"
    assert session.mode_switches[-1] == "NRT_PRIMITIVE_EXECUTION"


# ----------------- zero_ft_sensor -----------------


def test_zero_ft_sensor_switches_mode_and_runs_primitive(fake_flexivrdk):
    flexivrdk = _flexivrdk_with_coord_and_jpos(fake_flexivrdk)
    session = _RecorderSession(flexivrdk)
    fh.zero_ft_sensor(session)
    # Mode forced to primitive execution.
    assert ("switch_mode", "NRT_PRIMITIVE_EXECUTION") in session.calls
    # ZeroFTSensor primitive issued with empty params.
    primitives = [
        c for c in session.calls if c[0] == "execute_primitive"
    ]
    assert any(name == "ZeroFTSensor" for _, (name, _) in primitives)
    # Helper waits on the "terminated" state key (not reachedTarget).
    assert ("wait_for_primitive", "terminated") in session.calls


def test_walk_path_joint_locked_false_carries_through(fake_flexivrdk):
    flexivrdk = _flexivrdk_with_coord_and_jpos(fake_flexivrdk)
    session = _RecorderSession(flexivrdk)
    wp = _pose_entry()
    fh.walk_path(session, {"waypoints": [wp]}, joint_locked=False)
    _, payload = [c for c in session.calls if c[0] == "execute_primitive"][0]
    _, params = payload
    assert params["enableFixRefJntPos"] is False


# ----------------- move_z_relative -----------------


class _RecorderSessionWithState(_RecorderSession):
    """Like _RecorderSession but exposes a current arm state for readback."""

    def __init__(self, flexivrdk_module, current_tcp):
        super().__init__(flexivrdk_module)
        self._current_tcp = list(current_tcp)

    def selected_arm_state(self):
        state = _FakeArmState(tcp_pose=list(self._current_tcp))
        return "ARM", state


def test_move_z_relative_descends_by_negative_delta(fake_flexivrdk):
    flexivrdk = _flexivrdk_with_coord_and_jpos(fake_flexivrdk)
    # Arm currently sits at (0.5, 0.1, 0.3) with identity orientation.
    session = _RecorderSessionWithState(
        flexivrdk, current_tcp=[0.5, 0.1, 0.3, 1.0, 0.0, 0.0, 0.0]
    )

    fh.move_z_relative(session, -0.05, vel_m_s=0.04)

    assert session.calls[0][0] == "execute_primitive"
    name, params = session.calls[0][1]
    assert name == "MoveL"
    assert params["vel"] == pytest.approx(0.04)
    # Position carried over with Z reduced by 50 mm.
    target = params["target"]
    assert target.position == pytest.approx([0.5, 0.1, 0.25])
    # WORLD frame.
    assert target.ref_frame == ["WORLD", "WORLD_ORIGIN"]
    # Then a wait.
    assert session.calls[1] == ("wait_for_primitive", "reachedTarget")


def test_move_z_relative_positive_delta_ascends(fake_flexivrdk):
    flexivrdk = _flexivrdk_with_coord_and_jpos(fake_flexivrdk)
    session = _RecorderSessionWithState(
        flexivrdk, current_tcp=[0.4, -0.2, 0.5, 1.0, 0.0, 0.0, 0.0]
    )
    fh.move_z_relative(session, +0.1)
    _, params = session.calls[0][1]
    assert params["target"].position == pytest.approx([0.4, -0.2, 0.6])


def test_move_z_relative_holds_orientation(fake_flexivrdk):
    flexivrdk = _flexivrdk_with_coord_and_jpos(fake_flexivrdk)
    # Start with a non-identity orientation (90 deg about Z, hand-computed).
    start_tcp = [0.4, 0.0, 0.3, 0.7071, 0.0, 0.0, 0.7071]
    session = _RecorderSessionWithState(flexivrdk, current_tcp=start_tcp)

    fh.move_z_relative(session, -0.05)

    _, params = session.calls[0][1]
    # The Coord wrapper holds RPY (degrees) derived from the same quaternion;
    # check that the orientation isn't accidentally zeroed (would happen if
    # the helper forgot to carry over the quaternion).
    assert any(abs(v) > 1e-3 for v in params["target"].orientation), (
        "expected orientation to be carried over from current TCP, "
        f"got {params['target'].orientation}"
    )


# ----------------- wiggle_about_virtual_point -----------------


def test_rotate_tcp_zero_delta_is_identity():
    # Identity quat input (already normalized) — helper renormalizes internally
    # so a slightly off-norm input would otherwise drift by ~1e-6.
    tcp = [0.5, -0.1, 0.4, 1.0, 0.0, 0.0, 0.0]
    result = fh.rotate_tcp_about_virtual_point(tcp, offset_cm=2.0, roll_deg=0, pitch_deg=0, yaw_deg=0)
    for a, b in zip(result, tcp):
        assert a == pytest.approx(b, abs=1e-9)


def test_rotate_tcp_with_zero_offset_preserves_position():
    """offset_cm=0 means pivot at TCP origin -> only orientation changes."""
    tcp = [0.5, -0.1, 0.4, 1.0, 0.0, 0.0, 0.0]
    result = fh.rotate_tcp_about_virtual_point(
        tcp, offset_cm=0.0, roll_deg=0, pitch_deg=0, yaw_deg=10.0
    )
    assert result[0] == pytest.approx(0.5, abs=1e-9)
    assert result[1] == pytest.approx(-0.1, abs=1e-9)
    assert result[2] == pytest.approx(0.4, abs=1e-9)
    # Quaternion has changed (10 deg yaw applied).
    assert result[3] != pytest.approx(1.0, abs=1e-6)


def test_wiggle_about_virtual_point_dispatches_expected_count(fake_flexivrdk):
    flexivrdk = _flexivrdk_with_coord_and_jpos(fake_flexivrdk)

    class _S(_RecorderSession):
        def selected_arm_state(self):
            state = _FakeArmState(
                q=[0.0] * 7,
                tcp_pose=[0.5, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0],
            )
            return "ARM", state

    session = _S(flexivrdk)
    fh.wiggle_about_virtual_point(
        session, yaw_deg=5.0, repeat_count=3
    )
    # Keyframes: [zero, +Δ, -Δ, +Δ, -Δ, +Δ, -Δ, zero] = 8 entries.
    # Transitions executed = 8 - 1 = 7 -> 7 MovePTP calls.
    moves = [c for c in session.calls if c[0] == "execute_primitive"]
    assert len(moves) == 7
    waits = [c for c in session.calls if c[0] == "wait_for_primitive"]
    assert len(waits) == 7


# ----------------- gripper_set -----------------


class _FakeGripperLive:
    """Gripper stand-in that records Move() calls and reports is_moving."""

    class _States:
        def __init__(self, width, force, is_moving):
            self.width = width
            self.force = force
            self.is_moving = is_moving

    def __init__(self, moving_polls: int = 1) -> None:
        self.moves: list[tuple[float, float, float]] = []
        self._moving_polls = moving_polls
        self._polls = 0

    def Move(self, width, vel, force):  # noqa: N802
        self.moves.append((width, vel, force))
        self._polls = 0  # reset for a new move

    def states(self):
        self._polls += 1
        moving = self._polls <= self._moving_polls
        # If moves has been called, reflect the requested width once stopped.
        w = self.moves[-1][0] if self.moves else 0.04
        return self._States(width=w, force=0.0, is_moving=moving)


def test_gripper_set_dispatches_move_with_args():
    g = _FakeGripperLive(moving_polls=0)  # appears stopped immediately
    fh.gripper_set(g, 0.005, vel_m_s=0.04, force_n=8.0, poll_dt_s=0.0)
    assert g.moves == [(0.005, 0.04, 8.0)]


def test_gripper_set_polls_until_not_moving():
    g = _FakeGripperLive(moving_polls=3)
    fh.gripper_set(g, 0.005, poll_dt_s=0.0, timeout_s=1.0)
    # Polled at least until is_moving went false (4th poll).
    assert g._polls >= 4


def test_gripper_set_returns_on_timeout_without_raising():
    g = _FakeGripperLive(moving_polls=10**6)  # never stops
    fh.gripper_set(g, 0.005, poll_dt_s=0.0, timeout_s=0.05)


def test_gripper_set_wait_false_skips_polling():
    g = _FakeGripperLive(moving_polls=10**6)
    fh.gripper_set(g, 0.005, wait=False, poll_dt_s=0.0)
    assert g.moves == [(0.005, 0.05, 10.0)]  # defaults flowed through
    assert g._polls == 0  # states() was never called


def test_gripper_set_settle_after_s_sleeps_post_poll(monkeypatch):
    g = _FakeGripperLive(moving_polls=0)
    sleeps: list[float] = []
    import time as _time

    real_sleep = _time.sleep

    def fake_sleep(t):
        sleeps.append(t)
        # Don't actually delay tests.

    monkeypatch.setattr(_time, "sleep", fake_sleep)
    fh.gripper_set(g, 0.005, poll_dt_s=0.0, settle_after_s=0.25)
    # The settle delay was requested as one of the sleep calls.
    assert 0.25 in sleeps
    monkeypatch.setattr(_time, "sleep", real_sleep)


def test_gripper_set_settle_zero_does_not_sleep_at_end(monkeypatch):
    g = _FakeGripperLive(moving_polls=0)
    sleeps: list[float] = []
    import time as _time

    monkeypatch.setattr(_time, "sleep", lambda t: sleeps.append(t))
    fh.gripper_set(g, 0.005, poll_dt_s=0.0, settle_after_s=0.0)
    # Default settle of 0 means no >0 sleep call for settling.
    # The initial poll_dt_s sleep is 0; nothing else gets recorded.
    assert all(s == 0.0 for s in sleeps)


def test_move_z_relative_rejects_bad_tcp_state(fake_flexivrdk):
    flexivrdk = _flexivrdk_with_coord_and_jpos(fake_flexivrdk)
    # Arm state returns a 3-element TCP instead of 7.
    session = _RecorderSessionWithState(flexivrdk, current_tcp=[0.0, 0.0, 0.0])
    with pytest.raises(RuntimeError, match="tcp_pose has 3 values"):
        fh.move_z_relative(session, -0.01)


def test_move_ptp_joint_passes_captured_joints_as_ik_seed(fake_flexivrdk):
    flexivrdk = _flexivrdk_with_coord_and_jpos(fake_flexivrdk)
    session = _RecorderSession(flexivrdk)
    # Distinctive joint values so we can confirm they propagated.
    import math as _math
    q_rad = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    pose = _pose_entry(q_rad=q_rad)

    fh.move_ptp_joint(session, pose)

    _, params = session.calls[0][1]
    expected_deg = [_math.degrees(v) for v in q_rad]
    assert params["refJntPos"].q_deg == pytest.approx(expected_deg)
    assert params["target"].ref_q_m == pytest.approx(expected_deg)
