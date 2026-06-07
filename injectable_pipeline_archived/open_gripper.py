"""One-shot: fully open the GN01 gripper. Use when the recipe left it closed."""

from flexiv_helpers import RobotSession, gripper_set

ROBOT_SN = "Rizon4-062930"
GRIPPER_NAME = "Flexiv-GN01"
OPEN_WIDTH_M = 0.04

with RobotSession(ROBOT_SN) as session:
    # init=True runs Init() — the gripper's calibration sequence — which is
    # required for Move() to actually take effect in a fresh subsystem state.
    # ~4 s startup cost but reliable.
    gripper = session.setup_gripper(GRIPPER_NAME, init=True)
    # settle_after_s holds the connection open after polling exits so the
    # async Move can complete before robot.Stop() fires on context exit.
    gripper_set(gripper, OPEN_WIDTH_M, settle_after_s=1.5)
