# Recipe Development Notes

This is the shared recipe-development guide for scripts under `project/recipe/`.
Individual recipe folders should keep their own notes focused on the actual object,
poses, robot moves, and Arduino/cutting-machine actions for that recipe.

## What We Learned From `injectable_pipeline`

The injectable pipeline has several patterns worth keeping:

- Keep hardware plumbing out of recipes. Robot setup, primitive dispatch, gripper
  moves, quaternion math, force descent, and Arduino serial commands belong in
  shared helpers.
- Make a recipe a readable sequence of named phases. Each phase should say what
  the robot and external hardware should do next.
- Use captured named positions as data, not code. Recipe-specific pose data should
  live beside the recipe in `key_positions/`, `paths/`, or another small data file.
- Put tuning values at the top of the recipe. Speeds, gripper widths, force limits,
  approach distances, and pauses should be visible before the motion logic.
- Always have a safe intermediate. Every risky travel should route through a known
  clear pose unless a direct move has been intentionally validated.
- Make dry-run and validation easy. Before moving hardware, a recipe should be able
  to load positions, print the planned phase order, and fail fast if data is missing.

## Shared Helper Layer

Reusable code copied forward from `project/injectable_pipeline/` lives in:

```text
project/helper/
  __init__.py
  flexiv_helpers.py
  arduino_client.py
```

Recipe scripts should import from this helper layer instead of reimplementing
functions from `record_robot_waypoints.py`, `play_recorded_waypoints.py`, or the
injectable scripts.

Most useful helpers:

- `RobotSession` for connect, enable, fault clear, operational wait, and shutdown.
- `move_ptp_joint` for moving to a captured pose.
- `move_z_relative` for vertical approach / retreat.
- `walk_path` when a recipe records multi-waypoint paths.
- `gripper_set` for open / close with wait and force limits.
- `tcp_pose_to_coord_args`, `quat_to_rpy_deg`, and `joints_to_jpos_deg` for pose
  conversion.
- `ArduinoClient` for vise, cutter, and machine-state coordination.

## Lessons From `LLM.txt` And `example_py`

The Flexiv examples and `project/LLM.txt` point to a reliable robot-side lifecycle:

1. Create the robot connection.
2. Clear robot fault if one is present.
3. Enable the robot and wait for `operational()`.
4. Switch to `NRT_PRIMITIVE_EXECUTION` before primitive moves.
5. Execute one primitive at a time.
6. Wait on the primitive's documented transition condition.
7. Stop the robot on exit.

`helper.flexiv_helpers.RobotSession` already owns most of this. Recipes should
focus on phase order and leave robot session mechanics to the helper.

Examples to learn from:

- `example_py/basics3_primitive_execution.py`: explicit primitives such as
  `Home`, `MoveJ`, `MoveL`; wait for `reachedTarget`.
- `example_py/basics5_zero_force_torque_sensors.py`: call `ZeroFTSensor` before
  force-sensitive motion and wait for `terminated`.
- `example_py/basics6_gripper_control.py`: the gripper is both a device and a
  tool; enable the gripper and switch the robot tool before recipe motion.
- `example_py/intermediate3_non_realtime_cartesian_pure_motion_control.py`: use
  `NRT_CARTESIAN_MOTION_FORCE` only when the recipe needs Cartesian direct motion
  or compliance, then restore primitive execution mode.
- `example_py/intermediate4_non_realtime_cartesian_motion_force_control.py`: for
  contact search, set a max contact wrench, move slowly, poll external wrench,
  and stop when contact is detected.

Important API constraints from `LLM.txt`:

- The project default robot serial is `Rizon4-062930`.
- RDK robot Cartesian distances are in meters.
- Captured key positions store quaternions, but `flexivrdk.Coord` wants Euler
  orientation. Use helper conversion functions.
- `flexivrdk.Coord` has five conceptual fields: position, orientation, reference
  frame, reference robot joints, and reference external-axis joints. The helper
  fills the fixed-size vectors.
- Primitive-state shape varies between RDK versions. Use the helper's group-aware
  `execute_primitive` and `wait_for_primitive`.

## Cutting-Machine Coordination

The cutting machine is a separate state machine behind `ArduinoClient`. A recipe
should treat it as a peer subsystem, not as part of the robot API.

Useful cutting-machine rules:

- Serial protocol is line-based ASCII at 115200 baud.
- Mutating commands emit `ACK`, then exactly one `DONE` or `ERR`.
- Query commands such as `GET_STATUS` return immediate `DONE` or `ERR`.
- Only one mutating cutting-machine command may run at a time.
- Absolute X/Z/rotary commands require `HOME_ALL` first.
- `STOP_ALL` is the emergency software command and forces `blade_on=false`.
- Vise commands may take a long time; use the helper client's longer timeout.
- A machine timeout or fault should stop the recipe rather than auto-retry.

Use this startup gate for recipes that need the vise or cutter:

1. Connect `ArduinoClient`.
2. Read `GET_STATUS`.
3. If `faulted=true`, stop for operator inspection, or call `CLEAR_FAULTS` only
   after the operator confirms the cause is gone.
4. If `homed=false`, run `HOME_ALL` before absolute moves, vise commands, or cuts.
5. Confirm `busy=false` and `blade_on=false` before letting the robot enter the
   machine workspace.

Safe robot/cutter handoff rules:

- The robot owns the part until the vise has positively clamped it.
- The vise owns the part before any blade motion.
- The robot must be at a proven safe pose before `CUT_HEIGHT`, raw X/Z motion, or
  blade-on commands.
- During `CUT_HEIGHT`, the robot should not move.
- After `CUT_HEIGHT` returns `DONE`, verify `blade_on=false`, `x_mm=0`, `z_mm=0`,
  and `rot_deg=0` before robot re-entry.
- If any Arduino command raises timeout or `ERR`, call `STOP_ALL`, keep the robot
  clear, and end the run.

## Universal Recipe Shape

A recipe should be a thin executable wrapped around four pieces:

1. **Params**: all tunable numbers at the top of the file.
2. **Position loading**: read required key-position JSON files into a common pose
   format.
3. **Phase functions**: small functions such as `phase_pick`, `phase_transfer`,
   and `phase_place`.
4. **Runtime shell**: parse CLI flags, validate required data, connect hardware,
   run phases, and stop cleanly on fault.

Preferred folder layout:

```text
project/recipe/<RecipeName>/
  <RecipeName>V1.py
  <recipename>.md
  key_positions/
    ...
```

Later, if a recipe needs many positions or paths, add:

```text
  paths/
  logs/
  params.json
```

Keep each recipe script recipe-specific. Anything reusable by another recipe should
move to `project/helper/`.

## Dry-Run Expectations

Every recipe should eventually support a dry-run mode that does not connect to
hardware. It should print:

- resolved key-position directory
- required positions found / missing
- phase order
- current params
- whether Arduino coordination is enabled
- machine preconditions that would be checked before motion
