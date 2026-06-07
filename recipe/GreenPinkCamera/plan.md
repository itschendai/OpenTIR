# GreenPink Injectable Teardown

This file is a collaboration document for building `recipe.py`.

- **Objective (human pseudo code)** is where the human writes what the system
  should do, in plain operational language.
- **Implementation workspace** is where we translate that objective into poses,
  robot primitives, Arduino commands, safety checks, and code tasks.

For later recipes, copy this structure: keep the human objective near the top,
then let the rest of the file explain how we will turn it into a Python script.

## Objective (human pseudo code)

This section is human-owned. Edit it freely when the desired behavior changes.
The implementation sections below should be updated to match it.

//frame 1
1. Robot opens gripper.
2. Robot moves to Middle point.
Calibration preflight between steps 2 and 3: robot moves to tag 1, then tag 2, then tag 3. If the calibration flag is on, it runs `cali tag N`, updates the saved key positions for Vise, Plate, Spring, Glass, and Plastic, then returns to Middle.

//frame 2
3. Robot moves to Plate point.
4. Robot runs Align Injectable.
5. Robot runs Grasp.
6. Robot moves straight up World Z +20 cm.
7. Robot moves to Middle point.

//frame 3
8. Robot moves to above Vise position.
9. Robot runs Flexiv `InsertComp` primitive down in world Z to insert injectable into vise.
10. Machine closes vise.
11. Robot exits the compliant insertion hold, switches to joint hold / non-force control at the current pose, then opens gripper to 5 cm width to release the injectable.
12. While still in joint hold, robot opens gripper fully, waits for gripper to finish opening, then switches back to motion primitive control and moves straight up in world Z by 15 cm to avoid hitting the injectable and wait.

//frame 4
13. Machine starts cut height primitive (`cut_height z_mm=133 x_mm=110.5 deg=360`) to remove top.
14. Robot moves down to Vise position.
15. Robot closes gripper to 80 N.
16. Robot performs a twist: rotate around TCP X by 5 degrees, then return to 0.
17. Robot slowly raises up in **positive world Z** by 20 cm to release cap.

//frame 5
18. Robot moves to Middle point.
19. Robot moves to Plastic point.
20. Robot opens gripper.
Remove spring:
21. Robot moves to Middle point.
22. Robot moves to above vise position (Vise world Z +10 cm).
23. Robot moves down to Vise position.
24. Robot closes gripper to 80 N.
25. Robot slowly raises up in **positive world Z** by 15 cm to release cap.
26. Robot moves to Middle point.
27. Robot moves to Spring point.
28. Robot opens gripper.
Remove yellow plastic:
29. Robot moves to Middle point.
30. Robot moves to above vise position (Vise world Z +10 cm).
31. Robot moves down to a bit below Vise position (Vise world Z -3.8 cm).
32. Robot closes gripper to 80 N.
33. Robot slowly raises up in **positive world Z** by 10 cm to release cap.
34. Robot moves to Middle point.
35. Robot moves to Plastic point.
36. Robot opens gripper.
Remove shell and glass:
37. Robot moves to Middle point.
38. Robot moves to above vise position (Vise world Z +10 cm).
39. Robot moves down to a bit below Vise position (Vise world Z -6 cm).
40. Robot closes gripper to 80 N.
41. Machine opens vise.
42. Robot slowly raises up in **positive world Z** by 20 cm to release cap.
43. Robot moves to Middle point.
44. Robot moves to Glass point.
45. The robot performs Dump with one continuous MoveC arc around an upper-left virtual point in the tool frame (Rz 176 and then turn back).
46. Robot moves to Plastic point.
47. Robot opens gripper.
48. Robot moves to Middle point.

## Implementation Workspace

Everything below this line is our shared translation layer for turning the
objective into code.

### Current Interpretation

The current objective describes a first integrated teardown:

1. Start machine-side at `Inter`.
2. Move through the `Home` waypoint before approaching the pickup plate.
3. Run a vise relocalization preflight: move to captured `Vise-cali`, detect
   the fixed ChArUco board, update the saved `Vise` key position from
   `tag_01_to_vise_tcp.json`, then return to `Home`.
4. Move to captured `Plate`.
5. Run camera-based `Align Injectable`, then adaptive `Grasp`.
6. Lift the whole injectable straight up in positive world Z, then leave
   through `Home` to `Inter`.
7. Load it into the vise from `above_vise` using Flexiv `InsertComp`.
8. Let the Arduino close the vise.
9. Exit the compliant insertion hold, park the arm in joint hold for the gripper-open actions, then switch back to motion primitive control and retreat to a safe waiting pose.
10. Let the Arduino run `CUT_HEIGHT` with `z=133`, `x=110.5`, `deg=360` to
   remove the top.
11. Return to relocalized `Vise`, grip the cap, twist around TCP X, lift it out in
   positive world Z, then leave the machine through `Inter` and `Home` before
   dropping at `plastic`.

This means `recipe.py` is not just robot playback. It coordinates the Flexiv
robot, the GN01 gripper, and the Arduino cutting machine.

### Captured Key Positions

Key positions live in `key_positions/`.

| Name | File | Current use |
| --- | --- | --- |
| `Inter` | `Inter.json` | Machine-side intermediate / waiting pose near the vise. |
| `Home` | `Home.json` | Transfer waypoint for entering/exiting the pickup side. |
| `Vise-cali` | `Vise-cali.json` | Staging pose for board-visible cali-vise relocalization before any vise interaction. |
| `Plate` | `Plate.json` | Camera staging pose above the pickup plate. |
| `Vise` | `Vise.json` | Vise/loading contact pose. |
| `plastic` | `plastic.json` | Drop pose for plastic / post-cut injectable piece. |
| `spring` | `spring.json` | Spring handling or disposal pose; confirm intended use. |
| `glass` | `glass.json` | Glass/body handling or disposal pose; confirm intended use. |

### Vise Calibration Preflight

Keep the numbered teardown CLI steps stable. The recipe should treat
`cali-vise` as a **preflight phase between steps 2 and 3**, not as a new
numbered `--step` target.

The flow is:

1. Move `Home -> Vise-cali`.
2. Run the existing `record_robot_waypoints.cali_vise()` helper.
3. Save the updated `Vise.json` in this recipe's `key_positions/` directory.
4. Reload the new `Vise` pose into the running recipe context.
5. Return `Vise-cali -> Home`.

Calibration assets:

- Board config: `project/calibration/tag_01.yaml`
- Eye-in-hand calibration: `project/calibration/camera_tcp.yaml`
- Board-to-vise reference: `project/calibration/tag_01_to_vise_tcp.json`

The recipe can synthesize approach and clearance poses from captured poses by
offsetting world Z:

- `pickup_lift = current world Z + 20 cm` after the adaptive grasp
- `above_vise = Vise world Z + 5 cm`
- `vise_retreat = current world Z + 15 cm`
- `cap_lift = current world Z + 20 cm`

World Z is the explicit approach / retreat convention for this plan.

For vise insertion, use Flexiv's official `InsertComp` primitive from the
[Adaptive Assembly family](https://primitive.flexiv.com/primitives/en/3.11/rizon4/Adaptive%20Assembly.html#insertcomp):

- The recipe intent is **negative world Z** insertion.
- Flexiv `InsertComp` takes `insertAxis` in the TCP coordinate system, so the
  recipe resolves world `-Z` to the matching signed TCP axis at runtime.
- Current captured `Vise` orientation resolves world `-Z` to TCP `X`.
- Wait for the primitive's default transition condition: `isMoving == 0`.
- Keep the initial force conservative until tuned on hardware.

### Subsystems

Flexiv robot:

- Serial: `Rizon4-062930`
- Gripper: `Flexiv-GN01`
- Default motion: task-space-only `MovePTP` to captured/synthesized TCP poses,
  with camera alignment + adaptive grasp for pickup and `InsertComp` for vise insertion
- Captured joint angles are stored for reference, but GreenPink `MovePTP` does
  not pass them as IK seeds by default; Flexiv's solver should choose the
  efficient joint path.
- Safe pose: `Inter`
- Helper module: `project/helper/flexiv_helpers.py`

Arduino cutting machine:

- Serial protocol: line-based ASCII, 115200 baud
- Python client: `project/helper/arduino_client.py`
- Commands needed for this objective:
  - `GET_STATUS`
- `HOME_ALL`
- `CLOSE_VISE`
- `OPEN_VISE`
- `CUT_HEIGHT`
- `STOP_ALL` on fault

### Translation Table

| Human step | Python / robot action | Arduino action | Notes |
| --- | --- | --- | --- |
| 1 | Start `RobotSession`, setup gripper, move `Inter` | Connect / status check if Arduino enabled | Startup gate. |
| 2 | Move to `Home` | None | Panel-clear machine entry waypoint. |
| 2a | Move to `Vise-cali`, run `cali-vise`, reload `Vise`, return `Home` | None | Preflight relocalization; keeps later vise approach/insert/grip steps aligned to the current board pose. |
| 3 | Move to `Plate` camera staging pose | None | Camera pickup staging pose above the plate. |
| 4 | Run camera `Align Injectable` | None | Uses RealSense RGB-D, hand-eye calibration, 2 cm offset toward pink, TCP +X toward green/teal. |
| 5 | Run adaptive `Grasp` | None | Open -> pre-contact MoveL -> ZeroFTSensor -> Contact -> GraspComp. |
| 6 | Lift by positive world Z +20 cm | None | Straight vertical retreat after pickup. |
| 7 | Move to `Home` | None | Pickup-side transfer waypoint. |
| 8 | Move to `Inter` | None | Machine-side transfer waypoint. |
| 9 | Move to synthesized `above_vise` | None | `Vise` world Z +5 cm. |
| 10 | Execute Flexiv `InsertComp` | None | TCP-frame insertion; wait for `isMoving == 0`. |
| 11 | Hold part in gripper | `CLOSE_VISE` | Robot keeps custody until vise confirms closed. |
| 12 | Switch to joint hold, then open gripper to 5 cm width | None | Only after `CLOSE_VISE DONE`; avoids lingering `InsertComp` compliance during release. |
| 13 | Open gripper fully in joint hold, switch back to primitive motion, retreat by world Z +15 cm, wait | None | Clear vertical wait pose before cutting. |
| 14 | Robot holds still | `CUT_HEIGHT z=133 x=110.5 deg=360` | Robot must be clear. |
| 15 | Move down in world Z to `Vise` | None | Only after cut final state is safe. |
| 16 | Close gripper at 80 N | None | Wait until stopped, then hold `GRIPPER_SETTLE_S` before twisting/lifting. |
| 17 | Twist around TCP X | None | Rotate 5 degrees, then return to start orientation. |
| 18 | Lift by positive world Z +20 cm | None | Release cap/top from the body; this direction is correct for lifting out. |
| 19 | Move to `Inter` | None | Machine-side safe exit after cap lift. |
| 20 | Move to `Home` | None | Pickup-side transfer before drop. |
| 21 | Move to `plastic` | None | Drop location. |
| 22 | Open gripper | None | Release cap/top. |

### Proposed Phase Structure For `recipe.py`

Use phase functions that map directly to the human objective:

1. `phase_startup()`
   - Load key positions.
   - Validate required files and pose fields.
   - Connect robot.
   - Setup gripper.
   - Optionally connect / home Arduino.

2. `phase_startup_transfer()`
   - `Inter`
   - `Home`

3. `phase_calibrate_vise()`
   - move to `Vise-cali`
   - run `cali-vise`
   - reload `Vise.json`
   - return to `Home`

4. `phase_pick_from_camera_align_grasp()`
   - move to `Plate` camera staging pose
   - run `Align Injectable`
   - run adaptive `Grasp`
   - lift by positive world Z +20 cm
   - `Home`
   - `Inter`

5. `phase_load_vise()`
   - synthesized `above_vise`
   - `InsertComp` into the vise
   - `CLOSE_VISE`
   - switch from `InsertComp` compliance to joint hold
   - open gripper to 5 cm width
   - open fully
   - switch back to primitive motion
   - retreat by 15 cm

6. `phase_cut()`
   - verify robot clear
   - `CUT_HEIGHT x=110.5 z=133 deg=360`
   - verify blade off, X/Z returned home, and rotary is within tolerance

7. `phase_twist_and_drop_cap()`
   - move back down to captured `Vise`
   - close gripper and wait for firm grip settle
   - twist around TCP X by 5 degrees, then return to start orientation
   - lift by positive world Z +20 cm
   - move `Inter`
   - move `Home`
   - move to `plastic`
   - open gripper

8. `phase_shutdown()`
   - open gripper
   - verify Arduino not busy and blade off
   - `RobotSession` exits and stops robot

Recipe controls for this preflight:

- default full recipe: run vise calibration before pickup
- `--skip-vise-calibration`: skip the preflight and use the saved `Vise.json`
- `--vise-calibration-only`: run only `Inter -> Home -> Vise-cali -> cali-vise -> Home`

### Robot-Only Smoke

`--robot-only-smoke` runs every robot/gripper step in the numbered plan while
skipping only the Arduino actions:

- skips step 11 `CLOSE_VISE`
- still runs step 12, the clear/wait pose for cutting
- skips step 13 `CUT_HEIGHT`
- still runs steps 14-20: Vise re-entry, cap close, twist, positive world-Z
  cap lift, `Middle`, `plastic`, and gripper open

Use this with a safe test setup because the vise and cutting machine are not
holding or cutting the part in this mode.

### Single-Step Safety Checks

`recipe.py` can run one numbered plan step at a time:

```bash
python3 project/recipe/GreenPinkCamera/recipe.py --dry-run --step 14
python3 project/recipe/GreenPinkCamera/recipe.py --step 14
python3 project/recipe/GreenPinkCamera/recipe.py --dry-run --step 17
```

Use this to verify motion safety from the expected pre-step state before
running the full recipe. Step 13 (`CUT_HEIGHT`) still enforces the vise-closed
machine state and still asks for the `CUT` confirmation unless `--yes` is used.

### Safety Gates

Before robot enters vise area:

- Arduino `busy=false`
- Arduino `faulted=false`
- Arduino `blade_on=false`
- Arduino is homed, or recipe has just run `HOME_ALL`

Before cut:

- Robot is at the step-12 clear/wait pose or `Middle`
- Vise has confirmed `CLOSED`
- Force reading is acceptable
- Rotary is at `0` or has been reset with `ROTATE_ABS deg=0`

After cut:

- `CUT_HEIGHT` returned `DONE`
- `blade_on=false`
- `x_mm=0`
- `z_mm=0`
- `abs(rot_deg) <= ROT_SAFE_TOL_DEG`
- `busy=false`
- `faulted=false`

Fault handling:

- If robot faults, stop recipe and call Arduino `STOP_ALL` if connected.
- If Arduino returns `ERR` or times out, call `STOP_ALL`, keep robot clear, and
  end the run for operator inspection.
- No automatic retries in V1.

### Parameters To Put At Top Of `recipe.py`

```python
PARAMS = {
    "ROBOT_SN": "Rizon4-062930",
    "GRIPPER_NAME": "Flexiv-GN01",
    "KEY_POSITION_DIR": "key_positions",
    "MOVE_JNT_VEL_SCALE": 30,
    "MOVE_USE_REF_JOINTS": False,
    "CARTESIAN_INSERT_VEL_M_S": 0.06,
    "CARTESIAN_RETREAT_VEL_M_S": 0.06,
    "GRIPPER_OPEN_WIDTH_M": 0.06,
    "GRIPPER_RELEASE_WIDTH_M": 0.05,
    "GRIPPER_CLOSE_WIDTH_M": 0.0,
    "GRIPPER_VELOCITY_M_S": 0.05,
    "GRIPPER_FORCE_N": 80.0,
    "GRIPPER_SETTLE_S": 1.0,
    "GRIPPER_OPEN_SETTLE_S": 1.5,
    "TRAY_APPROACH_Z_OFFSET_M": 0.05,
    "TRAY_GRIP_Z_OFFSET_M": 0.0,
    "VISE_APPROACH_Z_OFFSET_M": 0.05,
    "VISE_RETREAT_Z_OFFSET_M": 0.15,
    "INSERTCOMP_INSERT_AXIS": "AUTO_WORLD_NEG_Z",
    "INSERTCOMP_COMP_AXIS": [0, 1, 1, 0, 0, 0],
    "INSERTCOMP_MAX_CONTACT_FORCE_N": 8.0,
    "INSERTCOMP_DEADBAND_SCALE": 80.0,
    "INSERTCOMP_INSERT_VEL_M_S": 0.01,
    "INSERTCOMP_COMP_VEL_SCALE": 20.0,
    "INSERTCOMP_TIMEOUT_S": 20.0,
    "CUT_X_MM": 110.5,
    "CUT_Z_MM": 133.0,
    "CUT_DEG": 360.0,
    "ROT_SAFE_TOL_DEG": 0.2,
    "CAP_GRIP_Z_OFFSET_M": 0.0,
    "CAP_TWIST_DEG": 5.0,
    "CAP_TWIST_REPEAT_COUNT": 1,
    "CAP_LIFT_Z_OFFSET_M": 0.20,
}
```

### Open Questions For Tuning

- Is `Vise` already the final inserted pose?
- Does `AUTO_WORLD_NEG_Z` still resolve correctly after any future vise-pose
  recapture?
- What `InsertComp` `maxContactForce` is enough to seat the injectable without
  pushing too hard?
- What gripper close width should pair with the 80 N force?
- Is `plastic` the final drop location for the cap/top removed after cutting?
- Is a single 5 degree TCP-X twist enough to break the cap loose?
- What are `spring` and `glass` intended to do in this version?

### Minimum First Code Pass

Build `recipe.py` in this order:

1. Dry-run loader that prints required positions and the translated phase order.
2. Robot-only motion:
   `Inter -> Home -> Plate -> Align Injectable -> Grasp -> pickup_lift -> Home -> Inter -> above_vise -> InsertComp`.
3. Add gripper open/close.
4. Add Arduino status and `HOME_ALL`.
5. Add `CLOSE_VISE`.
6. Add `CUT_HEIGHT`.
7. Add TCP-X twist, world-Z lift, and drop at `plastic`.
8. Add single-step execution with `recipe.py --step <N>` so each numbered plan
   step can be checked independently before running the full recipe.
