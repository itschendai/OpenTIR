# LightGreen Injectable Teardown

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
13. Machine starts cut height primitive `cut_height z_mm=120 x_mm=111.5 deg=360` to remove the top.
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
31. Machine starts cut height primitive `cut_height z_mm=132.5 x_mm=111.5 deg=360` to free the yellow plastic.
32. Robot moves down to a bit below Vise position (Vise world Z -3.8 cm).
33. Robot closes gripper to 80 N.
34. Robot slowly raises up in **positive world Z** by 10 cm to release the yellow plastic.
35. Robot moves to Middle point.
36. Robot moves to Plastic point.
37. Robot opens gripper.


Remove shell and glass:
38. Robot moves to Middle point.
39. Robot moves to above vise position (Vise world Z +10 cm).
40. Robot moves down to a bit below Vise position (Vise world Z -6 cm).
41. Robot closes gripper to 80 N.
42. Machine opens vise.
43. Robot slowly raises up in **positive world Z** by 20 cm to release cap.
44. Robot moves to Middle point.
45. Robot moves to Glass point.
46. The robot performs Dump with one continuous MoveC arc around an upper-left virtual point in the tool frame (Rz 176 and then turn back).
47. Robot moves to Plastic point.
48. Robot opens gripper.
49. Robot moves to Middle point.

## Implementation Workspace

Everything below this line is our shared translation layer for turning the
objective into code.

### Current Interpretation

The current objective describes a first integrated teardown:

1. Start at `Middle`.
2. Run the tag relocalization preflight by moving to `tag_1`, `tag_2`, and
   `tag_3`; when the calibration flag is enabled, run `cali tag N` at each
   staging pose and refresh the saved `Vise`, `Plate`, `Spring`, `Plastic`, and
   `Glass` key positions before returning to `Middle`.
3. Move to captured `Plate`.
4. Run camera-based `Align Injectable`, then adaptive `Grasp`.
5. Lift the whole injectable straight up in positive world Z, then return to
   `Middle`.
6. Load it into the vise from `above_vise` using Flexiv `InsertComp`.
7. Let the Arduino close the vise.
8. Exit the compliant insertion hold, park the arm in joint hold for the
   gripper-open actions, then switch back to motion primitive control and
   retreat to a safe waiting pose.
9. Let the Arduino run the first `CUT_HEIGHT` with `z=120`, `x=111.5`,
   `deg=360` to remove the top.
10. Return to `Vise`, grip the cap, twist around TCP X, lift it out in
    positive world Z, then drop it at `Plastic`.
11. Return to `Vise`, remove the spring, and drop it at `Spring`.
12. Let the Arduino run the second `CUT_HEIGHT` with `z=132.5`, `x=111.5`,
    `deg=360` to free the yellow plastic, then remove that piece and drop it at
    `Plastic`.
13. Return to `Vise` one final time, grip the remaining shell/glass assembly,
    open the vise, lift it out, dump the glass at `Glass`, then release the
    remaining plastic at `Plastic` and return to `Middle`.

This means `recipe.py` is not just robot playback. It coordinates the Flexiv
robot, the GN01 gripper, and the Arduino cutting machine.

### Captured Key Positions

Key positions live in `key_positions/`.

| Name | File | Current use |
| --- | --- | --- |
| `Middle` | `Middle.json` | Shared safe / transfer pose between pickup and vise actions. |
| `Plate` | `Plate.json` | Camera staging pose above the pickup plate. |
| `Vise` | `Vise.json` | Vise/loading contact pose. |
| `Spring` | `Spring.json` | Drop pose for the removed spring. |
| `Plastic` | `Plastic.json` | Drop pose for plastic / post-cut injectable pieces. |
| `Glass` | `Glass.json` | Drop pose for the glass/body handling pass. |
| `tag_1` | `tag_1.json` | Calibration staging pose that refreshes `Vise`. |
| `tag_2` | `tag_2.json` | Calibration staging pose that refreshes `Plate`. |
| `tag_3` | `tag_3.json` | Calibration staging pose that refreshes `Spring`, `Plastic`, and `Glass`. |

### Tag Calibration Preflight

Keep the numbered teardown CLI steps stable. The recipe should treat
tag calibration as a **preflight phase between steps 2 and 3**, not as a new
numbered `--step` target.

The flow is:

1. Move `Middle -> tag_1`.
2. If calibration is enabled, run `cali tag 1`, save the updated `Vise.json`
   into this recipe's `key_positions/` directory, and reload it into the
   running recipe context.
3. Move `tag_1 -> tag_2`.
4. If calibration is enabled, run `cali tag 2`, save the updated `Plate.json`,
   and reload it.
5. Move `tag_2 -> tag_3`.
6. If calibration is enabled, run `cali tag 3`, save the updated `Spring.json`,
   `Plastic.json`, and `Glass.json`, and reload them.
7. Return `tag_3 -> Middle`.

Calibration assets:

- Board config: `project/calibration/tag_01.yaml`
- Eye-in-hand calibration: `project/calibration/camera_tcp.yaml`
- Board-to-vise reference: `project/calibration/tag_01_to_vise_tcp.json`

The recipe can synthesize approach and clearance poses from captured poses by
offsetting world Z:

- `pickup_lift = current world Z + 20 cm` after the adaptive grasp
- `above_vise = Vise world Z + 5 cm`
- `vise_retreat = current world Z + 15 cm`
- `spring_remove_above_vise = Vise world Z + 10 cm`
- `yellow_remove_grip = Vise world Z - 3.8 cm`
- `shell_remove_grip = Vise world Z - 6 cm`
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
- Captured joint angles are stored for reference, but LightGreen `MovePTP` does
  not pass them as IK seeds by default; Flexiv's solver should choose the
  efficient joint path.
- Safe pose: `Middle`
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
| 1 | Start `RobotSession`, setup gripper, open gripper | Connect / status check if Arduino enabled | Startup gate. |
| 2 | Move to `Middle` | None | Shared safe pose before pickup or vise work. |
| 2a | Move to `tag_1`, `tag_2`, `tag_3`, optionally run `cali tag N`, reload updated poses, return `Middle` | None | Preflight relocalization keeps `Vise`, `Plate`, `Spring`, `Plastic`, and `Glass` aligned to the current board pose. |
| 3 | Move to `Plate` camera staging pose | None | Camera pickup staging pose above the plate. |
| 4 | Run camera `Align Injectable` | None | Uses RealSense RGB-D, hand-eye calibration, 2 cm offset toward pink, TCP +X toward green/teal. |
| 5 | Run adaptive `Grasp` | None | Open -> pre-contact MoveL -> ZeroFTSensor -> Contact -> GraspComp. |
| 6 | Lift by positive world Z +20 cm | None | Straight vertical retreat after pickup. |
| 7 | Move to `Middle` | None | Safe transfer back out of pickup. |
| 8 | Move to synthesized `above_vise` | None | `Vise` world Z +5 cm. |
| 9 | Execute Flexiv `InsertComp` | None | TCP-frame insertion; wait for `isMoving == 0`. |
| 10 | Hold part in gripper | `CLOSE_VISE` | Robot keeps custody until vise confirms closed. |
| 11 | Switch to joint hold, then open gripper to 5 cm width | None | Only after `CLOSE_VISE DONE`; avoids lingering `InsertComp` compliance during release. |
| 12 | Open gripper fully in joint hold, switch back to primitive motion, retreat by world Z +15 cm, wait | None | Clear vertical wait pose before the first cut. |
| 13 | Robot holds still | `CUT_HEIGHT z=120 x=111.5 deg=360` | First cut removes the top. |
| 14-20 | Re-enter `Vise`, grip/twist/lift cap, move `Middle -> Plastic`, open gripper | None | Cap/top removal and drop. |
| 21-28 | Re-enter `Vise`, grip/lift spring, move `Middle -> Spring`, open gripper | None | Spring removal and drop. |
| 29-30 | Move `Middle -> above_vise` for yellow-plastic removal | None | Stage for the second cut. |
| 31 | Robot holds still | `CUT_HEIGHT z=132.5 x=111.5 deg=360` | Second cut frees the yellow plastic. |
| 32-37 | Grip yellow plastic at `Vise z=-3.8 cm`, lift, move `Middle -> Plastic`, open gripper | None | Yellow-plastic removal and drop. |
| 38-42 | Re-enter `Vise`, grip shell at `Vise z=-6 cm`, then lift clear | `OPEN_VISE` at step 42 | Frees the remaining shell/glass assembly. |
| 43-49 | Move `Middle -> Glass`, dump, move `Plastic`, open gripper, return `Middle` | None | Final shell/glass separation and cleanup. |

### Proposed Phase Structure For `recipe.py`

Use phase functions that map directly to the human objective:

1. `phase_startup()`
   - Load key positions.
   - Validate required files and pose fields.
   - Connect robot.
   - Setup gripper.
   - Optionally connect / home Arduino.

2. `phase_startup_transfer()`
   - open gripper
   - move to `Middle`

3. `phase_calibrate_tags()`
   - move to `tag_1`, `tag_2`, and `tag_3`
   - optionally run `cali tag N`
   - reload any refreshed key positions
   - return to `Middle`

4. `phase_pick_from_camera_align_grasp()`
   - move to `Plate` camera staging pose
   - run `Align Injectable`
   - run adaptive `Grasp`
   - lift by positive world Z +20 cm
   - return to `Middle`

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
   - `CUT_HEIGHT x=111.5 z=120 deg=360`
   - verify blade off, X/Z returned home, and rotary is within tolerance

7. `phase_twist_and_drop_cap()`
   - move back down to captured `Vise`
   - close gripper and wait for firm grip settle
   - twist around TCP X by 5 degrees, then return to start orientation
   - lift by positive world Z +20 cm
   - move `Middle`
   - move to `Plastic`
   - open gripper

8. `phase_remove_spring()`
   - move `Middle -> above_vise -> Vise`
   - close gripper on the spring and lift by positive world Z +15 cm
   - move `Middle -> Spring`
   - open gripper

9. `phase_cut_and_remove_yellow_plastic()`
   - move `Middle -> above_vise`
   - `CUT_HEIGHT x=111.5 z=132.5 deg=360`
   - move down to `Vise z=-3.8 cm`
   - close gripper and lift by positive world Z +10 cm
   - move `Middle -> Plastic`
   - open gripper

10. `phase_remove_shell_and_glass()`
   - move `Middle -> above_vise -> Vise z=-6 cm`
   - close gripper on the remaining shell/glass assembly
   - `OPEN_VISE`
   - lift by positive world Z +20 cm
   - move `Middle -> Glass`
   - run `Dump`
   - move `Plastic`
   - open gripper
   - return to `Middle`

11. `phase_shutdown()`
   - open gripper
   - verify Arduino not busy and blade off
   - `RobotSession` exits and stops robot

Recipe controls for this preflight:

- default full recipe: run tag calibration before pickup
- `--skip-tag-calibration`: skip the relocalization pass and use the saved key positions
- `--tag-calibration-only`: run only `Middle -> tag_1 -> tag_2 -> tag_3 -> Middle`

### Robot-Only Smoke

`--robot-only-smoke` runs every robot/gripper step in the numbered plan while
skipping only the Arduino actions:

- skips step 10 `CLOSE_VISE`
- still runs step 12, the clear/wait pose for cutting
- skips step 13, the first `CUT_HEIGHT`
- skips step 31, the yellow-plastic `CUT_HEIGHT`
- skips step 42 `OPEN_VISE`
- still runs the surrounding robot-only removal motions for cap, spring, yellow
  plastic, and shell/glass handling

Use this with a safe test setup because the vise and cutting machine are not
holding or cutting the part in this mode.

### Single-Step Safety Checks

`recipe.py` can run one numbered plan step at a time:

```bash
python3 project/recipe/LightGreen/recipe.py --dry-run --step 13
python3 project/recipe/LightGreen/recipe.py --dry-run --step 31
python3 project/recipe/LightGreen/recipe.py --dry-run --step 40
```

Use this to verify motion safety from the expected pre-step state before
running the full recipe. Steps 13 and 31 (`CUT_HEIGHT`) still enforce the
vise-closed machine state and still ask for the `CUT` confirmation unless
`--yes` is used.

### Safety Gates

Before robot enters vise area:

- Arduino `busy=false`
- Arduino `faulted=false`
- Arduino `blade_on=false`
- Arduino is homed, or recipe has just run `HOME_ALL`

Before each cut:

- Robot is at the step-12 clear/wait pose or `Middle`
- Vise has confirmed `CLOSED`
- Force reading is acceptable
- Rotary is at `0` or has been reset with `ROTATE_ABS deg=0`

After each cut:

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
    "PICKUP_LIFT_Z_OFFSET_M": 0.20,
    "VISE_APPROACH_Z_OFFSET_M": 0.05,
    "VISE_RETREAT_Z_OFFSET_M": 0.15,
    "SPRING_REMOVE_APPROACH_Z_OFFSET_M": 0.10,
    "SPRING_REMOVE_GRIP_Z_OFFSET_M": 0.0,
    "SPRING_REMOVE_LIFT_Z_OFFSET_M": 0.15,
    "YELLOW_REMOVE_APPROACH_Z_OFFSET_M": 0.10,
    "YELLOW_REMOVE_GRIP_Z_OFFSET_M": -0.038,
    "YELLOW_REMOVE_LIFT_Z_OFFSET_M": 0.10,
    "SHELL_REMOVE_APPROACH_Z_OFFSET_M": 0.10,
    "SHELL_REMOVE_GRIP_Z_OFFSET_M": -0.06,
    "SHELL_REMOVE_LIFT_Z_OFFSET_M": 0.20,
    "INSERTCOMP_INSERT_AXIS": "AUTO_WORLD_NEG_Z",
    "INSERTCOMP_COMP_AXIS": [0, 1, 1, 0, 0, 0],
    "INSERTCOMP_MAX_CONTACT_FORCE_N": 8.0,
    "INSERTCOMP_DEADBAND_SCALE": 80.0,
    "INSERTCOMP_INSERT_VEL_M_S": 0.01,
    "INSERTCOMP_COMP_VEL_SCALE": 20.0,
    "INSERTCOMP_TIMEOUT_S": 20.0,
    "CUT_X_MM": 111.5,
    "CUT_Z_MM": 120.0,
    "CUT_DEG": 360.0,
    "YELLOW_CUT_X_MM": 111.5,
    "YELLOW_CUT_Z_MM": 132.5,
    "YELLOW_CUT_DEG": 360.0,
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
- Is `Plastic` the final drop location for the cap/top removed after cutting?
- Is a single 5 degree TCP-X twist enough to break the cap loose?
- Are `Spring` and `Glass` the final intended drop locations in this version?

### Minimum First Code Pass

Build `recipe.py` in this order:

1. Dry-run loader that prints required positions and the translated phase order.
2. Robot-only motion:
   `Middle -> Plate -> Align Injectable -> Grasp -> pickup_lift -> Middle -> above_vise -> InsertComp`.
3. Add gripper open/close.
4. Add Arduino status and `HOME_ALL`.
5. Add `CLOSE_VISE`.
6. Add both `CUT_HEIGHT` actions.
7. Add TCP-X twist, world-Z lift, and drops at `Plastic`, `Spring`, and `Glass`.
8. Add single-step execution with `recipe.py --step <N>` so each numbered plan
   step can be checked independently before running the full recipe.
