# Eye-in-Hand Calibration — Handoff & Positioning Issue

## TL;DR (Human Summary)

The eye-in-hand calibration is **complete and validated** — `camera_tcp.yaml` is correct (held-out board-spread max 1.25 mm, well within the training-time error). The `verify` functional test correctly computes where the TCP should land (100 mm along the ChArUco board normal, TCP approach axis pointing back at the board), but **the arm crashes into itself when MovePTP tries to reach that pose**.

This is **not a calibration bug**. It's a workspace geometry problem: the board is mounted close to the robot base, at an awkward angle, and the 100 mm standoff target is in a region where the only IK solutions require self-colliding arm configurations. The Flexiv planner does not do online self-collision avoidance during `MovePTP`.

**Quick paths forward (pick the cheapest that works):**

1. **Move the board** to a more open location with 200+ mm of free space in front of the marked face along its normal. No re-calibration needed — `camera_tcp.yaml` is mount-dependent, not workspace-dependent.
2. **Increase standoff**: `--distance-mm 250` or `300` to give the arm room to swing.
3. **Try the other approach axis**: `--approach-axis -z` (with `--dry-run` first to confirm the new target looks sane).
4. **Pre-position the arm closer to a valid pose** before pressing `c`, so MovePTP has a short, safe trajectory.
5. **Skip the motion test entirely**: verify the calibration via multi-view consistency (capture from 2–3 angles, confirm predicted `T_world_board` agrees across views — `solve` and `validate` already do this implicitly).

Calibration is good. The visual "move to the tag" test is just hard to perform safely in this particular physical setup.

---

## Current State of the Project

### Files
| Path | Purpose |
|---|---|
| `calibrate_eye_in_hand.py` | Main script. Subcommands: `preview`, `capture`, `solve`, `validate`, `verify`. |
| `tag_01.yaml` | ChArUco board definition (squares, marker length, dictionary, IDs). |
| `camera_tcp.yaml` | Solved calibration result: `T_tcp_camera` 4×4 transform, plus quality stats. |
| `samples/` | 10 training samples used for the current solve. |
| `holdout/` | 9 held-out samples used for validation. |
| `preview_snapshots/` | (May or may not exist) annotated frames the user saved via the `s` key. |
| `HANDOFF.md` | This document. |

### Calibration quality (as of last solve / validate)

| Metric | Training (max) | Held-out (max) | Threshold | Verdict |
|---|---|---|---|---|
| Translation spread | 1.52 mm | 1.25 mm | ≤ 3 mm | Pass |
| Rotation spread | 0.500° | 0.606° | ≤ 1° | Pass |
| Reprojection error | 0.224 px | 0.347 px | ≤ 0.5 px | Pass |

The held-out translation spread is slightly **better** than training, which strongly indicates the solver did not overfit.

The calibration's translation component (`T_tcp_camera[:3, 3]`) is `[-0.0779, +0.0120, -0.0976] m` (camera origin expressed in TCP frame, total offset ~12.5 cm). The axis check shows camera optical +Z is 20.1° from the expected `[0, 0, 1]` in TCP — within the 45° default tolerance.

### Environment
- **Python**: `/home/src0/flexiv_rdk/.venv/bin/python` is the only Python on this system that has `cv2` (opencv-python 4.13), `pyrealsense2`, `flexivrdk` 1.9.0, `spdlog`. **The system `python3` does NOT.** Always invoke the venv Python explicitly.
- **Robot**: Flexiv Rizon 4 (7-DOF), SN `Rizon4-062930`.
- **Camera**: Intel RealSense D405, serial `323622271112`, mounted near the TCP (eye-in-hand).
- **Flexiv RDK**: 1.9.0 (v1.9 API, not v2). `Mode` enum has IDLE, NRT_PRIMITIVE_EXECUTION, NRT_JOINT_POSITION, NRT_JOINT_IMPEDANCE, NRT_CARTESIAN_MOTION_FORCE, NRT_PLAN_EXECUTION, NRT_SUPER_PRIMITIVE, UNKNOWN.

### Calibration workflow as it stands

```bash
# 1) Live preview with ChArUco overlay; robot enters FloatingJoint so user
#    can hand-position the arm. Press 'c' to save a sample.
/home/src0/flexiv_rdk/.venv/bin/python calibrate_eye_in_hand.py preview

# 2) (Alternate, one-shot prompted capture without preview)
/home/src0/flexiv_rdk/.venv/bin/python calibrate_eye_in_hand.py capture --prompt

# 3) Solve hand-eye from samples/, write camera_tcp.yaml
/home/src0/flexiv_rdk/.venv/bin/python calibrate_eye_in_hand.py solve

# 4) Validate against a held-out sample directory
/home/src0/flexiv_rdk/.venv/bin/python calibrate_eye_in_hand.py validate \
    --samples-dir holdout

# 5) Functional test: 'c' captures + computes target, 'g' moves robot, 'x'
#    cancels, 'q' quits. ALWAYS dry-run first.
/home/src0/flexiv_rdk/.venv/bin/python calibrate_eye_in_hand.py verify --dry-run
/home/src0/flexiv_rdk/.venv/bin/python calibrate_eye_in_hand.py verify
```

---

## The Positioning Issue (Detailed)

### Symptom
After successfully validating the calibration, running `verify` and pressing `g` to execute the computed MovePTP causes **the arm to collide with itself (or with another arm link / base) before reaching the target pose**. The target itself is geometrically correct — verified both with synthetic unit tests on `compute_verify_target` and by visual inspection of the printed predicted pose.

### Root cause

The physical mounting of the ChArUco board creates a target pose that the Flexiv planner cannot reach without an unsafe trajectory:

1. **Board is close to the robot base.** Cartesian space between board and arm body is small.
2. **Board normal points into a constrained region.** The 100 mm-standoff target sits in a corner of the workspace where IK has limited valid joint configurations.
3. **MovePTP plans in joint space**, choosing a configuration that respects joint limits and the static obstacle model on the controller. It does **not** perform online self-collision avoidance for arbitrary task targets. So it will pick a joint trajectory that swings the arm through a self-intersecting configuration on the way to the target.
4. **The 100 mm standoff is tight**, putting the TCP very near the board (and whatever surface the board is mounted on), amplifying the geometric difficulty.

### Why it didn't surface during calibration
- `preview`, `capture`: only *read* the TCP pose. No motion commanded. The arm was always in user-positioned, by-definition-reachable poses.
- `solve`, `validate`: pure data processing, no robot involvement.
- `verify` is the first subcommand that actually issues a motion. It's also the first time the controller is asked to drive the arm to a programmatically computed target it didn't get to "demonstrate" beforehand.

---

## What's Already Been Done (Do Not Re-investigate)

- **Sign of the standoff direction**: `compute_verify_target` correctly picks the camera-facing side of the board (uses `T_world_camera` position and `dot(camera-board, board_+Z)` to choose `±board_+Z`). Verified with synthetic tests in both Z-orientations. The earlier "moves to opposite side" bug is fixed.
- **Floating ↔ motion transitions**: `command_verify` now calls `session.robot.Stop()` before `SwitchMode("NRT_PRIMITIVE_EXECUTION")` and before re-entering `FloatingJoint`. Mirrors the `stop_floating()` / `start_joint_floating()` pattern from `record_robot_waypoints.py`.
- **Float parameters**: `FLOATING_DAMPING_LEVEL`, `FLOATING_RESPONSE_TORQUE`, `FLOATING_LOAD_COMPENSATION_SCALE` are byte-for-byte copies of `record_robot_waypoints.py`. If the float feels under-compensated (arm droops), the cause is on the controller (Tool/payload definition or F/T sensor zero), not in this script.
- **Yaw convention**: Option 1 (`target_R = board_R @ R_local` with `R_local` = shortest rotation aligning the approach axis to the standoff direction) — i.e., target frame matches board axes with the chosen approach axis flipped to point at the board.

---

## Recommended Next Steps (in priority order)

### 1. Move the board (highest expected value)
Mount it on a wall, easel, or vertical fixture in a location where:
- 200+ mm of free Cartesian space exists in front of the marked face along the normal.
- The normal does not point at the robot body or another arm link.
- The arm can plausibly approach face-on without folding back through itself.

**No re-calibration needed.** `T_tcp_camera` describes the rigid camera-to-TCP mount; it is independent of where the board lives in the workspace. After moving, re-run `verify` directly.

### 2. Increase standoff distance
```bash
calibrate_eye_in_hand.py verify --distance-mm 250
```
or even 300. More distance = more arm-swing room. Try first with `--dry-run` to confirm the target is now in a reachable area.

### 3. Try the other approach axis
```bash
calibrate_eye_in_hand.py verify --approach-axis -z --dry-run
```
If TCP +Z currently lands the wrist in a tight configuration, flipping might help. The choice depends on how the camera and any gripper are physically mounted relative to the TCP.

### 4. Add an intermediate via-point (code change, ~30 LOC)
Before MovePTP to the final target, MovePTP to a midway pose along the standoff axis (e.g., `--approach-via-mm 200` first, then to `--distance-mm 100`). Forces the planner to break the trajectory into two safer chunks. Implementation: in the `'g'` key handler of `command_verify`, compute a via target by adjusting `distance_m` and call MovePTP twice.

### 5. Switch to MoveL (Cartesian linear)
MovePTP plans in joint space and may swing wildly. MoveL forces a Cartesian-linear path. Helpful when start and end are spatially close. Less helpful when they aren't.

### 6. Abandon the visual-motion test; validate by multi-view consistency
You can demonstrate the calibration is correct without commanding any motion:
- Capture from 2–3 different viewpoints.
- For each, compute `T_world_board = T_world_tcp @ T_tcp_camera @ T_camera_board`.
- Confirm all views give the same `T_world_board` to within the validation spread (~1 mm).

This is essentially what `validate` already does, internally. The fact that it passes with 1.25 mm max spread is strong evidence the calibration is correct *independent of whether you can physically reach a verify target*. The visual test was a confirmation, not a requirement.

---

## For the Next Claude Instance

### Quick orientation

This is a Flexiv Rizon 4 eye-in-hand calibration. A RealSense D405 is mounted near the TCP. A ChArUco board is fixed in the workspace. The script captures images + TCP poses at multiple views, solves for `T_tcp_camera` via `cv2.calibrateHandEye`, and lets the user functionally verify by commanding the arm to a known offset from the detected board.

**Critical session-specific facts**:
- All deps live in `/home/src0/flexiv_rdk/.venv/bin/python`. The system `python3` will fail with `ModuleNotFoundError`.
- The Flexiv helper is in `/home/src0/flexiv_rdk/project/helper/flexiv_helpers.py` (not `injectable_pipeline/` — that directory was renamed `injectable_pipeline_archived/`).
- The reference for floating + MovePTP patterns is `/home/src0/flexiv_rdk/project/record_robot_waypoints.py`. Read its `start_joint_floating`, `stop_floating`, `execute_primitive`, and `wait_for_primitive` functions when in doubt about motion control patterns.

### Code map (functions in `calibrate_eye_in_hand.py`, in approximate file order)

| Function | Purpose |
|---|---|
| `import_robot_session` / `import_flexiv_helpers` | Lazy import of the helper module (adds `project/helper/` to `sys.path`). |
| `quat_wxyz_to_matrix`, `matrix_to_quat_wxyz`, `pose_vec_to_transform`, `invert_transform`, `transform_from_rt` | Rigid-transform math. |
| `rotation_angle_deg`, `mean_rotation_matrix` | Used by `validate` for spread metrics. |
| `load_board_config`, `create_charuco_board`, `detect_charuco_corners`, `estimate_charuco_pose` | ChArUco detection. Cross-version OpenCV ArUco/Charuco API shims live here. |
| `realsense_devices`, `select_camera`, `configure_color_sensor`, `capture_realsense_color`, `intrinsics_to_yaml` | RealSense capture pipeline. |
| `read_current_tcp_pose` | One-shot pose read (opens/closes RobotSession). Used only by `command_capture`. |
| `collect_detections`, `iter_sample_yamls`, `detect_sample`, `sample_image_path`, `sample_tcp_pose`, `camera_matrix_and_dist` | Loading samples and running detection on each. |
| `solve_hand_eye_from_transforms` | Wraps `cv2.calibrateHandEye`. |
| `pose_spread_stats`, `validation_stats`, `camera_axis_check` | Quality metrics. |
| `save_calibration_sample` | Writes `samples/<id>/{color.png, sample.yaml}`. Shared by `command_capture` and `command_preview`. |
| `command_capture` | One-shot prompted capture (no live preview). |
| `enable_joint_floating` | Switches robot to `NRT_PRIMITIVE_EXECUTION` and runs `FloatingJoint` primitive. Uses identical params to `record_robot_waypoints.py`. |
| `command_preview` | Live RealSense window with ChArUco overlay; opens persistent RobotSession in FloatingJoint; `c` saves a sample, `s` saves an annotated snapshot, `q`/ESC quits. |
| `parse_approach_axis`, `rotation_align_unit_vectors`, `compute_verify_target`, `tcp_pose_from_pos_R` | `verify` math helpers. |
| `command_verify` | Live preview + two-step capture/move: `c` computes target, `g` executes MovePTP, `x` cancels, `q` quits. |
| `command_solve` | Reads samples/, runs detect + hand-eye solve, writes `camera_tcp.yaml`. |
| `command_validate` | Loads `camera_tcp.yaml`, re-runs spread/reprojection on given sample dir. |
| `build_parser`, `main` | CLI plumbing. |

### Key conventions

- **Transform notation**: `T_a_b` is the 4×4 transform that maps coordinates from frame `b` into frame `a`. I.e., `P_a = T_a_b @ P_b`. This is what the `transform_record` helper documents in saved YAMLs.
- **TCP pose vector**: 7 elements `[x, y, z, qw, qx, qy, qz]` in world (meters + unit quaternion, w-first). This matches `state.tcp_pose` from the Flexiv RDK.
- **Hand-eye solve inputs** (`cv2.calibrateHandEye`): `R/t_gripper2base` ← `T_world_tcp`; `R/t_target2cam` ← `T_camera_board`; returns `R/t_cam2gripper` which we label `T_tcp_camera`.
- **Eye-in-hand math chain**: `P_world = T_world_tcp @ T_tcp_camera @ P_camera`. Computed at capture-time TCP pose.
- **Board frame**: OpenCV's ChArUco board with origin at one corner, XY in the board plane, Z perpendicular. The sign of Z (which way points "out of the marked face") can vary by OpenCV version / `legacy_pattern` flag. `compute_verify_target` handles either sign by checking camera position relative to the board plane.

### Gotchas

1. **Tool/payload state lives on the controller, not in this script.** If `FloatingJoint` makes the arm droop, the most likely cause is that someone ran `ZeroFTSensor` or payload identification on the pendant in the wrong physical configuration, leaving the active Tool with a wrong/zero payload mass. Gravity comp then under-compensates and the arm sags. Fix on the pendant: select a correct Tool, or re-run payload identification with the actual end-effector mounted. The script never calls `Tool.Switch(...)`.

2. **Floating ↔ motion transitions need `robot.Stop()` between them.** Already done in `command_verify`. If you add new motion calls, mirror this pattern (see `record_robot_waypoints.py`'s `stop_floating` / `start_joint_floating` cycle).

3. **MovePTP has no online self-collision avoidance.** This is the root cause of the user's current positioning problem. Mitigations: shorter trajectory (pre-position closer to target), via-point (two-stage MovePTP), or use MoveL (Cartesian linear, more predictable path).

4. **OpenCV ArUco API varies across versions.** `detect_charuco_corners` and `create_charuco_board` have version shims for both the new `ArucoDetector`/`CharucoDetector` API and the older `cv2.aruco.detectMarkers` / `interpolateCornersCharuco` functions. Don't simplify these without testing on the actual installed OpenCV version (4.13 currently).

5. **`PROJECT_DIR = HERE.parent`** points at `/home/src0/flexiv_rdk/project`. The original file had a garbage path pasted into the assignment (`HERE.parent/media/src0/061C-013B/cv/boards/tag_01.yaml`) that caused a `SyntaxError`. Don't reintroduce that.

6. **The Flexiv helper directory** is `helper/`, not `injectable_pipeline/`. The latter was renamed `injectable_pipeline_archived/`. `import_robot_session` and `import_flexiv_helpers` know about this; if you change the path, update both.

7. **Always `--dry-run` before letting the user run a `verify` motion command** when modifying target-computation logic. A self-collision or out-of-bounds target costs real money.

### Synthetic test for `compute_verify_target`

```python
import sys; sys.path.insert(0, '/home/src0/flexiv_rdk/project/calibration')
import calibrate_eye_in_hand as ceh
import numpy as np

# Board at world (0.5, 0, 0.2) with +Z up, camera above the board.
T_world_board = np.eye(4); T_world_board[:3, 3] = [0.5, 0, 0.2]
camera_pos = np.array([0.5, 0, 0.5])
pos, R = ceh.compute_verify_target(
    T_world_board, ceh.parse_approach_axis('+z'), 0.100, camera_pos)
# Expected: pos = [0.5, 0, 0.3], R @ [0,0,1] = [0,0,-1] (TCP +Z points down at board)

# Same camera position but board flipped (its local +Z now points DOWN in world)
T_world_board[:3, :3] = np.diag([1, -1, -1])
pos, R = ceh.compute_verify_target(
    T_world_board, ceh.parse_approach_axis('+z'), 0.100, camera_pos)
# Expected: still pos = [0.5, 0, 0.3], R @ [0,0,1] still = [0,0,-1]
# — the function picks the camera-facing side regardless of board's local Z.
```

### What the user is most likely to ask for next

- **A safer motion mode** — implementing the via-point option (Recommendation 4 above) or switching to MoveL.
- **A `zero-ft` subcommand** — I offered to add one and they haven't asked yet. Mirrors `execute_zero_ft_sensor` in `record_robot_waypoints.py` (uses the `ZeroFTSensor` primitive with `dataCollectTime=0.2`, `enableStaticCheck=0`, `calibExtraPayload=0`).
- **A consumption example** — Python snippet showing how to load `camera_tcp.yaml` and use it in their own pipeline. The math is already documented in `command_verify`; could be extracted as a `locate` subcommand that just prints predicted world coords of detected ArUco tags without commanding motion.
- **Multi-view consistency check as a subcommand** — capture from N poses, print `T_world_board` for each, report spread. Effectively `validate` with per-sample world-board pose dumps. Useful as a calibration-correctness check that requires no robot motion.

### Repository git state

- The branch is detached (`HEAD`), main is `main`. Latest commit is `57f6323 Release/v1.9`. None of the changes in this calibration workflow have been committed. If the user asks for a commit, ask first — there are untracked files (the venv, captured images, etc.) that should not be staged.

---
