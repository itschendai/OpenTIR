# Specifications

This document defines the contract for every component named in `architecture.md`. Every
function, class, parameter, error code, and file schema lives here. Implementation must
match — drift requires an update to this document in the same change set.

## 1. `pipeline_orchestrator.py`

### 1.1 PARAMS block (canonical list)

A single Python dict named `PARAMS` at the top of the file, frozen at startup. Each entry
documented inline with name, units, default, rationale. Operator tunes here.

| Name | Type | Units | Default | Rationale |
| --- | --- | --- | --- | --- |
| `ROBOT_SN` | str | — | `"Rizon4-062930"` | Per `project/LLM.txt` |
| `ARDUINO_PORT` | str \| None | — | `None` (auto-detect) | Falls back to glob `/dev/ttyACM*` |
| `ARDUINO_BAUD` | int | baud | `115200` | Matches firmware `SERIAL_BAUD` |
| `ARDUINO_DONE_TIMEOUT_S` | float | seconds | `30.0` | Long enough for `HOME_ALL` and `CUT_HEIGHT` |
| `POSE_FILE` | str | — | `"pipeline_poses.yaml"` | Resolved relative to script dir |
| `CUT_Z_MM` | float | mm | `134.0` | Recipe1 second cut (frees plastic top) |
| `CUT_X_MM` | float | mm | `111.0` | Recipe1 cut depth |
| `CUT_DEG` | float | deg | `359.0` | Leaves 1° uncut bridge for twist-off |
| `VISE_TARGET_FORCE_KG` | float | kg | `4.0` | `PRIMITIVES.md` default |
| `VISE_RELEASE_FORCE_KG` | float | kg | `0.2` | `OPEN_VISE` completion threshold |
| `VISE_VERTICAL_APPROACH_MM` | float | mm | `100.0` | Hard rule: any motion entering or exiting the vise slot region (descent in Phase 2, descent + post-grasp lift in Phases 4–6) must be pure world-Z for at least this distance before any lateral component is permitted. Keeps the injectable axis aligned with the slot. |
| `PICKUP_Z_LIFT_MM` | float | mm | `50.0` | Per operator: vertical-only segment after grasp |
| `IMPEDANCE_KX_NM` | float | N/m | `50.0` | Low XY stiffness during compliant grip (Phase 1) and vise insertion (Phase 2) |
| `IMPEDANCE_KY_NM` | float | N/m | `50.0` | Low XY stiffness, same usage as `IMPEDANCE_KX_NM` |
| `IMPEDANCE_KZ_NM` | float | N/m | `3000.0` | High Z stiffness — Z is the driven axis in both compliant phases |
| `IMPEDANCE_KROT_NMRAD` | float | N·m/rad | `40.0` | Hold orientation through compliance |
| `PICKUP_APPROACH_SPEED_MM_S` | float | mm/s | `5.0` | Slow Cartesian approach to `pickup_grasp` while compliant |
| `INSERT_FORCE_THRESHOLD_N` | float | N | `15.0` | Abort threshold on wrist Fz during insert |
| `INSERT_DEPTH_MAX_MM` | float | mm | `40.0` | Safety budget for downward travel |
| `INSERT_SPEED_MM_S` | float | mm/s | `5.0` | Slow descent for compliant insertion |
| `TWIST_ANGLE_DEG` | float | deg | `90.0` | Wrist twist to snap uncut bridge |
| `TWIST_AXIS` | str | — | `"J7"` | Which arm joint performs the twist |
| `TWIST_LIFT_MM` | float | mm | `30.0` | Vertical lift during twist |
| `GRIPPER_OPEN_WIDTH_M` | float | m | `0.04` | Per existing scripts |
| `GRIPPER_CLOSE_WIDTH_M` | float | m | `0.0` | Closed |
| `GRIPPER_VELOCITY_M_S` | float | m/s | `0.05` | Slow for grip on small parts |
| `GRIPPER_FORCE_LIMIT_N` | float | N | `40.0` | Close force limit |
| `MOVE_JNT_VEL_SCALE` | int | % | `10` | MovePTP velocity scale |
| `MOVE_ZONE_RADIUS` | str | — | `"ZFine"` | MovePTP blending |
| `PHASE_PAUSE_S` | float | s | `0.5` | Pause between phases (settle) |
| `LOG_DIR` | str | — | `"logs"` | Per-cycle log files written here |

Adding a parameter requires updating this table.

### 1.2 CLI

```
python pipeline_orchestrator.py [options]

Options:
  --robot-sn STR           Override ROBOT_SN
  --arduino-port STR       Override ARDUINO_PORT (e.g., /dev/ttyACM0)
  --pose-file PATH         Override POSE_FILE
  --cycles N               Run N cycles then exit (default: infinite until Ctrl-C)
  --once                   Equivalent to --cycles 1
  --dry-run                Walk the state machine without connecting to hardware;
                           print every planned action and pose transition
  --skip-arduino           Skip serial connection; mock the Arduino client (for
                           Flexiv-only rehearsal)
  --skip-flexiv            Mock the robot; only exercise Arduino commands (for
                           machine-only rehearsal)
  --yes                    Skip startup confirmation prompt
  -h / --help              Show this help
```

The script exits 0 on clean completion (all requested cycles done), 1 on fault, 130 on
keyboard interrupt.

### 1.3 State machine

```python
class Phase(Enum):
    STARTUP = "STARTUP"
    PICKUP = "PICKUP"
    LOAD = "LOAD"
    CUT = "CUT"
    REMOVE_TOP = "REMOVE_TOP"
    REMOVE_SPRING = "REMOVE_SPRING"
    REMOVE_BODY = "REMOVE_BODY"
    FAULT = "FAULT"
    SHUTDOWN = "SHUTDOWN"
```

Transitions are linear (`STARTUP` → ... → `REMOVE_BODY` → `PICKUP`) with `FAULT` reachable
from any phase. The orchestrator records each transition in the per-cycle log with a
timestamp.

### 1.4 Per-phase contract

For each phase, three sets of fields. "Pre" must hold when the phase begins; "Post"
must hold when the phase completes successfully; "Failure" enumerates the errors that
trigger a `FAULT` transition.

#### Phase 1 — `PICKUP`

The injectable sits upright in a passive retainer (not a bore). The compliant grip lets
the gripper self-center against the part, and the *post-compliance* TCP is used as the
lift origin so we lift from where the wrist actually ended up rather than from the
nominal pre-grasp pose.

- **Pre**: gripper open; arm at `home`; pose file contains `pickup_pre_grasp`,
  `pickup_grasp`, `pickup_lifted`, `above_vise`; path `pickup_lifted_to_above_vise`
  optional.
- **Actions**:
  1. `MovePTP` to `pickup_pre_grasp` in stiff mode.
  2. Switch to Cartesian impedance with PARAMS stiffness (low KX/KY, high KZ, moderate
     orientation stiffness).
  3. Slow Cartesian approach to `pickup_grasp` at `PICKUP_APPROACH_SPEED_MM_S`. Z is
     held; XY can drift under contact with the retainer or part.
  4. Close gripper to `GRIPPER_CLOSE_WIDTH_M`.
  5. Read `robot.states().tcp_pose` and store as the **lift origin**.
  6. Switch back to stiff mode.
  7. Vertical Z lift of `PICKUP_Z_LIFT_MM` from the captured lift origin, ending at
     `pickup_lifted` (XY = lift origin's XY, Z = lift origin's Z + lift).
  8. Walk path (or direct `MovePTP`) to `above_vise`.
- **Post**: arm at `above_vise` joint pose; gripper holds injectable; gripper width and
  force consistent with a held part (logged but not gating in v1); `lift_origin` recorded
  in the cycle log.
- **Failure**: pose missing, MovePTP fault, gripper does not close to expected width,
  Cartesian impedance mode switch fails.

#### Phase 2 — `LOAD`

**Vise-approach invariant (cross-cutting).** The injectable axis must stay
aligned with the vise slot during entry. Operationally: the **last
`VISE_VERTICAL_APPROACH_MM` of motion before slot contact must be pure world
+Z descent — no lateral component**. `above_vise` is captured at or above
slot-top + `VISE_VERTICAL_APPROACH_MM` so a straight -Z descent from it always
satisfies the rule. Symmetrically: the **first `VISE_VERTICAL_APPROACH_MM` of
motion when exiting the slot region (Phases 4–6 post-grasp lifts) must be pure
+Z** before any path waypoint introduces lateral travel.

- **Pre**: arm at `above_vise`; gripper holds injectable; Arduino reports
  `vise_state=OPEN` and `x_mm≈0`.
- **Actions**: switch arm to Cartesian impedance with PARAMS stiffness values;
  Cartesian Z descent at `INSERT_SPEED_MM_S` up to `INSERT_DEPTH_MAX_MM`, aborting if
  `ext_wrench_in_tcp.fz > INSERT_FORCE_THRESHOLD_N`; restore stiff mode; call Arduino
  `close_vise(VISE_TARGET_FORCE_KG)`; open gripper; walk path (or direct) to
  `safe_intermediate`. The descent and the subsequent retreat from the slot
  region must satisfy the **Vise-approach invariant** above.
- **Post**: Arduino reports `vise_state=CLOSED`, `force_kg ≥ VISE_TARGET_FORCE_KG`,
  `x_mm=0`; gripper open; arm at `safe_intermediate`.
- **Failure**: insertion aborts on force threshold; `CLOSE_VISE` returns `ERR`; gripper
  cannot open.

#### Phase 3 — `CUT`

- **Pre**: arm at `safe_intermediate`; Arduino reports `vise_state=CLOSED`, `homed=true`,
  `faulted=false`.
- **Actions**: send Arduino `cut_height(z_mm=CUT_Z_MM, x_mm=CUT_X_MM, deg=CUT_DEG)`;
  block on `DONE` with `ARDUINO_DONE_TIMEOUT_S` per-command timeout.
- **Post**: Arduino reports `x_mm=0`, `rot_deg=0`, `blade_on=false`, `homed=true`;
  arm has not moved.
- **Failure**: `ERR` from Arduino; timeout; Flexiv fault detected mid-cut.

#### Phase 4 — `REMOVE_TOP`

- **Pre**: arm at `safe_intermediate`; cut complete; vise still closed.
- **Actions**: walk path (or direct) to `above_vise`; Cartesian descent to grasp the top
  (fixed Z offset from `above_vise`, configurable in PARAMS); close gripper; rotate
  `TWIST_AXIS` by `TWIST_ANGLE_DEG` while lifting `TWIST_LIFT_MM`; walk path to disposal
  pose (`disposal_top` if defined else `disposal`); open gripper.
- **Post**: gripper open; arm has cleared the disposal pose; vise still closed.
- **Failure**: pose missing; uncut bridge does not snap (detected as gripper resistance
  during lift, future work — v1 logs and continues).

#### Phase 5 — `REMOVE_SPRING`

- **Pre**: top removed; vise still closed.
- **Actions**: walk path to `above_vise`; descent to spring grasp pose (configurable);
  close gripper; lift; walk to disposal pose (`disposal_spring` if defined else
  `disposal`); open gripper.
- **Post**: gripper open; vise still closed.
- **Failure**: same as Phase 4.

#### Phase 6 — `REMOVE_BODY`

- **Pre**: spring removed; vise still closed; body in vise.
- **Actions**:
  1. Walk path to `above_vise`.
  2. Descent to body grasp pose.
  3. Close gripper on body to `GRIPPER_CLOSE_WIDTH_M`.
  4. Call Arduino `open_vise()`; block on `DONE`.
  5. Lift body vertically.
  6. Walk to disposal pose (`disposal_body` if defined else `disposal`); open gripper.
- **Post**: gripper open; vise open; Arduino reports `vise_state=OPEN`; arm cleared.
- **Failure**: `OPEN_VISE` returns `ERR`; gripper drops body (detectable as gripper width
  jumping open while motion continues — logged, not gating in v1).

- **Deferred alternative (firmware change required)**: A "release-then-grip" variant,
  potentially combined with opening the vise *past* the force-zero threshold by a
  small extra distance for maximum body clearance, has been considered. It would
  require either a new firmware primitive (`MOVE_VISE_REL`) or a new argument on
  `OPEN_VISE` (`release_extra_mm`). Out of scope for v1; revisit only if the default
  `grip_then_release` flow proves unreliable on real injectables.

## 2. `arduino_client.py`

### 2.1 `class ArduinoClient`

```python
class ArduinoClient:
    def __init__(self, port: str | None = None, baud: int = 115200,
                 done_timeout_s: float = 30.0, logger=None) -> None: ...
    def connect(self) -> None: ...
    def close(self) -> None: ...

    # mutating primitives — block on DONE
    def home_all(self) -> dict: ...
    def move_x_abs(self, x_mm: float, feed_mm_s: float | None = None) -> dict: ...
    def move_z_abs(self, z_mm: float, feed_mm_s: float | None = None) -> dict: ...
    def rotate_abs(self, deg: float, speed: float | None = None) -> dict: ...
    def move_rel(self, axis: str, delta: float, feed: float | None = None) -> dict: ...
    def set_blade(self, on: bool) -> dict: ...
    def close_vise(self, target_force_kg: float = 4.0) -> dict: ...
    def open_vise(self, target_force_kg: float = 0.2) -> dict: ...
    def cut_height(self, z_mm: float, x_mm: float, deg: float) -> dict: ...
    def stop_all(self) -> dict: ...
    def clear_faults(self) -> dict: ...

    # queries — single response
    def get_status(self) -> dict: ...
    def get_force(self) -> dict: ...
```

Return value is a dict with all `key=value` fields from the matching `DONE` line, plus
the synthetic key `command` (echoing the verb) and `cmd_id` (int).

### 2.2 Wire format

Per `XZ Stage Code v2/PRIMITIVES.md`. The client guarantees:

- Every mutating call emits one line `<CMD> cmd_id=<N> [args]\n` and reads lines until it
  sees `DONE <N> <CMD> ...` or `ERR <N> <CMD> ...`. Intermediate `ACK <N> <CMD>` is
  consumed and ignored. Any unrelated unsolicited lines are buffered for diagnostics.
- `cmd_id` is monotonically increasing per `ArduinoClient` instance, starting at 1.
- Argument values are formatted as `<key>=<value>` with floats rendered to six decimal
  places and booleans rendered as `ON`/`OFF` where the primitive expects them.
- `done_timeout_s` applies per command. On timeout, the client sends `STOP_ALL` and
  raises `ArduinoTimeoutError`.

### 2.3 Exceptions

```python
class ArduinoError(Exception): pass
class ArduinoTimeoutError(ArduinoError): pass
class ArduinoCommandError(ArduinoError):
    def __init__(self, code: str, message: str, cmd_id: int, command: str): ...
```

`ArduinoCommandError.code` is one of the codes in `PRIMITIVES.md` (`NOT_HOMED`, `BUSY`,
`FAULTED`, `INVALID_ARG`, `LIMIT_HIT`, `SENSOR_MISSING`, `TIMEOUT`, `INVALID_STATE`).

## 3. `flexiv_helpers.py`

### 3.1 `class RobotSession`

```python
class RobotSession:
    def __init__(self, robot_sn: str, logger=None) -> None: ...
    def __enter__(self) -> "RobotSession": ...
    def __exit__(self, exc_type, exc, tb) -> None: ...

    @property
    def robot(self) -> "flexivrdk.Robot": ...
    @property
    def gripper(self) -> "flexivrdk.Gripper | None": ...
    def setup_gripper(self, gripper_name: str, init: bool = True) -> "flexivrdk.Gripper": ...
    def selected_arm_state(self) -> tuple[str | None, object]: ...
    def execute_primitive(self, name: str, params: dict) -> None: ...
    def wait_for_primitive(self, state_key: str = "reachedTarget",
                           dt: float = 0.2, timeout_s: float | None = None) -> None: ...
    def switch_mode(self, mode_name: str) -> None: ...
    def set_cartesian_impedance(self, kx: float, ky: float, kz: float,
                                k_rot: float,
                                damping_ratio: list[float] | None = None) -> None: ...
    def read_external_wrench(self) -> dict: ...
```

Behaviors:

- `__enter__`: instantiate `flexivrdk.Robot`, clear fault if present, `Enable()`,
  wait `operational()`. After `operational()` returns true, emits one info log line
  identifying the detected RDK API surface — `"RDK API surface: joint-group (v2.0
  path)"` when both `robot.groups()` and `flexivrdk.PrimitiveArgs` exist, otherwise
  `"RDK API surface: single-group (v1.9 path)"`. Raises on failure.
- `__exit__`: `robot.Stop()` and best-effort cleanup. Never raises during exit.
- `selected_arm_state`: returns `(group_label, state)` where `group_label` contains
  `"ARM"` if present, else first group. Mirrors the existing pattern in
  `record_robot_waypoints.py`.
- `set_cartesian_impedance`: calls `robot.SetCartesianImpedance` with the per-axis
  stiffness vector `[kx, ky, kz, k_rot, k_rot, k_rot]`. The optional
  `damping_ratio` argument is the SDK `Z_x` per-axis damping ratio vector
  (length 6); when `None` the SDK default is used. Components of either vector
  must be non-negative.

### 3.2 Module-level unit-conversion helpers

```python
def joints_to_jpos_deg(q_rad: Iterable[float]) -> list[float]: ...
def quat_to_rpy_deg(qw: float, qx: float, qy: float, qz: float) -> list[float]: ...
def tcp_pose_to_coord_args(
    tcp_pose: Iterable[float],
    ref_frame: tuple[str, str] = ("WORLD", "WORLD_ORIGIN"),
    ref_joints_deg: Iterable[float] | None = None,
    ref_external: Iterable[float] | None = None,
) -> tuple: ...
```

- `joints_to_jpos_deg(q_rad)` — converts a 7-element radians vector to degrees
  ready to pass to `flexivrdk.JPos(...)`. Raises `ValueError` if the input is not
  length 7. Addresses the `RobotStates.q`-in-radians vs `JPos`-in-degrees gotcha
  called out in `project/LLM.txt`.
- `quat_to_rpy_deg(qw, qx, qy, qz)` — unit quaternion → roll/pitch/yaw in degrees.
  Lifted verbatim from `project/play_recorded_waypoints.py` so the pipeline and
  the existing playback script agree on the conversion bit-for-bit.
- `tcp_pose_to_coord_args(tcp_pose, ref_frame, ref_joints_deg, ref_external)` —
  given the 7-vector `[x, y, z, qw, qx, qy, qz]` in meters/quaternion, returns
  the 5 positional arguments `(position, orientation_deg, ref_frame, ref_q_m,
  ref_q_e)` ready to splat into `flexivrdk.Coord(...)`. Position stays in
  meters; orientation is converted to degrees via `quat_to_rpy_deg`.

## 4. `train_pipeline_poses.py`

### 4.1 Behavior

1. Connects to the robot using `RobotSession`.
2. Sets up gripper if requested (`--init-gripper`).
3. Switches to `FloatingJoint` zero-gravity mode.
4. Walks the operator through the **pose checklist** below, in order.
5. For each entry, prompts the operator (e.g., `Jog the arm to [pickup_grasp]. Press
   Enter to capture, 'r' to redo, 's' to skip).
6. On capture, reads `robot.states().q` and `robot.states().tcp_pose` and records both
   along with the gripper state.
7. For each **path entry**, the trainer enters a sub-loop: operator presses Enter for
   each intermediate waypoint, types `done` when finished. Order is preserved.
8. After the checklist, writes the YAML file atomically (temp file + rename).
9. CLI flags allow resuming a partially captured file (`--resume`), overwriting
   (`--overwrite`), and starting at a specific entry (`--start-at NAME`).

### 4.2 Pose checklist

Single poses (8):

- `home` — safe arm pose at startup and between cycles
- `pickup_pre_grasp` — entry to the vertical descent
- `pickup_grasp` — gripper closes here
- `pickup_lifted` — `PICKUP_Z_LIFT_MM` above `pickup_grasp`, end of vertical lift
- `above_vise` — start of impedance descent and the return target after the cut
- `safe_intermediate` — clear of the cutting envelope, robot waits here during `CUT_HEIGHT`
- `disposal` — single shared drop pose (also used as fallback when
  `disposal_top/spring/body` are missing)
- (optional) `disposal_top`, `disposal_spring`, `disposal_body` — per-component drop poses

Paths (5 minimum, more if obstacles demand it):

- `lifted_to_above_vise` — from `pickup_lifted` to `above_vise`
- `above_vise_to_safe_intermediate` — from `above_vise` to `safe_intermediate`
- `safe_intermediate_to_above_vise` — return after the cut
- `above_vise_to_disposal` — used for top, spring, and body unless per-component
  disposal poses are defined
- `disposal_to_home` — back to start of next cycle

Optional paths captured only if the operator opts into per-component disposal:

- `above_vise_to_disposal_top`
- `above_vise_to_disposal_spring`
- `above_vise_to_disposal_body`

## 5. `pipeline_poses.yaml` schema

```yaml
schema_version: 1
trainer_version: "<git short SHA or timestamp>"
robot_sn: "Rizon4-062930"
captured_at: "2026-05-19T11:30:00-07:00"

# Required: every pose listed in the checklist (minus the optional per-component ones).
poses:
  home:
    q_deg: [..., 7 values]
    q_rad: [..., 7 values]
    tcp_pose_world:
      order: ["x", "y", "z", "qw", "qx", "qy", "qz"]
      values: [..., 7 values]
    gripper_state:
      width_m: 0.04
      force_n: 0.5
  pickup_pre_grasp: { ... }
  pickup_grasp: { ... }
  pickup_lifted: { ... }
  above_vise: { ... }
  safe_intermediate: { ... }
  disposal: { ... }
  # optional:
  # disposal_top: { ... }
  # disposal_spring: { ... }
  # disposal_body: { ... }

# Required: every path the orchestrator references. Empty list allowed (means "no
# intermediate waypoints, use direct MovePTP").
paths:
  lifted_to_above_vise:
    from: pickup_lifted
    to: above_vise
    waypoints:
      - name: wp1
        q_deg: [..., 7 values]
        tcp_pose_world: { order: [...], values: [...] }
        gripper_state: { width_m: ..., force_n: ... }
      - name: wp2
        ...
  above_vise_to_safe_intermediate: { ... }
  safe_intermediate_to_above_vise: { ... }
  above_vise_to_disposal: { ... }
  disposal_to_home: { ... }
```

Validation rules at load time:

- `schema_version == 1`.
- Every required pose is present and has 7 joint values and 7 TCP values.
- Every required path is present (even if `waypoints: []`).
- Every `path.from` and `path.to` references a pose that exists in `poses`.
- If any of `disposal_top`, `disposal_spring`, `disposal_body` is set, the matching
  `above_vise_to_disposal_<x>` path may also be set (else fallback to
  `above_vise_to_disposal`).

## 6. Orchestrator error codes

| Code | Meaning |
| --- | --- |
| `E_FLEXIV_FAULT` | Robot reported fault that did not clear |
| `E_FLEXIV_NOT_OPERATIONAL` | `operational()` did not become true within startup window |
| `E_ARDUINO_TIMEOUT` | No `DONE` received within `ARDUINO_DONE_TIMEOUT_S` |
| `E_ARDUINO_ERR` | Wraps any Arduino `ERR` response |
| `E_FORCE_LIMIT` | `INSERT_FORCE_THRESHOLD_N` exceeded during compliance descent |
| `E_POSE_MISSING` | Required pose or path absent from YAML |
| `E_POSE_SCHEMA` | YAML failed schema validation |
| `E_GRIPPER_INIT` | Gripper init failed or width/force out of expected range |
| `E_INTERRUPTED` | Ctrl-C / SIGINT |
| `E_STATE_MACHINE` | Illegal phase transition requested by the orchestrator |
| `E_UNEXPECTED` | Unhandled hardware/SDK exception caught by the broad `except` in `Orchestrator.run()`; the original traceback is logged |

## 7. Dependencies and versions

- `flexivrdk` — same version as the existing scripts use; pinned via `pip` at install
  time. Joint-group API per `LLM.txt`.
- `pyserial` — already implied by `movej_joint_10deg.py`; pin `>=3.5`.
- `pyyaml` — new dependency for the YAML pose file. Pin `>=6.0`.
- `spdlog` — keep, matches existing scripts.
- `pytest` — `>=7.0` for the test suite.

No new dependencies beyond `pyyaml` and `pytest`.
