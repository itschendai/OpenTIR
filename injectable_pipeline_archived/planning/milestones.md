# Milestones

Ordered build sequence. Each milestone is self-contained: implementation + tests + docs
update in a single change set. The acceptance criterion is the gate for moving on.

Milestones M1–M4 are bench work and CI-friendly. M5–M11 require lab time. M12 is the
wrap-up.

## M1 — `arduino_client.py` baseline

**Scope**

- Implement `ArduinoClient` with every method in `specifications.md` §2.1.
- ACK/DONE/ERR parser, command ID counter, timeout + STOP_ALL recovery, command
  formatting.
- Unit and mock-serial integration tests per `testing_plan.md`.

**Acceptance**

- All §arduino_client tests in `testing_plan.md` pass on CI.
- A standalone smoke script (`python -m project.arduino_client --port /dev/ttyACM0 home`)
  performs `HOME_ALL` against a real Arduino and prints the parsed `DONE` dict. Recorded
  in `hardware_log.md`.

## M2 — `flexiv_helpers.py`

**Scope**

- Implement `RobotSession`, `execute_primitive`, `wait_for_primitive`,
  `selected_arm_state`, `switch_mode`, `set_cartesian_impedance`, `read_external_wrench`.
- The existing `project/record_robot_waypoints.py` and
  `project/play_recorded_waypoints.py` are **not refactored** in v1: the Phase 3
  implementation prompt explicitly directs that those scripts remain untouched. The
  helpers are used only by the new `injectable_pipeline/` code. The patterns in the
  existing scripts are the reference, not the rewrite target.

**Acceptance**

- All §flexiv_helpers tests in `testing_plan.md` pass on CI.
- `set_cartesian_impedance` is exercised on real hardware during M5 / M6 (compliant
  pickup and vise insertion). M2 itself only needs to land the helper module with
  passing unit tests; the HIL compliance-yield check is rolled into M5 / M6.

## M3 — `train_pipeline_poses.py` and YAML schema

**Scope**

- Implement the trainer per `specifications.md` §4.
- Implement YAML schema validator (shared module used by both the trainer and the
  orchestrator).
- Resume / overwrite / start-at flags.

**Acceptance**

- All §train_pipeline_poses tests in `testing_plan.md` pass.
- HIL: a full lab session captures all required poses and paths for the v1 recipe.
- The resulting YAML loads cleanly via the validator.

## M4 — `pipeline_orchestrator.py` skeleton and dry-run

**Scope**

- PARAMS block, state machine, pose loader, CLI argument parser.
- Per-phase stubs that print their intent (no hardware calls yet, but the orchestrator
  walks the full state graph).
- `--dry-run`, `--once`, `--cycles N`, `--skip-arduino`, `--skip-flexiv` flags.

**Acceptance**

- Unit and integration tests per `testing_plan.md` pass on CI.
- `python pipeline_orchestrator.py --dry-run --cycles 2` prints two complete cycles
  with the expected phase sequence and primitive calls.
- No hardware connection is attempted in `--dry-run`.

## M4.5 — Pre-HIL correctness pass

Applies the findings from the M1–M4 review before any hardware-in-loop milestone runs.
Pure bug-fix milestone; no new feature surface. All existing tests must still pass.

**Scope**

- **H1 — Remove the wrong Cartesian Mode integer fallbacks.** In
  `flexiv_helpers.MODE_VALUES`, the entries `NRT_CARTESIAN_MOTION_FORCE=6` and
  `RT_CARTESIAN_MOTION_FORCE=5` do not match the documented v1.9 declaration order
  (which puts RT=6 and NRT=7). The integer fallback is only used when attribute
  lookup fails; on a healthy v1.9 build the named attributes exist and the fallback
  never fires, but a wrong fallback is worse than no fallback. Delete the two
  Cartesian entries from `MODE_VALUES`. Keep `NRT_PRIMITIVE_EXECUTION=8` and
  `NRT_JOINT_IMPEDANCE=4` (those match). If the attribute lookup ever fails for a
  Cartesian mode, `rdk_mode()` should raise — that's the desired loud failure.

- **H2 — Remove the dead `SetCartesianStiffness` fallback.** Per the v1.9 release
  notes, the method was renamed to `SetCartesianImpedance` and the old name no
  longer exists. In `RobotSession.set_cartesian_impedance`, keep only the
  `SetCartesianImpedance` branch; remove the second `getattr(robot,
  "SetCartesianStiffness")` probe. Also: `SetCartesianImpedance(K_x, Z_x=[0.7]*6)`
  takes an optional damping-ratio vector. Add an optional
  `damping_ratio: list[float] | None = None` argument to `set_cartesian_impedance`;
  pass it through to the SDK when supplied, fall back to the SDK default when None.

- **H3 — Log the detected RDK API surface at startup.** In `RobotSession.__enter__`,
  after the robot becomes operational, emit one info log line identifying whether
  the joint-group surface (v2.0: `groups()` + `PrimitiveArgs`) or the single-group
  surface (v1.9) is in use. Wording suggestion: `"RDK API surface: single-group
  (v1.9 path)"` or `"RDK API surface: joint-group (v2.0 path)"`. No behavior
  change; this is observability so M5 HIL runs confirm the assumed code path.

- **M1 — Unit-conversion helpers for the JPos/RobotStates and Coord gotchas.**
  Add three module-level helpers to `flexiv_helpers.py`:

  - `joints_to_jpos_deg(q_rad: Iterable[float]) -> list[float]` — converts radians
    to degrees, validates length 7, returns the list ready for
    `flexivrdk.JPos(...)`.
  - `quat_to_rpy_deg(qw: float, qx: float, qy: float, qz: float) -> list[float]` —
    converts a unit quaternion to roll/pitch/yaw in degrees. Lift the existing
    function from `project/play_recorded_waypoints.py` (do not rederive).
  - `tcp_pose_to_coord_args(tcp_pose: Iterable[float], ref_frame=("WORLD",
    "WORLD_ORIGIN"), ref_joints_deg=None, ref_external=None) -> tuple` — returns
    the 5 positional arguments ready for `flexivrdk.Coord(position, orientation,
    ref_frame, ref_q_m, ref_q_e)`. Position stays in meters; orientation converted
    to degrees via `quat_to_rpy_deg`.

  Tests must cover identity quaternion, 90° rotation about Z, and a TCP pose
  round-trip (build args, construct a mock Coord, read back).

- **M2 — `E_STATE_MACHINE` error code.** Add `E_STATE_MACHINE = "E_STATE_MACHINE"`
  to `ErrorCode` in `pipeline_orchestrator.py`. Use it in `Orchestrator.transition`
  instead of `E_POSE_SCHEMA` for illegal transitions. Update
  `specifications.md` §6 Error Codes table to include the new entry. Add one new
  test asserting that an illegal transition raises `OrchestratorError` with
  `code == ErrorCode.E_STATE_MACHINE`.

- **M3 — Broaden `Orchestrator.run()` so hardware exceptions still trigger Arduino
  `STOP_ALL`.** Today `run()` catches only `OrchestratorError` and
  `KeyboardInterrupt`. Anything else — `serial.SerialException`, an RDK runtime
  error, a bug in a phase handler — propagates up to the `with session:` block,
  which calls `robot.Stop()` but never calls `arduino.stop_all()`. The cutting
  machine keeps doing whatever it was doing. Two layers of defense:

  1. **Broad `except Exception` clause** in `run()`. Wrap the exception in
     `OrchestratorError(ErrorCode.E_UNEXPECTED, str(exc))`, log the full
     traceback via `traceback.format_exc()` so debugging information is
     preserved, call `_handle_fault(wrapped)`, return `1`.
  2. **`try/finally` around the cycle loop**, ending in a best-effort
     `_best_effort_stop_all()` helper that calls `arduino.stop_all()` inside a
     try/except so the `finally` never raises. This guarantees the cutting
     machine is stopped even on paths that escape every except clause (for
     example a bug inside `_handle_fault` itself, or a sudden `BaseException`
     such as `SystemExit`).

  Add `E_UNEXPECTED = "E_UNEXPECTED"` to `ErrorCode` and to
  `specifications.md` §6. Document the new behavior in the failure-handling
  table in `architecture.md` (one new row: "Unexpected hardware/SDK exception
  during cycle" → "broad-catch in `run()` plus `try/finally` guard").

  Cost: one redundant STOP_ALL call on every clean shutdown (idempotent on the
  firmware side, sub-second). About 15 lines of code plus tests.

**Specifications updates required**

- `specifications.md` §1.1 PARAMS — no PARAMS additions (damping ratio stays SDK-default).
- `specifications.md` §3 `RobotSession` — document the new `damping_ratio` parameter
  on `set_cartesian_impedance` and the three new module-level conversion helpers.
- `specifications.md` §6 — add `E_STATE_MACHINE` row and `E_UNEXPECTED` row.
- `architecture.md` — failure-handling table gets one new row for the broad-catch
  behavior; no other design changes (H3 log line is observability).

**Acceptance**

- All existing tests still pass: `pytest project/injectable_pipeline/tests/`.
- New tests pass for `joints_to_jpos_deg`, `quat_to_rpy_deg`,
  `tcp_pose_to_coord_args`, and the `E_STATE_MACHINE` illegal-transition case.
- New tests for M3 pass:
  - Inject an arbitrary `RuntimeError` into a phase handler (via mock). `run()`
    returns `1`; the mock Arduino has received `STOP_ALL`; `E_UNEXPECTED` appears
    in the captured log output along with a traceback.
  - Make `arduino.stop_all()` raise inside `_handle_fault`. `run()` still
    returns without propagating; the `finally`-clause STOP_ALL attempt is
    logged.
- `MODE_VALUES` no longer contains either Cartesian entry.
- `set_cartesian_impedance` no longer references `SetCartesianStiffness`.
- A grep for `E_POSE_SCHEMA` in `pipeline_orchestrator.py` returns only legitimate
  pose-schema errors, not transition errors.
- `RobotSession.__enter__` logs the detected RDK API surface exactly once.
- `Orchestrator.run()` has the broad `except Exception` clause plus the
  `try/finally` STOP_ALL guard.
- `specifications.md` is updated in the same change set.

**Out of scope**

- No new runtime dependencies.
- No Arduino firmware changes.
- No new features. This is bug-fix, observability, and safety-defense only.

## M5 — Phase 1: pickup with compliant grip

**Scope**

- Implement the pickup phase per `specifications.md` §1.4 Phase 1.
- Switch to Cartesian impedance with PARAMS stiffness (low KX/KY, high KZ).
- Slow Cartesian approach to `pickup_grasp`; gripper close around injectable.
- Read `robot.states().tcp_pose` after grip; use as lift origin.
- Switch to stiff mode; vertical Z lift from captured origin to `pickup_lifted`.
- Walk path (or direct) to `above_vise`.

**Acceptance**

- Phase-1 HIL test in `testing_plan.md` passes: arm ends at `above_vise` joint pose
  within tolerance with the injectable in the gripper.
- Misalignment test at 10 mm offset succeeds (retainer + compliance self-center).
- Cycle log contains the captured lift origin TCP for the run.

## M6 — Phase 2: load with passive impedance

**Scope**

- Switch to Cartesian impedance with PARAMS stiffness.
- Slow Z descent with force-threshold abort.
- Restore stiff mode; call Arduino `close_vise`.
- Open gripper; walk path to `safe_intermediate`.

**Acceptance**

- Phase-2 HIL test passes: vise reports `CLOSED`, `force_kg >= VISE_TARGET_FORCE_KG`,
  arm at `safe_intermediate`.
- Force-limit-trip test passes: a deliberate obstruction triggers
  `E_FORCE_LIMIT` and the orchestrator goes to `FAULT` without crashing.

## M7 — Phase 3: cut

**Scope**

- Send `cut_height(CUT_Z_MM, CUT_X_MM, CUT_DEG)`; block on `DONE`.
- Verify Arduino reports `x_mm=0`, `rot_deg=0`, `blade_on=false` after.
- Confirm the arm has not moved during this phase (assertion at the boundary).

**Acceptance**

- Phase-3 HIL test passes end-to-end with a real injectable in the vise.
- Operator confirms the cut quality matches the Recipe1 tuning for the chosen height.

## M8 — Phase 4: twist-off and dispose top

**Scope**

- Walk path from `safe_intermediate` to `above_vise`.
- Cartesian descent to grasp the top (offset configurable in PARAMS).
- Wrist twist + vertical lift.
- Walk path to disposal pose; open gripper.

**Acceptance**

- Phase-4 HIL test passes: top is removed; uncut bridge snaps clean; gripper deposits
  the top at the disposal pose.
- Operator confirms top is fully separated (no plastic fragments hanging from the body).

## M9 — Phase 5: spring removal

**Scope**

- Return to `above_vise`.
- Descent to spring grasp pose (PARAMS-tuned).
- Lift, walk to disposal pose, open gripper.

**Acceptance**

- Phase-5 HIL test passes: spring removed from body bore and deposited at the disposal
  pose.

## M10 — Phase 6: body removal

**Scope**

- Implement Phase 6 per `specifications.md` §1.4 Phase 6 (grip-then-release).
- Walk path to `above_vise`; descent to body grasp pose; close gripper on body; call
  Arduino `open_vise()`; on `DONE`, lift body vertically; walk to disposal pose; open
  gripper.

**Acceptance**

- Phase-6 HIL test passes: body removed; vise reports `OPEN`; arm clears the machine.
- Grip-then-release ordering verified by inspection (no body drop during handoff).

## M11 — Loop, cycle counting, fault path

**Scope**

- `--cycles N` runs N complete cycles back-to-back.
- Per-cycle logging to `LOG_DIR/cycle_<timestamp>.log`.
- Fault path: any `FAULT` transition exits with code 1; orchestrator does not auto-retry.
- Resume / restart semantics: documented in `overview.md` (manual operator restart).

**Acceptance**

- E2E HIL test passes: 3 cycles back-to-back with no operator intervention; all logs
  present.
- Failure-injection tests per `testing_plan.md` pass (USB pull, E-stop, deliberate
  misalignment).

## M12 — Documentation and retrospective

**Scope**

- `project/planning/README.md` with an operator quick-start: how to train poses, how to
  run, what to do on a fault.
- Update PARAMS comments in `pipeline_orchestrator.py` with the tuned values that came
  out of the lab (forces, speeds, twist angle, impedance stiffness).
- One-page operator manual (`project/planning/operator_manual.md`).
- Retrospective: known gaps and the natural next steps (active force search, vision
  hand-off, multi-cut recipe iteration).

**Acceptance**

- A new operator, given only the planning docs and the captured pose YAML, runs one
  successful cycle without help.

## Out-of-band tasks (not gating any milestone)

- Removal or deprecation marker on the old `movej_joint_10deg.py` Arduino half (the
  9600-baud `ON/OFF` code). The robot half remains useful as a Flexiv example.
- Optional: a `project/planning/runbook.md` with common-error recipes once the pipeline
  has run enough to surface them.
