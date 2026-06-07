# M4.5 Pre-HIL Correctness Pass — Prompt

Paste the contents below into a fresh Claude Code session. Treat everything between
the `---` markers as the prompt. Substitute your absolute workspace path where
indicated.

---

You are picking up an injectable-processing-pipeline project at the start of
milestone **M4.5**. M1–M4 are complete and have been reviewed; M4.5 is a focused
correctness pass that fixes five issues surfaced by that review. M5 (the first
hardware-in-loop milestone) is **blocked** until M4.5 closes.

## Working directory

`<WORKSPACE_ROOT>/flexiv_rdk_existing/project/injectable_pipeline/`

(Substitute the operator's absolute path.)

## Authoritative references — read in this order before touching any code

1. `planning/milestones.md` — the **M4.5** section is the canonical scope for this
   milestone. Read it in full. Read M1–M4 only for context.
2. `planning/specifications.md` — §1.1 PARAMS, §3 `RobotSession`, §6 error codes.
   You will be updating §3 and §6.
3. `planning/architecture.md` — failure-handling table and design decisions. No
   design changes in this milestone, but read it so your edits stay consistent.

Also reference:

- `flexiv_helpers.py`, `pipeline_orchestrator.py`, `pose_schema.py`,
  `train_pipeline_poses.py`, `arduino_client.py` — the existing implementations.
- `../play_recorded_waypoints.py` — sibling script. **Lift** its `quat_to_rpy_deg`
  function for M1; do not rederive.
- `../LLM.txt` — Flexiv RDK conventions, especially the JPos-degrees /
  RobotStates-radians and Coord-orientation-degrees gotchas.

## Scope (from M4.5 in `planning/milestones.md`)

Six items, in any order:

- **H1** — Delete `NRT_CARTESIAN_MOTION_FORCE` and `RT_CARTESIAN_MOTION_FORCE` from
  `flexiv_helpers.MODE_VALUES`. Keep the other two entries. If `rdk_mode()` is ever
  called with a Cartesian-mode name on a build where the attribute is missing, it
  should raise — that's the desired loud failure.

- **H2** — Remove the dead `SetCartesianStiffness` fallback in
  `RobotSession.set_cartesian_impedance`. Add an optional
  `damping_ratio: list[float] | None = None` argument; pass it to the SDK when
  supplied, omit when None so the SDK default applies. Update
  `specifications.md` §3 accordingly.

- **H3** — In `RobotSession.__enter__`, after `operational()` returns true, emit one
  info log line identifying the detected RDK API surface: `joint-group (v2.0
  path)` if both `robot.groups()` and `flexivrdk.PrimitiveArgs` exist, else
  `single-group (v1.9 path)`. No behavior change.

- **M1** — Add three module-level helpers to `flexiv_helpers.py`:

  ```
  joints_to_jpos_deg(q_rad)     -> list[float], length 7, validates length
  quat_to_rpy_deg(qw, qx, qy, qz) -> list[float], length 3, degrees
  tcp_pose_to_coord_args(tcp_pose, ref_frame=("WORLD", "WORLD_ORIGIN"),
                         ref_joints_deg=None, ref_external=None) -> tuple
  ```

  Lift `quat_to_rpy_deg` verbatim from `../play_recorded_waypoints.py`. Update
  `specifications.md` §3 to document the three helpers. Add unit tests covering
  identity quaternion, 90° rotation about Z, and a full TCP round-trip.

- **M2** — Add `E_STATE_MACHINE` to the `ErrorCode` enum in
  `pipeline_orchestrator.py`. Use it in `Orchestrator.transition` instead of
  `E_POSE_SCHEMA` for illegal transitions. Add the row to `specifications.md` §6.
  Add a test asserting that an illegal transition raises `OrchestratorError` with
  the new code.

- **M3** — Broaden `Orchestrator.run()` so unknown exceptions still trigger
  Arduino `STOP_ALL`. Two layers:

  1. Add a third `except Exception` clause. Log the full traceback
     (`traceback.format_exc()`), wrap in
     `OrchestratorError(ErrorCode.E_UNEXPECTED, str(exc))`, call
     `_handle_fault(wrapped)`, return `1`.
  2. Wrap the entire run body in `try/finally` and call a new helper
     `_best_effort_stop_all()` in the `finally`. The helper calls
     `arduino.stop_all()` inside its own try/except so the `finally` cannot
     raise. This guarantees the cutting machine is stopped even on paths that
     escape every except clause (e.g., a bug inside `_handle_fault` itself).

  Add `E_UNEXPECTED = "E_UNEXPECTED"` to `ErrorCode`. Update
  `specifications.md` §6 with the new row. Update
  `architecture.md`'s failure-handling table with a row for "unexpected
  hardware/SDK exception during cycle → broad-catch + try/finally guard".

  Tests:
  - Inject an arbitrary `RuntimeError` into a phase handler (via mock).
    `run()` returns 1; the mock Arduino received `STOP_ALL`; `E_UNEXPECTED`
    appears in the captured log output along with a traceback.
  - Make `arduino.stop_all()` raise inside `_handle_fault`. `run()` still
    returns without propagating; the `finally`-clause STOP_ALL attempt is
    logged.

## Out of scope

- No new runtime dependencies.
- No Arduino firmware changes.
- No new features. This milestone is bug-fix, observability, and safety-defense only.

## Workflow rules (from the operator's CLAUDE.md)

- Write tests for each fix. All existing tests + new tests must pass before
  reporting completion.
- Update `specifications.md` in the same change set as the code change (§3 for the
  helpers and the damping-ratio arg, §6 for the new error code). Do not let docs
  drift from code.
- If a fix reveals a deeper issue not captured in M4.5, **stop and consult** the
  operator — do not expand the milestone unilaterally.
- Prefer editing existing files over creating new ones unless a new module is
  clearly warranted (none should be in this milestone).

## Acceptance gate

Before reporting M4.5 done, all of the following must hold:

1. `pytest project/injectable_pipeline/tests/` is green (existing + new tests).
2. `MODE_VALUES` no longer contains either Cartesian entry.
3. `set_cartesian_impedance` no longer references `SetCartesianStiffness`.
4. A `grep -n E_POSE_SCHEMA pipeline_orchestrator.py` shows the code only in
   pose-schema-related call sites, not in `transition()`.
5. `RobotSession.__enter__` logs the detected RDK API surface exactly once per
   session entry.
6. `Orchestrator.run()` has both a broad `except Exception` clause and a
   `try/finally` STOP_ALL guard. The M3 tests (RuntimeError injection +
   _handle_fault-raises scenario) pass.
7. `specifications.md` §3 documents the three new helpers and the `damping_ratio`
   argument; §6 documents both `E_STATE_MACHINE` and `E_UNEXPECTED`.
8. `architecture.md`'s failure-handling table has a new row for the unexpected
   hardware/SDK exception path.

## Start

Begin by:

1. Reading `planning/milestones.md` (M4.5 section) and the four files listed under
   *Authoritative references*.
2. Listing back to the operator the exact files you intend to edit and the exact
   names of the new tests you intend to add.
3. Waiting for the operator's go-ahead before writing any code.

End your final report with the command output from `pytest
project/injectable_pipeline/tests/ -q` so the operator can see the suite is green.

---
