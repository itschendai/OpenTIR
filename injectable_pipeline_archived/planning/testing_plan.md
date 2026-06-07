# Testing Plan

## Framework

- **Unit and integration tests**: `pytest` in `project/tests/`.
- **Hardware-in-loop tests**: executed manually in the lab, with the operator following
  a checklist and recording pass/fail in `project/tests/hardware_log.md`. Each hardware
  test names its acceptance criteria so a different operator can repeat it.
- **End-to-end smoke**: full cycle on real hardware, gated by all preceding tests passing.

Coverage target: every component in `specifications.md` has at least one happy-path test
and one failure-path test. Every PARAMS value referenced by code is exercised at its
default at least once.

## Test categories

1. **Unit (CI-able)** — no hardware, no serial port, no robot. Pure Python.
2. **Integration with mocks** — mocked Arduino serial via a fake stream, mocked Flexiv
   RDK via `unittest.mock`. Still CI-able.
3. **Hardware-in-loop (HIL)** — real arm and/or real Arduino, real fixtures.
4. **End-to-end (E2E)** — full pipeline cycle on real hardware.

A milestone may close with categories 1–2 passing on CI plus at least one category-3 run
recorded in `hardware_log.md`. Category 4 gates only the final milestone.

## Component tests

### `arduino_client.py`

Unit:

- `parse_ack_line` returns correct `(cmd_id, command)` for well-formed `ACK ...` lines.
- `parse_done_line` returns the trailing `key=value` dict with correct typing for floats,
  ints, bools, and strings.
- `parse_err_line` returns `(cmd_id, command, code, message)`; `message` strips quotes.
- `format_command` produces the expected wire format for each primitive, with floats at
  six decimal places and booleans as `ON`/`OFF` where required.
- `cmd_id` increments monotonically per instance, starting at 1.

Integration (mock serial):

- `home_all` round-trip: client writes one line, fake serial responds with `ACK` then
  `DONE`, method returns the parsed dict.
- `cut_height` round-trip with all required args; verifies args are serialized.
- ERR-response path: fake serial responds with `ERR`; method raises
  `ArduinoCommandError` with correct `code` and `message`.
- Timeout path: fake serial responds with neither `DONE` nor `ERR`; method raises
  `ArduinoTimeoutError` after `done_timeout_s`; client emits `STOP_ALL` afterwards.
- Stray-line tolerance: fake serial emits an unsolicited line before `DONE`; the method
  ignores it and still returns.

HIL:

- Real Arduino, real serial: `home_all` completes; `get_status` returns a dict with the
  expected keys.
- `clear_faults` is idempotent when not faulted (no ERR).

### `flexiv_helpers.py`

Unit (mocked `flexivrdk`):

- `RobotSession.__enter__` calls `fault()`, `ClearFault()` if true, `Enable()`, and waits
  for `operational()`. Raises if `ClearFault` fails.
- `RobotSession.__exit__` calls `Stop()` and does not propagate exceptions raised inside
  the `with` block via the exit path itself.
- `execute_primitive` resolves joint groups when `groups()` is present and falls back to
  the legacy single-argument form otherwise.
- `wait_for_primitive` returns when the state key resolves to truthy for all groups;
  raises `TimeoutError` past `timeout_s`.
- `switch_mode` resolves named modes via attribute lookup, falling back to the integer
  map already present in `record_robot_waypoints.py`.
- `set_cartesian_impedance` calls the documented RDK setter with a length-6 stiffness
  vector and validates input.

HIL:

- `RobotSession` opens and closes cleanly on the real robot.
- `set_cartesian_impedance` followed by a slow Z move shows visible XY compliance under
  manual side-load (operator pushes the wrist; arm yields).

### `train_pipeline_poses.py`

Unit:

- Pose checklist has the expected names (matches `specifications.md` §4.2).
- YAML output passes schema validation (uses the same validator as the orchestrator).
- Resume mode reads an existing YAML and skips already-captured entries.
- `--overwrite` blanks the file.
- Aborting mid-session writes a partial YAML that is still schema-valid for the entries
  it contains.

Integration (mocked robot):

- Capture a single pose: the YAML entry has 7-value `q_deg`, 7-value `tcp_pose_world`,
  and a gripper block.
- Capture a path with three waypoints in order; YAML preserves order.

HIL:

- Full session: operator captures all 8 required poses and 5 required paths in one go.
  Resulting YAML loads without error in the orchestrator.

### `pipeline_orchestrator.py`

Unit:

- `PARAMS` validation: every key documented in `specifications.md` §1.1 is present in
  the dict; type matches.
- Pose loader: refuses to start when a required pose is missing, naming the missing
  entries.
- State machine: every legal transition is reachable; every illegal transition raises.
- `--dry-run` walks the full state machine without instantiating an `ArduinoClient` or
  `RobotSession`.

Integration (mocked both sides):

- Single dry-run cycle prints the expected sequence of phase transitions and the
  expected primitive calls in order.
- Arduino `ERR` mid-cycle transitions the orchestrator to `FAULT` and exits 1.
- Insertion force-limit trip transitions to `FAULT`; the orchestrator retracts Z and
  calls `STOP_ALL` on the Arduino before exiting.
- `--cycles 3` runs three full cycles in the mocks and exits 0.

HIL — single phase at a time (matches the milestone order in `milestones.md`):

- **Phase 1**: `--once` with phases 2–6 stubbed; arm ends at `above_vise` with the
  injectable in the gripper.
- **Phase 2**: starts from Phase 1's end state; ends with vise closed and arm at
  `safe_intermediate`.
- **Phase 3**: Arduino completes `CUT_HEIGHT`; arm has not moved; `x_mm=0` after.
- **Phase 4**: top is snapped, gripper drops at `disposal`.
- **Phase 5**: spring removed and disposed; vise still closed.
- **Phase 6**: body gripped, `OPEN_VISE` returns `DONE`, body lifted and disposed.

E2E:

- One full cycle, then three back-to-back cycles, with no operator intervention between
  cycles. All six phases pass for every cycle. Log files exist in `LOG_DIR`.

## Failure injection (HIL)

- Pull the Arduino USB during Phase 3 mid-cut: orchestrator raises `E_ARDUINO_TIMEOUT`,
  sends `STOP_ALL` (best-effort, the line may be dead), enters `FAULT`, and exits 1.
- Press the E-stop during Phase 5: Flexiv fault detected, orchestrator does not call
  further primitives, enters `FAULT`.
- Deliberately misalign `pickup_grasp` by 10 mm: passive impedance still grasps; verify
  by inspection.
- Deliberately misalign by 30 mm: gripper either misses or jams; orchestrator either
  completes Phase 1 with no part in the gripper (gripper width out of expected band) or
  trips the insertion force threshold in Phase 2. Either is acceptable; the test
  documents observed behavior.

## Test data

- A fixture YAML `tests/fixtures/pipeline_poses_valid.yaml` with synthetic but
  schema-valid pose data.
- A fixture YAML `tests/fixtures/pipeline_poses_missing.yaml` missing `safe_intermediate`
  to exercise the loader error path.
- A fake serial fixture that replays canned transcripts for the Arduino integration
  tests.

## Acceptance gate per milestone

A milestone closes when:

1. All unit tests it introduces pass.
2. All integration tests it introduces pass.
3. The HIL test(s) for that milestone are run at least once and recorded in
   `tests/hardware_log.md` with operator name, date, and pass/fail.
4. `specifications.md` is updated if the milestone exposed a spec change.
