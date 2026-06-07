# In-Lab Implementation Prompt — M3 HIL through M11

Paste the contents below into a fresh Claude Code session **running on the lab
machine** (the one wired to the Flexiv Rizon4 and the cutting-machine Arduino).
Treat everything between the `---` markers as the prompt. Substitute the
operator's absolute workspace path where indicated.

This is a multi-milestone session. The agent should not try to finish all of
M5–M11 in one sitting; pause and check in with the operator at every milestone
boundary.

---

You are continuing implementation of an injectable-processing-pipeline project at
Stanford (CS225A / ME310). Milestones M1, M2, M3 (code only), M4, and M4.5 are
complete; **107 unit + mocked-integration tests pass**. You are now sitting at the
lab machine with real hardware connected. Everything from here onward is
hardware-in-loop (HIL). The operator is present at the bench with an E-stop.

## Working directory

`<WORKSPACE_ROOT>/flexiv_rdk_existing/project/injectable_pipeline/`

(Substitute the operator's absolute path. All paths in this prompt are relative
to this directory unless noted.)

## Authoritative references — read before any other action

1. `planning/milestones.md` — the canonical ordered plan. You are starting at the
   **M3 HIL** acceptance (lab pose capture) and proceeding through M5–M11.
2. `planning/specifications.md` — §1 orchestrator + PARAMS, §1.4 per-phase
   contracts, §3 RobotSession + helpers, §5 YAML schema, §6 error codes.
3. `planning/architecture.md` — failure-handling table, design decisions, directory
   layout.
4. `planning/testing_plan.md` — required HIL tests per component.
5. `planning/phase3_implementation_prompt.md` — the original Phase 3 brief; read
   for context, but this in-lab prompt supersedes it for the HIL portion.

Also consult as needed:

- `../XZ Stage Code v2/PRIMITIVES.md` — Arduino contract.
- `../LLM.txt` — Flexiv RDK conventions (joint-group API, unit gotchas).
- Sibling scripts `../record_robot_waypoints.py`,
  `../play_recorded_waypoints.py`, `../../example_py/basics1_display_robot_states.py`
  — usage patterns. Note that `../movej_joint_10deg.py`'s Arduino half is
  deprecated (9600-baud / "ON-OFF" code from before the firmware refactor).

## Hardware preconditions — operator confirms BEFORE you write or run anything

Ask the operator to confirm each line out loud. Do not proceed past this section
until every item is `[x]`.

- [ ] Flexiv Rizon4 powered up. Serial number `Rizon4-062930`.
- [ ] Pendant is in **Auto / Remote** mode (not Manual / Teach). The RDK cannot
      reach `operational()` otherwise.
- [ ] Arduino Mega 2560 connected via USB. A `/dev/ttyACM*` or
      `/dev/serial/by-id/*Arduino*` device exists.
- [ ] Cutting machine power on. Blade relay safe (off and disarmed if testable).
- [ ] Flexiv GN01 gripper physically attached and visible in Flexiv Elements.
- [ ] Pickup slot loaded with an injectable in the standard orientation.
- [ ] Vise is OPEN and at X-axis home (loading position).
- [ ] Workspace clear of human limbs, cables, fixtures. No items inside the arm's
      reach envelope except the pickup slot, vise, and disposal bin.
- [ ] Operator has the E-stop in hand.
- [ ] Python venv active. `pytest tests/ -q` reports green from your terminal.

## Phase 0 — Pre-HIL smoke tests (mandatory; do these first)

Three short checks, in order. Report each result back to the operator before
moving on. If any check fails, **stop** and ask for guidance — do not improvise a
workaround.

### Step A — Mode enum probe

Run from the lab terminal:

```bash
python -c "import flexivrdk; print(sorted([(m, int(getattr(flexivrdk.Mode, m))) for m in dir(flexivrdk.Mode) if not m.startswith('_')]))"
```

Expected (documented declaration order in v1.9):

```
[('IDLE', 0), ('NRT_CARTESIAN_MOTION_FORCE', 7), ('NRT_JOINT_IMPEDANCE', 4),
 ('NRT_JOINT_POSITION', 5), ('NRT_PLAN_EXECUTION', 9),
 ('NRT_PRIMITIVE_EXECUTION', 8), ('RT_CARTESIAN_MOTION_FORCE', 6),
 ('RT_JOINT_IMPEDANCE', 2), ('RT_JOINT_POSITION', 3), ('RT_JOINT_TORQUE', 1)]
```

If `NRT_CARTESIAN_MOTION_FORCE` resolves to anything other than **7** or
`RT_CARTESIAN_MOTION_FORCE` to anything other than **6**, **stop**. The named
attribute path is what matters; what we want to confirm is that the attribute
lookup works at all. Paste the observed list into chat and ask the operator how
to proceed.

### Step B — Arduino smoke

```bash
python -m arduino_client status
```

This sends `GET_STATUS` and prints the parsed `DONE` dict. Expected: a dict with
`homed=False, busy=False, faulted=False, vise_state=OPEN` (the firmware boots
unhomed). If you see `ArduinoTimeoutError` or no port found, the Arduino is not
reachable — stop and ask the operator to verify USB and firmware.

Optional follow-up after the operator confirms it is safe to home:

```bash
python -m arduino_client home
```

Watch the machine. Confirm `DONE` returns and the X/Z/rotary axes are at home.

### Step C — Robot connectivity

```bash
python ../../example_py/basics1_display_robot_states.py Rizon4-062930
```

This is the vendor script. Confirm joint positions print at 1 Hz. Ctrl-C to exit
after one second. If `Enable()` hangs or `operational()` never goes true, the
pendant is probably not in Auto/Remote mode. Stop and ask.

After all three steps pass, record:

```
## YYYY-MM-DD Phase 0 smoke — <operator initials> — PASS
- Mode probe: <paste line>
- Arduino status: <paste relevant fields>
- Robot operational: yes
```

…to `tests/hardware_log.md` (create the file if it does not exist).

## Phase 1 — M3 HIL acceptance: capture poses (operator-driven)

The trainer is **operator-driven**. You do not press the capture key. Your job:

1. Tell the operator how to start the trainer:
   ```bash
   python train_pipeline_poses.py
   ```
   Or with `--init-gripper` if the gripper needs initialization first.

2. Stay in the chat. As the trainer walks the operator through the checklist
   (`home`, `pickup_pre_grasp`, `pickup_grasp`, `pickup_lifted`, `above_vise`,
   `safe_intermediate`, `disposal`, then the five required paths), watch the
   operator's reported output. If they see a Python traceback or an "arm state
   has N joint values" error, stop and diagnose.

3. After the trainer reports `All entries captured. YAML written to
   pipeline_poses.yaml`, verify the file from your own terminal:
   ```bash
   python -c "import pose_schema; doc = pose_schema.read_yaml('pipeline_poses.yaml'); pose_schema.validate(doc); print('valid; poses=', list(doc['poses']), 'paths=', list(doc['paths']))"
   ```
   This must print "valid; poses=[...]" with all required entries.

4. Re-run `pytest tests/ -q` to confirm nothing regressed.

5. Record in `tests/hardware_log.md`:
   ```
   ## YYYY-MM-DD M3 HIL — <operator initials> — PASS
   - Captured 7 required poses + 5 required paths.
   - YAML at injectable_pipeline/pipeline_poses.yaml validates.
   ```

After M3 HIL is logged, **stop and check in** with the operator before starting
M5.

## Phase 2 — M5 through M11 implementation, milestone by milestone

For each milestone in order (M5, M6, M7, M8, M9, M10, M11), follow this exact
loop. **Do not start the next milestone until the previous one has a PASS line
in `tests/hardware_log.md` and the operator says go.**

### Per-milestone loop

1. **Read** the milestone's *Scope* and *Acceptance* sections in
   `planning/milestones.md`. Re-read the matching `planning/specifications.md`
   section for the exact contract.
2. **Implement** the code for that milestone in the existing source files
   (`pipeline_orchestrator.py` for phase bodies, `flexiv_helpers.py` if a new
   helper is needed). Prefer editing existing functions over adding new ones.
3. **Write tests** alongside the code. New unit/mocked-integration tests go in
   `tests/`. Existing M1–M4.5 tests must still pass.
4. **Run** `pytest tests/ -q`. All green before proceeding.
5. **Dry-run** the orchestrator end-to-end:
   ```bash
   python pipeline_orchestrator.py --dry-run --once
   ```
   Confirm the dry-run prints the expected phase sequence and primitive calls
   for the milestone's phase.
6. **Tell the operator** what is about to happen on the hardware: which phase
   runs, which primitives fire on the Arduino, what the arm will do, what the
   expected post-state is, and what to watch for.
7. **Operator confirms** before you run on hardware. Wait for explicit go-ahead.
8. **Run on hardware**, conservatively. For the first run of any new milestone,
   use `PARAMS["MOVE_JNT_VEL_SCALE"] = 5` (half the default) and keep the
   `--once` flag. The operator may ask you to slow it further.
9. **Observe** the run. If any of the *Acceptance* criteria are not met, stop.
   Diagnose. Do not retry blindly. If the cause requires a spec change,
   update `planning/specifications.md` in the same change set, **then** ask the
   operator for go-ahead to re-run.
10. **Log** the outcome to `tests/hardware_log.md`:
    ```
    ## YYYY-MM-DD M<n> — <operator initials> — PASS/FAIL — <one-line notes>
    - tuned params: <name>=<value>, <name>=<value>
    - observed: <one-line description>
    ```
11. **Pause** and ask the operator whether to continue to the next milestone. Do
    not assume the answer.

### Milestone-specific safety guards

These are non-negotiable, on top of the per-milestone loop above.

- **M5 (Phase 1 pickup)**. Start with `IMPEDANCE_KX_NM = IMPEDANCE_KY_NM = 50`
  and `PICKUP_APPROACH_SPEED_MM_S = 5`. If you see the wrist deflect more than
  ~10 mm in XY during the compliant approach, stop — that's outside the slot
  retainer's design tolerance.
- **M6 (Phase 2 load)**. The vise `CLOSE_VISE` primitive has a 5-minute timeout
  on the firmware side. If your `arduino.close_vise()` call ever times out
  client-side before the firmware, the firmware will still be clamping. Send
  `STOP_ALL` and wait for the operator to inspect.
- **M7 (Phase 3 cut)**. **Never run `cut_height` without first confirming**:
  (1) the vise reports `CLOSED` and `force_kg ≥ VISE_TARGET_FORCE_KG`, and
  (2) the arm is at `safe_intermediate` (read joint pose, compare to the YAML
  entry with a tolerance, log the comparison before issuing the command). The
  blade turns on during this primitive. The operator's hand must not be inside
  the machine envelope.
- **M8 (Phase 4 twist + dispose top)**. The first twist+lift may snap or stretch
  the bridge unpredictably. Run with `TWIST_ANGLE_DEG = 45` (half the default)
  on the first attempt; raise only if 45° is insufficient.
- **M9 (Phase 5 spring)**. The spring may be loose. If the gripper closes on
  empty air (gripper width close to `GRIPPER_CLOSE_WIDTH_M` after close), log it
  but do not crash — fall through to Phase 6.
- **M10 (Phase 6 body)**. Grip-then-release ordering is non-negotiable: gripper
  must close on the body before `OPEN_VISE` is called. Wire this as a hard
  assertion in the code, not just a comment.
- **M11 (loop + cycle counting)**. The first multi-cycle run uses `--cycles 2`.
  Do not jump straight to 3+ until two cycles in a row complete clean.

## Stop conditions (any of these → halt immediately)

- The operator says stop, or moves toward the E-stop.
- Any unexpected Arduino `ERR` or Flexiv fault that does not clear on a single
  `ClearFault()` retry.
- A pytest regression. `tests/` was green at the start of the session and must
  remain green throughout.
- Any motion that does not match the planned trajectory the dry-run printed —
  speed, direction, target, or order of operations.
- A test that the milestone's acceptance gate requires but you cannot make pass.
  Do not weaken or delete the test.

## Out of scope

- Vision integration (`cameratest.py`, `segment_injectable.py`). v1 has no
  vision.
- Multi-cut recipes in a single cycle. The architecture leaves room, but only
  one `CUT_HEIGHT` per cycle is exercised in v1.
- Tuning the firmware. The Arduino firmware in `../XZ Stage Code v2/` is out
  of scope for this session unless a milestone explicitly requires a firmware
  change (none in M5–M11). If you find you need one, stop and ask.

## When to stop the session

Stop the session when **any** of the following is true:

- The operator says stop or needs to leave.
- A milestone has a logged PASS and the operator decides to wrap.
- A blocker requires offline investigation (e.g., a part redesign, a firmware
  change, a missing dependency).

Before ending the session, write a short summary message to the operator listing
which milestones closed today, which are still open, which tuned parameter
values were settled, and any open questions for next time. The summary should
mirror what is in `tests/hardware_log.md` so the next session's operator can
pick up cleanly.

## Start

Begin by:

1. Confirming the hardware preconditions checklist (every line) with the
   operator.
2. Running the Phase 0 smoke tests A, B, C in order.
3. Asking the operator: "All smoke tests pass. Do we start M3 HIL pose capture
   now, or continue to M5 directly?" (They will normally want M3 HIL first —
   M5 cannot run without `pipeline_poses.yaml`.)
4. Proceeding milestone by milestone after each operator approval.

End every milestone with a one-paragraph status update in chat plus a row in
`tests/hardware_log.md`. Do not skip ahead.

---
