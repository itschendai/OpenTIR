# Phase 3 Implementation Prompt

Paste the contents below into a fresh Claude Code instance to start implementation.
Treat everything between the `---` markers as the prompt.

---

You are taking over the implementation phase of a robotics project at Stanford
(CS225A / ME310). The project disassembles injectables using a Flexiv Rizon4 arm and an
Arduino-controlled cutting machine. All requirements and design work is complete; your
job is to write the code per the existing specifications.

## Working directory

`<WORKSPACE_ROOT>/flexiv_rdk_existing/project/injectable_pipeline/`

All new code, planning docs, and tests live in this single self-contained subdirectory.
The rest of this prompt uses `injectable_pipeline/` as shorthand.

The existing `flexiv_rdk_existing/project/` scripts (`record_robot_waypoints.py`,
`play_recorded_waypoints.py`, `movej_joint_10deg.py`, vision scripts) are siblings of
`injectable_pipeline/` and are not imported by it. Reference them for usage patterns;
do not modify them.

## Authoritative references — read in this order before writing any code

1. `injectable_pipeline/planning/overview.md` — goals, scope, success criteria,
   assumptions.
2. `injectable_pipeline/planning/architecture.md` — components, data flow, state
   machine, key design decisions, failure-handling matrix, directory layout.
3. `injectable_pipeline/planning/specifications.md` — interface contracts.
   **Section 1.1 (PARAMS table)**, **Section 1.4 (per-phase contracts)**,
   **Section 2 (ArduinoClient)**, **Section 4 (trainer)**, and **Section 5 (YAML
   schema)** are the contracts you must match exactly.
4. `injectable_pipeline/planning/testing_plan.md` — every test you must write for every
   component, broken out by unit / mocked-integration / hardware-in-loop (HIL).
5. `injectable_pipeline/planning/milestones.md` — ordered build sequence M1 through M12
   with per-milestone acceptance criteria. **Do not skip ahead.**

Also read before starting M1:

- `../../XZ Stage Code v2/PRIMITIVES.md` — the Arduino's ASCII command/response
  contract. Your `arduino_client.py` must speak this exactly.
- `../LLM.txt` — Flexiv RDK conventions for the joint-group API, primitive execution,
  Coord/JPos types, units, and pitfalls. Pay particular attention to the v2.0
  joint-group rule (`robot.ExecutePrimitive(...)` takes a dict keyed by joint group).
- The existing sibling scripts (`../record_robot_waypoints.py`,
  `../play_recorded_waypoints.py`, `../movej_joint_10deg.py`,
  `../../example_py/basics1_display_robot_states.py`) for usage patterns. Note that
  `movej_joint_10deg.py`'s Arduino half is deprecated; do not inherit its 9600-baud /
  `ON-OFF` style.

## Workflow rules (from the operator's CLAUDE.md)

- **Build one milestone at a time, in order.** No work on M5 until M4 closes, etc.
- **Per milestone**: implement → write tests → run tests → fix until passing → only
  then move on. Tests are not optional and not deferred.
- **If implementation reveals a flaw in the specification**, stop, edit the relevant
  planning doc, confirm the change with the operator, *then* continue. Do not let
  code and specifications drift apart.
- **If a test cannot be made to pass**, stop and ask. Do not delete or weaken the test.
- **Prefer editing existing files over creating new ones** unless `architecture.md`
  calls for a new module.
- **No new dependencies** beyond those listed in `specifications.md` §7
  (`pyyaml`, `pytest` are new; everything else is already used by existing scripts).
- **Do not modify the Arduino firmware** in `flexiv_rdk_existing/XZ Stage Code v2/`
  unless a milestone explicitly requires it. The deferred "release past force=0"
  feature in `specifications.md` §1.4 Phase 6 is *not* in scope.
- **When uncertain, ask**. Do not guess at requirements that weren't specified.

## Safety considerations

- The Arduino controls a sharp ultrasonic blade and a 4-motor stage. The Flexiv arm has
  long reach. The operator will be at the bench with an E-stop during HIL milestones.
- Default to **conservative speeds and forces** in initial HIL tests. The PARAMS table
  defaults are starting points; expect the operator to tune them down further before
  the first real run.
- Phase 3 (M7) executes `CUT_HEIGHT` with the blade on. Do not start this milestone
  until M6 (vise load) has been verified to actually clamp the part. A loose part under
  a rotating blade is dangerous.
- Never command the arm to enter the cutting envelope while the machine is in a state
  other than X=0 loading. The orchestrator's state machine encodes this as a precondition;
  preserve that.

## Bench-only vs. lab-only milestones

You can complete M1–M4 entirely from your terminal using mocks and unit tests.
**Stop after M4 and wait for the operator's explicit confirmation** that they are at the
lab with the robot powered, the Arduino connected, and ready to run HIL tests. M5
through M11 require the operator's presence. M12 is documentation cleanup that can be
done from anywhere.

## Hardware environment

- **Robot**: Flexiv Rizon4, serial `Rizon4-062930`. Pendant must be in Auto/Remote
  mode for the RDK to reach `operational()`.
- **Gripper**: Flexiv GN01 parallel-jaw, initialized once at startup, ~4 s init wait.
- **Arduino**: Mega 2560 (PlatformIO target `megaatmega2560`) running the firmware in
  `flexiv_rdk_existing/XZ Stage Code v2/`. USB serial at **115200 baud** speaking the
  line-based primitive API in `PRIMITIVES.md`. Auto-detect via
  `/dev/ttyACM*` / `/dev/serial/by-id/*Arduino*`.
- **OS**: Ubuntu 22.04+ on the lab machine; the operator may also run from a workspace
  folder mounted via WSL (do not depend on a specific mount path — read it from
  `--robot-sn` and `--arduino-port` flags or PARAMS).

## How each milestone closes

1. All unit tests it introduces pass on CI (`pytest project/tests/`).
2. All mocked-integration tests it introduces pass on CI.
3. For HIL milestones: the operator's pass/fail entry is recorded in
   `injectable_pipeline/tests/hardware_log.md` (create this file if missing; the format
   is `## YYYY-MM-DD M<n> — operator name — PASS/FAIL — notes`).
4. `specifications.md` is updated if the milestone exposed any contract change.
5. A short summary message to the operator in chat: what shipped, what tests ran, what
   to verify, and what the next milestone needs from them.

## Start

Begin by reading the five planning docs and `PRIMITIVES.md`, then send the operator a
short message confirming:

- The set of files you plan to create for M1.
- The exact PARAMS keys you will use and the test names you will write.
- Anything in the specs that was ambiguous on your read.

Wait for the operator's go-ahead before writing M1. After that, proceed milestone by
milestone with a brief check-in at the end of each.

---
