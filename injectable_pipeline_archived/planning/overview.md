# Injectable Processing Pipeline — Overview

## Project goal

Build an end-to-end Python program that disassembles injectables using the Flexiv Rizon4
arm and the Arduino-controlled cutting machine, with no vision. The arm picks an injectable
from a fixed vertical slot, loads it into the vise, the machine cuts it once, then the arm
removes the plastic top, the spring, and the body in sequence and disposes of each. The
cycle then repeats.

## Scope (v1)

In scope:

- Pickup from a single known vertical slot fixture (hardcoded pose).
- Force-compliant load into the vise at the machine's loading position (X = X-axis home).
- A single `CUT_HEIGHT` per cycle, with the cut height as a tunable parameter (default
  `z_mm = 134`, the second cut from `Recipe1.md`, which frees the plastic top).
- Twist-off and disposal of the plastic top via wrist rotation + lift.
- Removal and disposal of the spring from the body bore.
- `OPEN_VISE`-after-grip ordering for body removal, followed by disposal.
- Indefinite loop, with `--cycles N` and `--once` CLI flags.
- A supporting pose-trainer script that walks the operator through capturing every
  required pose and every multi-waypoint path, using `FloatingJoint` to hand-guide the arm.

Out of scope (v1):

- Vision-based detection or per-part pose estimation.
- Multi-cut recipes in a single cycle (the architecture leaves room, but only one cut is
  exercised).
- The "cut, pull some parts, cut again, pull more" iterative pattern — this is a
  forward-looking requirement noted in `architecture.md` but not exercised here.
- Active force-search XY centering during pickup (passive Cartesian impedance instead).
- Per-injectable parameter overrides (one shared PARAMS block for the whole run).
- Safety-rated stop or interlocks beyond what Flexiv RDK and the operator's E-stop provide.
- Camera integration with `cameratest.py` or `segment_injectable.py`.

## Target users

- A lab operator running interactive sessions on the bench, jogging the arm in
  `FloatingJoint` mode and tuning parameters between cycles.
- A future automation hand-off where the same orchestrator runs unattended after the
  poses and parameters are frozen.

## Success criteria

1. **End-to-end completion.** Given a correctly trained `pipeline_poses.yaml` and a tuned
   PARAMS block, the orchestrator completes pickup → load → cut → top → spring → body →
   loop without operator intervention except for the initial start prompt.
2. **Trainer experience.** A new operator can train every required pose, including all
   obstacle-avoidance waypoints between fixed positions, in under fifteen minutes using
   the supporting pose-trainer script.
3. **Single tuning surface.** Every numeric value the operator might want to adjust —
   forces, speeds, heights, retry counts, timeouts — lives in one PARAMS block at the top
   of `pipeline_orchestrator.py`, with units and a one-line rationale per parameter.
4. **Tests pass.** Every test enumerated in `testing_plan.md` passes (CI for unit and
   mock-hardware integration tests; lab sign-off for hardware-in-loop tests).
5. **Reproducibility.** A second operator can rerun the pipeline using only the planning
   docs, the captured pose file, and the orchestrator's CLI help.

## Key assumptions

- **Robot.** Flexiv Rizon4 with serial `Rizon4-062930` (per `project/LLM.txt`).
  Pendant must be in Auto / Remote mode for the entire run; the RDK cannot reach
  `operational()` otherwise.
- **Gripper.** Flexiv GN01 parallel-jaw, initialized once at startup.
- **Arduino.** Mega 2560 running the firmware in `XZ Stage Code v2/`, exposing the
  twelve-primitive line-based API at 115200 baud per `XZ Stage Code v2/PRIMITIVES.md`.
- **Mechanical layout.** Loading position is X-axis home (`X = 0`); cutting position is
  at `X = x_mm` per the recipe (default 111). The same physical vise is used for both —
  only the X stage moves the workpiece.
- **Injectable orientation.** Held vertically in the vise with the rotary cut axis vertical.
- **Pickup slot geometry.** The injectable sits upright in a passive retainer that keeps
  it vertical — not in a deep bore. Compliant lateral centering of the gripper, not
  vertical insertion, is the right model for Phase 1.
- **Disposal.** A single shared disposal pose by default; the YAML schema and orchestrator
  support optional per-component disposal poses (`disposal_top`, `disposal_spring`,
  `disposal_body`) when the operator wants to split them.
- **No vision.** All pickup, vise, and disposal locations are captured by the trainer and
  saved to a YAML file the orchestrator reads at startup.

## Deferred to lab tuning (not blockers for spec)

These values cannot be set from a desk and are expected to land via iteration on the
hardware. They are PARAMS in the orchestrator, not architectural questions.

- Exact joint and TCP values for every named pose (set by the trainer).
- Twist angle and twist axis for the top-snap motion (default: wrist `J7`, ±90°).
- Cartesian XY stiffness for the impedance-mode descent (start very low, e.g., 50 N/m).
- `target_force_kg` for `CLOSE_VISE` (default 4.0 per `PRIMITIVES.md`).
- Vertical-descent depth from `pickup_pre_grasp` to `pickup_grasp` (default: trainer-set
  Z delta).
- Spring grasp depth and gripper width (passive; gripper-only).
- Insertion depth budget for the vise load.

## Definitions

- **Loading position** — Cutting machine state where `X = 0`. The arm interacts with the
  vise only when the machine is in this state.
- **Cutting position** — Any state where the X stage is advanced from home and the blade
  may be on. The arm must be at `safe_intermediate` for the entire duration.
- **Recipe** — A specific combination of `z_mm`, `x_mm`, `deg` passed to the Arduino's
  `CUT_HEIGHT` primitive. v1 uses one entry from `Recipe1.md`.
- **Path** — An ordered sequence of recorded waypoints between two named poses, used for
  obstacle avoidance during Cartesian / MovePTP motion.
