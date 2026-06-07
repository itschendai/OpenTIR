# Injectable Processing Pipeline

End-to-end Python orchestrator for disassembling injectables using a Flexiv Rizon4 arm
and the Arduino-controlled cutting machine. Self-contained: every file the pipeline
needs (code, planning docs, tests, requirements) lives in this directory.

## Deploying to another machine

This entire `injectable_pipeline/` directory is drop-in portable to any clone of the
CS225A repository that has the same layout — i.e., any machine with
`flexiv_rdk_existing/project/`. Copy the directory in:

```
flexiv_rdk_existing/project/injectable_pipeline/
```

Then install dependencies:

```
cd flexiv_rdk_existing/project/injectable_pipeline
pip install -r requirements.txt
```

The only external file the pipeline references is the Arduino firmware contract at
`../../XZ Stage Code v2/PRIMITIVES.md` (documentation only).

## Quick start

1. Confirm hardware:
   - Flexiv Rizon4 (`Rizon4-062930`) with pendant in **Auto/Remote** mode.
   - Arduino Mega 2560 on USB (115200 baud) running the firmware in
     `../../XZ Stage Code v2/`.
   - Flexiv GN01 gripper attached and discoverable.
   - Operator E-stop within reach.

2. Capture poses (first time only, or after any fixture move):

   ```
   python train_pipeline_poses.py
   ```

   Follow the on-screen checklist. Outputs `pipeline_poses.yaml`.

3. Dry-run the orchestrator to verify the plan without touching hardware:

   ```
   python pipeline_orchestrator.py --dry-run --once
   ```

4. Run one cycle:

   ```
   python pipeline_orchestrator.py --once
   ```

5. Run N cycles:

   ```
   python pipeline_orchestrator.py --cycles N
   ```

See `python pipeline_orchestrator.py --help` for all CLI flags.

## Tuning

Every numeric value the operator might want to adjust is in the `PARAMS` block at the
top of `pipeline_orchestrator.py`. The canonical list with units and rationale lives in
`planning/specifications.md` §1.1.

If a parameter change works well in the lab, update both the default in
`pipeline_orchestrator.py` and the row in `planning/specifications.md` in the same
change set — they must not drift.

## What lives where

| File / dir | Purpose |
| --- | --- |
| `pipeline_orchestrator.py` | Main script. PARAMS block at the top. Runs the six-phase cycle. |
| `arduino_client.py` | Serial client for the Arduino's primitive API (line-based ASCII, 115200 baud). |
| `flexiv_helpers.py` | `RobotSession` context manager and Flexiv RDK utilities. |
| `train_pipeline_poses.py` | Interactive pose recorder using `FloatingJoint`. Walks a checklist; writes `pipeline_poses.yaml`. |
| `pipeline_poses.yaml` | Operator-captured pose data. Created by the trainer. |
| `requirements.txt` | Pip dependencies (`flexivrdk`, `spdlog`, `pyserial`, `pyyaml`, `pytest`). |
| `planning/` | Specification documents. Read these before changing anything substantive. |
| `tests/` | Pytest suite. Run with `pytest` from this directory. |

## Safety

- The cutting machine drives a sharp ultrasonic blade. Never start the orchestrator
  with the vise empty unless you're explicitly testing the machine alone.
- The orchestrator never auto-recovers from a fault. On fault, the script exits with
  code 1 and the operator reviews + restarts manually.
- The robot is constrained to stay at `safe_intermediate` whenever the cutting machine
  is in cutting state (X ≠ 0). Do not edit this constraint without updating
  `planning/specifications.md` §1.4 Phase 3.

## License / origin

Built on top of Flexiv RDK v1.9 (Apache 2.0). See `../../README.md` for the SDK
license; this directory's code is the student-written project layer.
