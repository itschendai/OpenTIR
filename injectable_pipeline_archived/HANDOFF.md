# Handoff to the Lab Machine

This document covers the one-time move of the `injectable_pipeline/` directory
from the development machine to the lab machine via USB. The lab machine has no
git access; everything travels in a single directory tree.

## TL;DR

1. On dev: build a clean tarball of `injectable_pipeline/` excluding the venv
   and bytecode caches.
2. Copy the tarball to USB.
3. On lab: extract into `flexiv_rdk_existing/project/`.
4. Create a fresh venv on the lab machine and `pip install -r requirements.txt`.
5. Run `pytest tests/ -q`. Must report `107 passed`.
6. Open Claude Code on the lab machine and paste the
   `planning/inlab_implementation_prompt.md` content.

## 1. What to copy (and what NOT to)

### Copy the entire `injectable_pipeline/` directory

Source on the dev machine:

```
<dev workspace>/flexiv_rdk_existing/project/injectable_pipeline/
```

Contents that must travel:

- All Python source files at the top level (`arduino_client.py`,
  `flexiv_helpers.py`, `pose_schema.py`, `pipeline_orchestrator.py`,
  `train_pipeline_poses.py`)
- `README.md`, `HANDOFF.md` (this file), `requirements.txt`
- `planning/` — every `.md` file inside, including
  `inlab_implementation_prompt.md`, `m4_5_fix_prompt.md`,
  `phase3_implementation_prompt.md`, the five spec docs
- `tests/` — every `.py` test, `conftest.py`, `__init__.py`, the entire
  `tests/fixtures/` directory
- Any `tests/hardware_log.md` if one already exists (it does not on dev)

### Exclude these from the copy

- `.venv/` — Python virtual environment. Hard-coded paths; not portable.
  Saves 50–200 MB.
- Any `__pycache__/` directory at any depth.
- Any `*.pyc` file.
- Any `pipeline_poses.yaml` at the root of `injectable_pipeline/`. The lab
  machine recreates this from its own trainer session — dev-side poses are
  meaningless on the lab fixture.

### Build the tarball (recommended one-liner)

Run from a **WSL or Linux bash shell** (not PowerShell), in
`flexiv_rdk_existing/project/`. GNU tar's `--exclude` matches the basename of
any path component, so simple names work recursively — no glob wildcards
needed.

```bash
cd <dev workspace>/flexiv_rdk_existing/project
ls injectable_pipeline/   # sanity check the source directory exists
tar --exclude='.venv' --exclude='__pycache__' --exclude='*.pyc' --exclude='pipeline_poses.yaml' -czf /tmp/injectable_pipeline.tar.gz injectable_pipeline/
```

Verify the tarball was actually created and is sensibly sized:

```bash
ls -lh /tmp/injectable_pipeline.tar.gz       # expect a few hundred KB to a few MB
tar -tzf /tmp/injectable_pipeline.tar.gz | head -30
tar -tzf /tmp/injectable_pipeline.tar.gz | grep -E '(\.venv|__pycache__|\.pyc)' || echo "OK — no venv/cache/bytecode in the archive"
```

If the tarball is over 50 MB, the venv probably leaked in. Re-check the
`--exclude` patterns. If the verify command lists any `.venv`, `__pycache__`,
or `.pyc` entries, the excludes did not match — confirm you ran the command
from `flexiv_rdk_existing/project/` (not from inside `injectable_pipeline/`
itself, and not from PowerShell).

Copy `/tmp/injectable_pipeline.tar.gz` to USB.

## 2. Where to place on the lab machine

The lab machine is assumed to already have the CS225A repository cloned with the
same layout the dev machine uses — i.e., the Flexiv RDK, the existing
`project/` scripts (`record_robot_waypoints.py`, `play_recorded_waypoints.py`,
`movej_joint_10deg.py`, the vision scripts), and the `XZ Stage Code v2/`
Arduino firmware tree are all already there.

Target path on the lab machine:

```
<lab CS225A root>/flexiv_rdk_existing/project/injectable_pipeline/
```

Extract the tarball into `project/`:

```bash
cd <lab CS225A root>/flexiv_rdk_existing/project/
tar -xzf /path/to/usb/injectable_pipeline.tar.gz
ls injectable_pipeline/                  # sanity check
```

**If the lab machine does *not* have CS225A at all**: ask the operator before
proceeding. Either clone the rest of CS225A from a USB-delivered copy, or
bring the dev machine to the lab. The pipeline references the Arduino
firmware contract (`../XZ Stage Code v2/PRIMITIVES.md`) and the vendor RDK
examples (`../../example_py/`) for context; missing those breaks the in-lab
smoke tests.

## 3. Prerequisites on the lab machine

The handoff assumes the lab machine already has:

- **Python 3.10 or later** (the dev environment is 3.12). Check:
  ```bash
  python3 --version
  ```
- **pip** working, with either internet access or a local wheel mirror. The
  five `requirements.txt` dependencies are not large.
- **Flexiv RDK 1.9 Python bindings** (`pip show flexivrdk` returns version
  1.9.x). The lab almost certainly has this from previous student work. If
  not:
  ```bash
  pip install flexivrdk==1.9.*
  ```
- **USB serial device permissions.** The Arduino shows up as `/dev/ttyACM0`
  (or similar). The user running the orchestrator must be in the `dialout`
  group on Linux:
  ```bash
  sudo usermod -aG dialout $USER
  newgrp dialout      # or log out and back in
  ```

## 4. Post-copy verification

Always do these in order, **before** touching any hardware.

### Step 1 — fresh venv

```bash
cd <lab CS225A root>/flexiv_rdk_existing/project/injectable_pipeline/
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

If `flexivrdk` is already system-installed and `pip install` complains about
network access, install only the four runtime deps that lack offline
copies:

```bash
pip install pyyaml pytest pyserial spdlog
```

…and let the venv inherit `flexivrdk` from the system Python via:

```bash
python3 -m venv --system-site-packages .venv
```

### Step 2 — pytest baseline

```bash
pytest tests/ -q
```

Must report **107 passed**. If anything fails, stop and investigate before
proceeding. Common causes:

- `import flexivrdk` fails inside the venv → use `--system-site-packages`
  or pip-install `flexivrdk` into the venv.
- `import yaml` fails → `pip install pyyaml`.
- Tests that touch serial fail with permission errors → not yet an issue,
  the tests only use the fake serial. If you see them, something is wrong.

### Step 3 — dry-run

```bash
python pipeline_orchestrator.py --dry-run --once \
    --pose-file tests/fixtures/pipeline_poses_valid.yaml
```

Must print the full six-phase cycle and exit `0`. This proves the imports,
the PARAMS block, and the state machine all resolve without hardware.

### Step 4 — Arduino smoke (with hardware)

Only after Steps 1–3 pass. With the Arduino plugged in:

```bash
python -m arduino_client status
```

Expect a `DONE` dict with `homed=false busy=false faulted=false vise_state=OPEN`
(or whatever the firmware reports). Confirms the lab's serial port is
discoverable and the firmware speaks the expected protocol.

## 5. Using Claude Code on the lab machine

The directory contains three operator-facing prompts under `planning/`. Each is
designed to be pasted into a fresh Claude Code session:

| Prompt file | When to use |
| --- | --- |
| `planning/phase3_implementation_prompt.md` | The original M1–M12 brief. M1–M4.5 already done; not needed on the lab machine. |
| `planning/m4_5_fix_prompt.md` | Pre-HIL correctness pass. Already executed on dev. Not needed on the lab machine. |
| `planning/inlab_implementation_prompt.md` | **Use this one.** M3 HIL through M11. The lab-day prompt. |

To start a lab session:

1. Open Claude Code on the lab machine. Set the workspace folder to the lab's
   `flexiv_rdk_existing/` directory (one level above `project/`) so paths like
   `../../example_py/...` resolve correctly.
2. Open `injectable_pipeline/planning/inlab_implementation_prompt.md`.
3. Copy everything between the `---` markers (it is a long prompt, ~1200
   words).
4. Paste it into a fresh Claude Code session.
5. The agent reads the prompt, walks the hardware preconditions checklist with
   the operator, runs the Phase 0 smoke tests, and proceeds through M3 HIL,
   M5, M6, …

The agent is told to pause and check in with the operator at every milestone
boundary. The operator decides whether to continue.

## 6. Files the lab session will create

These do not exist on the dev machine and will appear in `injectable_pipeline/`
as the lab session progresses:

- `pipeline_poses.yaml` — operator-captured poses. Created by
  `train_pipeline_poses.py` during the M3 HIL phase. Required by the
  orchestrator on every subsequent HIL run.
- `tests/hardware_log.md` — append-only log of HIL test outcomes. Created on
  first PASS entry during Phase 0 smoke tests.
- `logs/` — per-cycle orchestrator logs once M11 lands. Path is configurable
  via `PARAMS["LOG_DIR"]`.

If you want to bring those back to the dev machine after a lab session,
reverse the tarball process — exclude the venv on the lab side and copy back.

## 7. Common gotchas

- **Wrong Python version on the lab machine.** If `python3` is 3.8 or 3.9, the
  code uses type-hint syntax (`dict | None`, `list[str]`) that requires 3.10+.
  Update Python before running.
- **`flexivrdk` version mismatch.** If the lab has 2.x installed instead of
  1.9, expect runtime errors at primitive dispatch. The code's defensive
  `hasattr` checks paper over much of this, but log carefully on the first
  HIL run. The H3 log line in `RobotSession.__enter__` will print which API
  surface is detected.
- **USB drive filesystem strips Unix permissions.** FAT32/exFAT USB drives
  do not preserve `+x` bits. If `python pipeline_orchestrator.py` fails
  with "permission denied," run via `python3 pipeline_orchestrator.py`
  explicitly. The scripts work as `python file.py` regardless of shebang.
- **Line endings.** WSL → USB → Linux preserves LF endings as long as you
  use `tar`/`cp` from the WSL terminal. If you use Windows Explorer to
  drag files, .py and .md may end up with CRLF; Python tolerates this on
  Linux but YAML may not. The tarball recipe in Section 1 avoids the issue.
- **Tarball over 50 MB.** Means the venv leaked in. Rebuild with the exact
  `--exclude` patterns from Section 1.

## 8. If you need to roll back

The dev machine still has the canonical copy of every file. If the lab
machine's state ever diverges (uncommitted edits, mis-runs, anything), the
recovery path is:

1. On dev: rebuild the tarball.
2. On USB → lab: delete `flexiv_rdk_existing/project/injectable_pipeline/`
   (after backing up `pipeline_poses.yaml` and `tests/hardware_log.md`).
3. Extract the fresh tarball.
4. Restore the two operator-created files.
5. Re-create the venv and re-install requirements.

## 9. Bringing lab outputs back

After a lab session, two artifacts will exist that did not on dev:

- `injectable_pipeline/pipeline_poses.yaml`
- `injectable_pipeline/tests/hardware_log.md`

Copy them back to dev via USB so the dev machine has the latest pose data and
the test log. The simplest approach:

```bash
# On the lab machine
cd <lab CS225A root>/flexiv_rdk_existing/project/injectable_pipeline/
tar -czf /tmp/lab_outputs.tar.gz pipeline_poses.yaml tests/hardware_log.md
```

Copy that tarball back to dev and extract over the existing
`injectable_pipeline/` so the planning docs and code stay in sync between
machines.
