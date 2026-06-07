# Cutting Cell GUI

A single-screen web GUI (HTML frontend + Flask/Python backend) for the Flexiv
cutting cell. One process owns all hardware and exposes it over HTTP.

## What it does

1. **Recipe** — list recipes from `../recipe/*/recipe.py`, load one (shows its
   phases + key positions), then **Run Full** (with a loop count), **Next Phase**
   to step phase-by-phase, and a **Speed** slider that scales the recipe's
   velocity params. **Stop** halts the robot and the Arduino.
2. **Robot control** — Home, Move-to saved waypoint, gripper open/close,
   Cartesian/joint floating on/off, Cali (zero F/T), clear fault, stop.
3. **Camera** — live RGB MJPEG stream from the RealSense D405 (depth ignored).
   `CameraBroker.overlay()` is the seam for plugging in injectable/tag detection.
4. **Robot status** — joint angles, TCP position + orientation (RPY), gripper
   width/force/moving, external wrench, fault/operational.
5. **Machine** — X/Z stage position, rotary angle, vise state + force; controls
   for home-all, vise open/close, X/Z/rotary moves.
6. **Cutting machine** — blade on/off and the full `cut_height(z, x, deg)`
   sequence (with a confirm dialog).

## Run

```bash
cd /home/src0/flexiv_rdk/project
/home/src0/flexiv_rdk/.venv/bin/python -m pip install -r GUI/requirements.txt   # once
/home/src0/flexiv_rdk/.venv/bin/python GUI/app.py
```

Then open <http://localhost:5000>. The robot, RealSense camera, and Arduino must
all be connected — startup fails loudly (naming the device) if one is missing.

## Architecture

- `app.py` — Flask routes; MJPEG camera stream; JSON status; control endpoints.
- `hardware.py` — `HardwareHub` builds/owns the handles; `OperationExecutor`
  runs **one** mutating operation at a time (every move/cut/vise/recipe);
  `BufferLogger` backs the recipe log panel.
- `controllers/robot_controller.py` — reuses `helper/flexiv_helpers.py`
  (`RobotSession`, `move_ptp_joint`, `gripper_set`, floating, `zero_ft_sensor`).
- `controllers/machine_controller.py` — wraps `helper/arduino_client.py`; a poll
  thread caches status and pauses while a long Arduino command runs.
- `controllers/camera_broker.py` — wraps `helper/injectable_camera_session.py`.
- `controllers/recipe_runner.py` — imports a recipe module and drives it via an
  injected `RecipeContext` backed by the GUI's hardware (no second process).

Config (serial numbers, ports, camera settings, speeds) lives in `config.py`.

## Notes / limits

- Long Arduino ops (`cut` ≈ 60 s, vise up to minutes) block the serial port, so
  the machine status shows the last cached values plus the active command name
  until the op completes.
- Step-through uses each recipe's `phase_*` functions in source order (or a
  `GUI_PHASES` list if a recipe defines one).
