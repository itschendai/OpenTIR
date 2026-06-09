# OpenTIR

Robotics automation project built on the [Flexiv RDK](https://github.com/flexivrobotics/flexiv_rdk),
combining robot waypoint recording/playback, RealSense camera vision, eye-in-hand
calibration, and a recipe-driven pipeline with a Flask GUI.

## Requirements

- Python **3.10+** (developed on 3.12)
- A Flexiv robot reachable on the network (for the robot-control scripts)
- An Intel RealSense camera (for the vision/calibration scripts)

## Setup

```bash
# 1. Create and activate a virtual environment
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate

# 2. Install dependencies
pip install --upgrade pip
pip install -r requirements.txt
```

### Segment Anything (SAM)

Some vision code uses Meta's Segment Anything, which is **not on PyPI** and is **not
committed to this repo**. Install it and download the model checkpoint separately:

```bash
pip install git+https://github.com/facebookresearch/segment-anything.git

# Download the ViT-H checkpoint (~2.4 GB) into the project root:
wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth
```

## What's not in this repo

The `.gitignore` intentionally excludes regenerable / large artifacts so the repo
stays small:

- `.venv/` — virtual environment (rebuild with the steps above)
- `__pycache__/`, `.pytest_cache/` — Python caches
- `*.mp4` and `calibration/captures|samples|holdout/` — large media/capture data
- the SAM model checkpoint (`sam_vit_h_4b8939.pth`)

## Project layout

| Path | Purpose |
|------|---------|
| `record_robot_waypoints.py` | Record robot waypoints/trajectories |
| `play_recorded_waypoints.py` | Replay recorded waypoints |
| `gripper_open_close.py` | Gripper control helper |
| `calibration/` | Eye-in-hand calibration scripts and configs |
| `camera/` | RealSense camera capture utilities |
| `GUI/` | Flask-based control GUI |
| `Simulation/` | Flexiv Element motion simulation helpers for recipe motions |
| `recipe/` | Recipe definitions and per-recipe key positions |
| `helper/` | Shared helper modules |
| `key_positions/` | Saved robot key positions |
| `injectable_pipeline_archived/` | Archived injectable-processing pipeline |

## Running

Activate the venv first (`source .venv/bin/activate`), then run the relevant script, e.g.:

```bash
python record_robot_waypoints.py
python play_recorded_waypoints.py
python Simulation/greenpink_fast_element_sim.py --dry-run
```

For the GUI, see `GUI/` (a Flask app; `GUI/requirements.txt` notes its extra dependency).
