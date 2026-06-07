"""Static configuration for the cutting-cell GUI.

Hardware identifiers and paths are centralized here so the controllers stay free
of magic constants. Values mirror what the existing CLI scripts and recipes use.
"""

from __future__ import annotations

from pathlib import Path

# ----- filesystem layout -----
GUI_DIR = Path(__file__).resolve().parent
PROJECT_DIR = GUI_DIR.parent                       # /home/src0/flexiv_rdk/project
KEY_POSITIONS_DIR = PROJECT_DIR / "key_positions"  # global saved waypoints
RECIPE_DIR = PROJECT_DIR / "recipe"                # recipe/<name>/recipe.py

# ----- robot -----
ROBOT_SN = "Rizon4-062930"
GRIPPER_NAME = "Flexiv-GN01"
GRIPPER_INIT = True
# Gripper open/close presets (metres / m s^-1 / N), matching gripper_open_close.py.
GRIPPER_OPEN_WIDTH_M = 0.04
GRIPPER_CLOSE_WIDTH_M = 0.0
GRIPPER_VELOCITY_M_S = 0.1
GRIPPER_OPEN_FORCE_N = 10.0
GRIPPER_CLOSE_FORCE_N = 40.0

# Velocity scale (1-100) for move-to-waypoint MovePTP from the control panel.
MOVE_JNT_VEL_SCALE = 20

# ----- camera (Intel RealSense D405) -----
CAMERA_SERIAL = "323622271112"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
CAMERA_WARMUP_FRAMES = 30
CAMERA_JPEG_QUALITY = 80

# ----- Arduino cutting machine -----
ARDUINO_PORT = None        # None -> autodetect under /dev/serial/by-id and /dev/ttyACM*
ARDUINO_BAUD = 115200
ARDUINO_READY_TIMEOUT_S = 5.0

# ----- web server -----
HOST = "0.0.0.0"
PORT = 5000
# How often the Arduino status cache is refreshed (skipped while a long command runs).
MACHINE_POLL_INTERVAL_S = 1.0
