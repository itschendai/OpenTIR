"""One-shot: clamp the cutting-machine vise.

Auto-homes the Arduino first (CLOSE/OPEN_VISE require X + rotary at home).
"""

from arduino_client import ArduinoClient, DEFAULT_READY_TIMEOUT_S

VISE_TARGET_FORCE_KG = 6.0

arduino = ArduinoClient(
    ready_timeout_s=DEFAULT_READY_TIMEOUT_S, done_timeout_s=60.0
)
arduino.connect()
try:
    if not arduino.get_status().get("homed"):
        arduino.home_all()
    print(arduino.close_vise(target_force_kg=VISE_TARGET_FORCE_KG))
finally:
    arduino.close()
