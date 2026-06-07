"""Run the cut strategy you're currently testing.

Assumes the vise is already clamped on a part. Use ``open_vise.py`` /
``close_vise.py`` to manage the clamp between runs.

Edit ``cut_strategy()`` to change the cuts. Each ``cut(arduino, ...)`` is
one cut on the currently-clamped sample. The rotary is auto-zeroed before
every cut (firmware precondition).
"""

from __future__ import annotations

import sys

from arduino_client import ArduinoClient, DEFAULT_READY_TIMEOUT_S


DEFAULT_Z_MM = 134.0
DEFAULT_X_MM = 111.0
DEFAULT_DEG  = 360.0

# Per-axis speed overrides. None = let the firmware pick its default.
SLOW_CUT_FEED_MM_S = 10      # X engagement / retract while the blade is engaged
FAST_TRAVEL_MM_S   = 200     # non-cut X/Z moves
# Rotary speed unit is firmware-defined and not documented in PRIMITIVES.md;
# a probe with speed=30 timed out (> 60 s to complete 360°), so the scale
# isn't deg/sec. Leaving both at None falls back to the firmware default —
# tune empirically by setting to a number and timing the result.
ROT_CUT_SPEED      = 60   # rotary sweep speed
ROT_RETURN_SPEED   = 3000   # rotary unwind / fast return


def cut(arduino, *, z=DEFAULT_Z_MM, x=DEFAULT_X_MM, deg=DEFAULT_DEG):
    """Fire one cut at (z, x, deg). Rotary auto-zeroed first."""
    print(f"CUT  z={z} mm  x={x} mm  deg={deg}")
    arduino.rotate_abs(0.0)
    result = arduino.cut_height(z_mm=z, x_mm=x, deg=deg)
    print(
        f"  done: x={result.get('x_mm')} z={result.get('z_mm')} "
        f"rot={result.get('rot_deg')}"
    )
    return result


def cut_strategy(arduino):
    """EDIT THIS to change the cut sequence for the next run."""
    # first cut
    # cut(arduino, x=110, z=90.0)
    # cut(arduino, x=110.8, z=90.0)
    # second cut
    # cut(arduino, x=110.0, z=134.0)
    # cut(arduino, x=110.8, z=134.0)

    # Manual expansion of cut(arduino, x=110.8, z=90.0).
    #
    # Firmware-side CUT_HEIGHT sequence (PRIMITIVES.md §CUT_HEIGHT, line 287):
    #     Move Z to z_mm, move X to 100.0, blade on, move X *slowly* to x_mm,
    #     rotate to deg, move X *slowly* back to 100.0, blade off, rotate back
    #     to 0.0 at *fast return* speed, move X/Z to 0.0, then run HOME_ALL.
    #
    # The "slow" cut feed and "fast return" rotary speed are firmware-internal
    # constants. The primitives below pass no feed/speed override, so they use
    # the firmware defaults — speeds may not exactly match the composite
    # CUT_HEIGHT. To tune, add feed_mm_s= / speed= keyword args explicitly.
    z_mm = 148.0
    x_mm = 110.8
    deg  = DEFAULT_DEG  # 360.0

    # Precondition: rotary at 0 (firmware also asserts this).
    arduino.rotate_abs(0.0, speed=ROT_RETURN_SPEED)

    arduino.move_z_abs(z_mm, feed_mm_s=FAST_TRAVEL_MM_S)        # Move Z to z_mm
    arduino.move_x_abs(100.0, feed_mm_s=FAST_TRAVEL_MM_S)       # Move X to 100.0 (intermediate staging)
    arduino.set_blade(True)                                      # Blade on
    arduino.move_x_abs(110, feed_mm_s=SLOW_CUT_FEED_MM_S)        # Slow X in to cut depth
    arduino.rotate_abs(deg, speed=ROT_CUT_SPEED)                 # Rotary sweep — the actual cut
    arduino.move_x_abs(100, feed_mm_s=SLOW_CUT_FEED_MM_S)
    arduino.rotate_abs(0.0, speed=ROT_RETURN_SPEED)              # unwind the cable, return
    arduino.move_x_abs(110.8, feed_mm_s=SLOW_CUT_FEED_MM_S)      # second cut
    arduino.rotate_abs(deg, speed=ROT_CUT_SPEED)
    arduino.move_x_abs(100, feed_mm_s=SLOW_CUT_FEED_MM_S)
    arduino.rotate_abs(0.0, speed=ROT_RETURN_SPEED)              # unwind the cable, return
    arduino.set_blade(False)                                     # Blade off
    arduino.move_x_abs(0.0, feed_mm_s=FAST_TRAVEL_MM_S)          # X home
    arduino.move_z_abs(0.0, feed_mm_s=FAST_TRAVEL_MM_S)          # Z home


def main() -> int:
    arduino = ArduinoClient(
        ready_timeout_s=DEFAULT_READY_TIMEOUT_S, done_timeout_s=60.0
    )
    arduino.connect()
    try:
        if not arduino.get_status().get("homed"):
            arduino.home_all()
        cut_strategy(arduino)
        return 0
    finally:
        try:
            arduino.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
