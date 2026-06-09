# Motor Speed Reference

This document summarizes the motion speeds currently configured in `src/primitives.cpp`.

## Conversion basis

### X and Z axes

The firmware currently assumes:

- `STEPS_PER_REV_NEMA17 = 200`
- `MM_PER_REV = 8`
- `STEPS_PER_MM = 25`

So:

- `1 step/s = 0.04 mm/s`
- `mm/s = steps/s / 25`

## Vise axis

The firmware currently assumes:

- `VISE_MOTOR_STEPS_PER_REV = 200`
- `VISE_GEAR_RATIO = 51`
- `VISE_OUTPUT_STEPS_PER_REV = 10200`

So:

- `1 step/s = 0.035294 deg/s` at the vise output shaft
- `deg/s = steps/s * 360 / 10200`

## Rotary axis caveat

The rotary-stage constants and command arguments are named as if they are in `deg/s`, and this is how the API should be interpreted.

However, the current code sends those values directly into `AccelStepper` for `m3` without a separate motor-steps-to-degrees conversion. That means the rotary numbers below are the commanded firmware values, not a mechanically calibrated physical `deg/s` proven by this code alone.

## X and Z speeds

| Operation | Firmware value | Converted speed |
| --- | --- | --- |
| Default `MOVE_X_ABS` / `MOVE_Z_ABS` with no `feed` override | `SPEED = 100000 steps/s` | `4000 mm/s` |
| Default X/Z leg inside `CUT_HEIGHT` when no slow feed is specified | `SPEED = 100000 steps/s` | `4000 mm/s` |
| X/Z homing approach | `HOMING_SPEED = 750 steps/s` | `30 mm/s` |
| X/Z homing backoff | `HOMING_BACKOFF_SPEED = 40 steps/s` | `1.6 mm/s` |
| `CUT_HEIGHT` slow X advance into cut | `CUT_HEIGHT_X_SLOW_FEED = 5 mm/s` | `5 mm/s` |
| `CUT_HEIGHT` slow X retract from cut | `CUT_HEIGHT_X_SLOW_FEED = 5 mm/s` | `5 mm/s` |

### X/Z command notes

- `MOVE_X_ABS` and `MOVE_Z_ABS` accept `feed=<float>` directly in `mm/s`.
- `MOVE_REL axis=X` and `MOVE_REL axis=Z` also accept `feed=<float>` in `mm/s`.
- When a `feed` is provided, the firmware converts it with `feed * STEPS_PER_MM`.

## Rotary stage speeds

| Operation | Firmware value | Interpreted unit |
| --- | --- | --- |
| Default `ROTATE_ABS` / `MOVE_REL axis=ROT` speed | `ROTARY_MOVE_SPEED = 60` | commanded `60 deg/s` |
| `ROTATE_ABS` / `MOVE_REL axis=ROT` with `speed=<float>` override | caller-provided value | commanded `deg/s` |
| Rotary homing coarse speed | `ROTARY_HOME_SPEED = 600` | commanded `600 deg/s` |
| Rotary homing fine speed near home | `ROTARY_HOME_FINE_SPEED = 150` | commanded `150 deg/s` |
| `CUT_HEIGHT` forward rotation to `365 deg` | `CUT_HEIGHT_ROTARY_CUT_SPEED = 60` | commanded `60 deg/s` |
| `CUT_HEIGHT` return rotation back to `0 deg` | `CUT_HEIGHT_ROTARY_RETURN_SPEED = 3000` | commanded `3000 deg/s` |

### Rotary command notes

- Normal rotary moves no longer use the old near-target slow-down logic.
- Rotary homing still slows down near home using `ROTARY_HOME_FINE_SPEED`.

## Vise speeds

| Operation | Firmware value | Converted speed at vise output |
| --- | --- | --- |
| `CLOSE_VISE` low-force approach while force `< 1.0 kg` | `VISE_LOW_FORCE_SPEED_STEPS_PER_SEC = 5000 steps/s` | `176.47 deg/s` |
| `CLOSE_VISE` loaded closing once force `>= 1.0 kg` | `VISE_CLOSE_SPEED_STEPS_PER_SEC = 1000 steps/s` | `35.29 deg/s` |
| `CLOSE_VISE` clutch-release backoff | `VISE_CLOSE_SPEED_STEPS_PER_SEC = 1000 steps/s` | `35.29 deg/s` |
| `OPEN_VISE` while force is still high | `VISE_OPEN_SPEED_STEPS_PER_SEC = 1000 steps/s` | `35.29 deg/s` |
| `OPEN_VISE` once force drops below `1.0 kg` | `VISE_LOW_FORCE_SPEED_STEPS_PER_SEC = 5000 steps/s` | `176.47 deg/s` |
| `OPEN_VISE` extra slack move | `VISE_LOW_FORCE_SPEED_STEPS_PER_SEC = 5000 steps/s` | `176.47 deg/s` |
| `OPEN_VISE` final clutch-release move | `VISE_OPEN_SPEED_STEPS_PER_SEC = 1000 steps/s` | `35.29 deg/s` |

## Practical summary

- X/Z default motion is currently configured very high at `4000 mm/s` unless a lower `feed` is supplied.
- X/Z homing approaches at `30 mm/s` and backs off at `1.6 mm/s`.
- `CUT_HEIGHT` uses `5 mm/s` for the slow cutting X moves.
- Vise motion alternates between about `35.29 deg/s` and `176.47 deg/s` at the output shaft, depending on force state.
- Rotary motion is commanded as `60`, `150`, or `600` in firmware, but should be mechanically calibrated if you want guaranteed real-world `deg/s`.
