# XZ Stage Pin Mapping

Board target: Arduino Mega 2560 (`megaatmega2560`)

This document reflects the current firmware in `src/main.cpp` and your current breakout-board wiring state.

## Pins Used By Firmware

| Pin | Signal | Purpose | Notes |
| --- | --- | --- | --- |
| D2 | `STEP4` | Motor 4 step | Vise motor step input |
| D3 | `DIR4` | Motor 4 direction | Vise motor direction input |
| D4 | `ENA4` | Motor 4 enable | Vise motor enable input |
| D25 | `BLADE_RELAY` | Blade relay control | `HIGH = on`, `LOW = off` |
| D10 | `STEP3` | Motor 3 step | Rotation stage step input |
| D11 | `DIR3` | Motor 3 direction | Rotation stage direction input |
| D12 | `ENA3` | Motor 3 enable | Rotation stage enable input |
| D34 | `STEP1` | Motor 1 step | X stage step input |
| D35 | `STEP2` | Motor 2 step | Z stage step input |
| D36 | `DIR1` | Motor 1 direction | X stage direction input |
| D37 | `DIR2` | Motor 2 direction | Z stage direction input |
| D38 | `ENA1` | Motor 1 enable | X stage enable input |
| D39 | `ENA2` | Motor 2 enable | Z stage enable input |
| A0 | `LOADCELL_DT` | HX711 load cell data | Added for load cell interface |
| D7 | `LIMIT_SWITCH_2` | Limit switch 2 input | Configured as `INPUT_PULLUP`, active low |
| A2 | `LIMIT_SWITCH_1` | Limit switch 1 input | Configured as `INPUT_PULLUP`, active low |
| A4 | `LOADCELL_SCK` | HX711 load cell clock | Added for load cell interface |
| D20 / SDA | I2C SDA | AS5600 angle sensor data | Used by `Wire.begin()` on Mega 2560 |
| D21 / SCL | I2C SCL | AS5600 angle sensor clock | Used by `Wire.begin()` on Mega 2560 |
| D0 | Serial RX | USB/serial communication | Used by `Serial.begin()` |
| D1 | Serial TX | USB/serial communication | Used by `Serial.begin()` |

## Pins Currently Not Connected On Breakout Board

You noted these are currently available on the breakout board:

| Pin | Breakout status | Firmware status | Recommendation |
| --- | --- | --- | --- |
| A0 | Depends on wiring | Used for HX711 DT | Reserved for load cell |
| A4 | Depends on wiring | Used for HX711 SCK | Reserved for load cell |
| A1 | Available after remap | Not used by current firmware | Free unless reassigned later |
| D0 | Not connected | Reserved for serial | Avoid reusing unless serial is no longer needed |
| D1 | Not connected | Reserved for serial | Avoid reusing unless serial is no longer needed |
| D20 | Depends on wiring | Reserved for I2C SDA | Required for AS5600 on Mega 2560 |
| D21 | Depends on wiring | Reserved for I2C SCL | Required for AS5600 on Mega 2560 |
| D5 | Available after remap | Not used by current firmware | Free unless reassigned later |
| D6 | Available after remap | Not used by current firmware | Free unless reassigned later |
| D8 | Available after remap | Not used by current firmware | Free unless reassigned later |
| D9 | Available after relay remap | Not used by current firmware | Free unless reassigned later |
| D13 | Not connected | Unused by project logic | Still tied to the onboard LED on Mega 2560 |
| D34-D39 | Depends on wiring | Assigned to M1/M2 driver inputs | Use for X and Z stage drivers |

## Best Truly Free Pins Right Now

These are the safest pins to reuse without changing the current firmware design:

- `D5` and `D6` are now free after moving M1 and M2.
- `D40-D49` are plain digital pins and are generally easier to repurpose than `D50-D53`.
- `D50-D53` are also usable as GPIO, but they overlap with the Mega's SPI pins.

## Use With Caution

- `D0` and `D1` are physically open on the breakout board, but the firmware uses them for serial communication.
- `D2`, `D3`, and `D4` are now assigned to the vise motor driver.
- `D7` is now assigned to the X-home limit switch.
- `D25` is now assigned to the blade relay.
- `D20` and `D21` must be wired for I2C / AS5600 on the Mega 2560.
- `A0` and `A4` are now assigned to the HX711 load cell interface.
- `D34-D39` are now assigned to the X and Z stage motor drivers.
- `D50-D53` can be used as GPIO if needed, but they are also the Mega's SPI pins.
- `D13` is not assigned to a device in this project, but it is connected to the Mega 2560 onboard LED.

## Wiring Note

Do not hard-wire unused GPIO pins directly to `GND` or `5V` unless the circuit is specifically designed for that. If a pin is later configured as an output and driven the opposite way, it can create a short. If you need a default state, use a pull-up or pull-down resistor instead.
