# XZ Stage Code v2

PlatformIO firmware for the Arduino-controlled X/Z stage, rotary axis, vise, and
cutting-machine serial protocol used by the OpenTIR cell.

## Layout

- `src/` — firmware entrypoint and serial/primitive implementations
- `platformio.ini` — PlatformIO environment for `megaatmega2560`
- `PIN_MAPPING.md` — hardware pin assignments
- `PRIMITIVES.md` — supported command/primitive notes
- `MOTOR_SPEEDS.md` — speed tuning notes
- `ABX00087-datasheet.pdf` — board datasheet reference

## Build

```bash
cd /home/src0/flexiv_rdk/project/arduino/xz-stage-code-v2
pio run
```

## Upload

```bash
cd /home/src0/flexiv_rdk/project/arduino/xz-stage-code-v2
pio run --target upload
pio device monitor --baud 115200
```
