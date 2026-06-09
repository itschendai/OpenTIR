# Primitive-Based Serial API for the Cutting Machine

## Summary
Recommend a **12-primitive layered API** for robot integration.

This is the best balance for the current firmware: small enough to keep the serial surface stable, but large enough that the upstream Python robot does not need to emulate keyboard jogs, infer hidden state, or guess when motions are done.

## Review Findings
- The current serial interface is keyboard-style, not robot-style: single-character commands plus a global step size (`P###`) in [src/main.cpp](</d:/Stanford/ME310/XZ Stage Code v2/src/main.cpp:330>) and [src/main.cpp](</d:/Stanford/ME310/XZ Stage Code v2/src/main.cpp:486>). That is fragile for automation.
- Motion is mostly **relative step jogging**. X and Z are tracked in mm internally, but there are no absolute move primitives exposed; rotary and vise are also not exposed as deterministic absolute commands.
- Homing is the only place where limit switches are used to stop motion. During normal moves, limit switches are only printed, not enforced as safety stops, in [src/main.cpp](</d:/Stanford/ME310/XZ Stage Code v2/src/main.cpp:621>).
- The vise/load-cell behavior needs cleanup before it becomes a production primitive. The force-based seek currently drives **M2 (Z)**, while the vise motor is **M4**, and the “closed” threshold (`5.0 kg`) does not match the seek stop threshold (`3.0 kg`).
- There are no protocol, state-machine, or safety tests in `test/`.

## Recommended Primitive Set
### Core production primitives: 8
- `home_all()`
- `move_x_abs(x_mm)`
- `move_z_abs(z_mm)`
- `rotate_abs(theta_deg)`
- `move_rel(axis, delta)`  
  Default debug/setup primitive for X, Z, rotary; not the main automation path.
- `set_blade(on)`
- `close_vise()`  
  Implement as a guarded compound action, not a raw step jog.
- `open_vise()`

### Safety and observability primitives: 4
- `stop_all()`
- `get_status()`
- `get_force()`
- `clear_faults()`

**Total: 12 primitives**

Do not go lower than this for a larger robot system. If you cut this down to the 7 example actions only, the external Python layer will have to recreate status, busy handling, stop behavior, and error recovery, which is exactly the logic that belongs in firmware.

## Public Interface / Behavior
- Replace the current one-char parser with a **line-based ASCII command API**. One command per line, named verbs, typed arguments.
- Use **one active machine command at a time**. Every mutating command returns:
  - `ACK <cmd_id>`
  - later `DONE <cmd_id>` or `ERR <cmd_id> <code> <message>`
- `get_status()` returns one structured snapshot with:
  - `homed`, `busy`, `faulted`
  - `x_mm`, `z_mm`, `rot_deg`
  - `blade_on`
  - `vise_state`
  - `force_kg`
  - `x_limit`, `z_limit`
  - active command name
- Absolute motion commands are rejected unless homing has completed.
- `stop_all()` immediately stops motion, clears the active command, and turns the blade off.
- `close_vise()` and `open_vise()` should become **stateful actions** with completion criteria. Do not leave them as raw step commands.
- Add **soft travel limits** for X and Z and abort on unexpected limit-switch activation outside homing.
- Keep workflow ownership in the external robot. The firmware should expose machine primitives, not full cut recipes.

## Test Plan
- Boot with all sensors present: verify `get_status()` reports ready, not busy, not faulted.
- Boot with AS5600 missing: verify `home_all()` fails cleanly and reports a fault.
- Call any absolute move before homing: verify rejection with a clear error.
- Run `home_all()`: verify rotary homes first, then X, then Z, and status reports zeroed coordinates.
- Run `move_x_abs`, `move_z_abs`, and `rotate_abs`: verify `ACK`, motion, `DONE`, and final reported position.
- Trigger a limit switch during a normal move: verify motion stops and a fault is raised.
- Run `close_vise()` and `open_vise()`: verify deterministic completion and correct `vise_state`.
- Run `stop_all()` during motion and with blade on: verify all motion stops and blade turns off.
- Send malformed or unknown commands: verify parser returns `ERR` without changing machine state.

## Assumptions and Defaults
- Chosen defaults: **layered API**, **absolute + relative motion**, and **explicit status/query primitives**.
- The upstream Python robot is responsible for sequencing cuts; the firmware is responsible for machine safety, state, and actuator execution.
- Rotary position will be reported in degrees relative to the AS5600-defined home.
- X and Z absolute ranges must be measured and configured before enabling production absolute moves.
- Vise control should be redesigned so its completion logic matches the actual clamp actuator and force sensor path.
