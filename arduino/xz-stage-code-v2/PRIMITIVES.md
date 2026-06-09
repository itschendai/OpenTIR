# Cutting Machine Primitive Reference

This document defines the robot-facing serial API for the cutting machine. It is intended to be the command and response contract used by an external Python program or a larger robotic system.

Commands are ASCII text, one command per line, terminated by `\n` or `\r\n`.

Mutating commands return an immediate acknowledgement and then a completion or error response:
- `ACK <cmd_id> <command>`
- `DONE <cmd_id> <command> [key=value ...]`
- `ERR <cmd_id> <command> code=<code> message="<text>"`

Query commands return a single immediate `DONE` response or an `ERR` response.

## Protocol Conventions

### Command format

Commands use uppercase names and `key=value` arguments:

```text
COMMAND arg=value arg=value
```

Example:

```text
MOVE_X_ABS cmd_id=101 x_mm=42.0
```

### Response format

Mutating command acknowledgement:

```text
ACK 101 MOVE_X_ABS
```

Successful completion:

```text
DONE 101 MOVE_X_ABS x_mm=42.0 busy=false homed=true faulted=false
```

Error:

```text
ERR 101 MOVE_X_ABS code=NOT_HOMED message="Machine must be homed before absolute motion"
```

### Shared state fields

These fields may appear in success responses and are part of the machine state model:

- `busy`: `true` when a mutating command is in progress
- `homed`: `true` when the machine has completed homing and machine coordinates are valid
- `faulted`: `true` when the machine is in a fault state and further motion should be blocked until recovered

### General assumptions

- Absolute motion commands require successful homing first.
- The firmware accepts only one active mutating command at a time.
- `STOP_ALL` forces the blade off.
- Vise operations are guarded by machine safety checks.
- X and Z travel ranges are firmware-configured and enforced as soft limits.

## Primitive Reference

## `HOME_ALL`

### Purpose

Home the rotary stage first, then the X stage, then the Z stage, and define machine zero.

### Command

```text
HOME_ALL [cmd_id=<int>]
```

### Required inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| None | N/A | No | This command does not require any input arguments. |

### Optional inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `cmd_id` | integer | No | Caller-supplied command identifier used to correlate responses. |

### Success outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `x_mm` | float | Yes | Final X position after homing, normally `0.0`. |
| `z_mm` | float | Yes | Final Z position after homing, normally `0.0`. |
| `rot_deg` | float | Yes | Final rotary position after homing. Homing zeroes the rotary counter, so this is normally `0.0`. |
| `busy` | boolean | Yes | `false` when the homing sequence completes. |
| `homed` | boolean | Yes | `true` when machine coordinates are established. |
| `faulted` | boolean | Yes | `false` if homing completed without fault. |

### Error outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `code` | string | Yes | Error code such as `BUSY`, `SENSOR_MISSING`, `LIMIT_HIT`, or `TIMEOUT`. |
| `message` | string | Yes | Human-readable reason for the failure. |

### Notes / preconditions

- This command is allowed from an unhomed state.
- Homing should fail if the rotary encoder is unavailable or if limit-based homing does not complete safely.
- A mutating command already in progress should cause this command to return `ERR ... code=BUSY`.

### Example

```text
> HOME_ALL cmd_id=100
< ACK 100 HOME_ALL
< DONE 100 HOME_ALL x_mm=0.0 z_mm=0.0 rot_deg=0.0 busy=false homed=true faulted=false
```

## `MOVE_X_ABS`

### Purpose

Move the X stage to an absolute machine-coordinate position in millimeters.

### Command

```text
MOVE_X_ABS x_mm=<float> [feed=<float>] [cmd_id=<int>]
```

### Required inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `x_mm` | float | Yes | Target X position in machine millimeters. |

### Optional inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `feed` | float | No | Optional motion speed or feed override for this move. |
| `cmd_id` | integer | No | Caller-supplied command identifier used to correlate responses. |

### Success outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `x_mm` | float | Yes | Final X position reached by the move. |
| `busy` | boolean | Yes | `false` when the move completes. |
| `homed` | boolean | Yes | `true` for a valid absolute move result. |
| `faulted` | boolean | Yes | `false` if motion completed without fault. |

### Error outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `code` | string | Yes | Error code such as `NOT_HOMED`, `INVALID_ARG`, `BUSY`, or `LIMIT_HIT`. |
| `message` | string | Yes | Human-readable reason for the failure. |

### Notes / preconditions

- The machine must be homed before absolute X motion is accepted.
- `x_mm` must fall within configured soft travel limits.
- Unexpected limit-switch activation during the move should fault the machine.

### Example

```text
> MOVE_X_ABS cmd_id=101 x_mm=42.0
< ACK 101 MOVE_X_ABS
< DONE 101 MOVE_X_ABS x_mm=42.0 busy=false homed=true faulted=false
```

## `MOVE_Z_ABS`

### Purpose

Move the Z stage to an absolute machine-coordinate position in millimeters.

### Command

```text
MOVE_Z_ABS z_mm=<float> [feed=<float>] [cmd_id=<int>]
```

### Required inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `z_mm` | float | Yes | Target Z position in machine millimeters. |

### Optional inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `feed` | float | No | Optional motion speed or feed override for this move. |
| `cmd_id` | integer | No | Caller-supplied command identifier used to correlate responses. |

### Success outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `z_mm` | float | Yes | Final Z position reached by the move. |
| `busy` | boolean | Yes | `false` when the move completes. |
| `homed` | boolean | Yes | `true` for a valid absolute move result. |
| `faulted` | boolean | Yes | `false` if motion completed without fault. |

### Error outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `code` | string | Yes | Error code such as `NOT_HOMED`, `INVALID_ARG`, `BUSY`, or `LIMIT_HIT`. |
| `message` | string | Yes | Human-readable reason for the failure. |

### Notes / preconditions

- The machine must be homed before absolute Z motion is accepted.
- `z_mm` must fall within configured soft travel limits.
- Unexpected limit-switch activation during the move should fault the machine.

### Example

```text
> MOVE_Z_ABS cmd_id=102 z_mm=12.5
< ACK 102 MOVE_Z_ABS
< DONE 102 MOVE_Z_ABS z_mm=12.5 busy=false homed=true faulted=false
```

## `CUT_HEIGHT`

### Purpose

Run the full cutting sequence at a requested Z height using the currently closed vise.

### Command

```text
CUT_HEIGHT z_mm=<float> x_mm=<float> deg=<float> [cmd_id=<int>]
```

### Required inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `z_mm` | float | Yes | Target Z position for the cut in machine millimeters. |
| `x_mm` | float | Yes | Target X cut position in machine millimeters. |
| `deg` | float | Yes | Rotary angle to cut through in degrees on the cumulative rotary counter. |

### Optional inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `cmd_id` | integer | No | Caller-supplied command identifier used to correlate responses. |

### Success outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `x_mm` | float | Yes | Final X position after the sequence, normally `0.0`. |
| `z_mm` | float | Yes | Final Z position after the sequence, normally `0.0`. |
| `rot_deg` | float | Yes | Final rotary position after the sequence, normally `0.0`. |
| `blade_on` | boolean | Yes | Final blade relay state, normally `false`. |
| `busy` | boolean | Yes | `false` when the sequence completes. |
| `homed` | boolean | Yes | Current homing state after the sequence. |
| `faulted` | boolean | Yes | `false` if the sequence completed without fault. |

### Error outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `code` | string | Yes | Error code such as `NOT_HOMED`, `SENSOR_MISSING`, `INVALID_ARG`, `INVALID_STATE`, `BUSY`, `TIMEOUT`, or `LIMIT_HIT`. |
| `message` | string | Yes | Human-readable reason for the failure. |

### Notes / preconditions

- The machine must be homed before `CUT_HEIGHT` is accepted.
- The AS5600 encoder and load cell must both be available.
- The vise must already be closed above `3.0 kg` before the cut begins.
- The rotary stage must be at `0.0` before the cut begins.
- `x_mm` and `z_mm` must fall within configured soft travel limits.
- The firmware currently executes this internal sequence:
  Move Z to `z_mm`, move X to `100.0`, turn the blade on, move X slowly to `x_mm`, rotate to `deg`, move X slowly back to `100.0`, turn the blade off, rotate back to `0.0` at the fast return speed, move X/Z to `0.0`, then run the same homing sequence as `HOME_ALL`.

### Example

```text
> CUT_HEIGHT cmd_id=103 z_mm=12.5 x_mm=120.0 deg=365.0
< ACK 103 CUT_HEIGHT
< DONE 103 CUT_HEIGHT x_mm=0.0 z_mm=0.0 rot_deg=0.0 blade_on=false busy=false homed=true faulted=false
```

## `ROTATE_ABS`

### Purpose

Rotate the rotary stage to an absolute angle on the rotary counter, where home resets the counter to `0.0`.

### Command

```text
ROTATE_ABS deg=<float> [speed=<float>] [cmd_id=<int>]
```

### Required inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `deg` | float | Yes | Target rotary angle in degrees on the cumulative rotary counter. Values may be signed and may exceed one revolution, for example `-30` or `720`. |

### Optional inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `speed` | float | No | Optional rotary speed override for this move. |
| `cmd_id` | integer | No | Caller-supplied command identifier used to correlate responses. |

### Success outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `rot_deg` | float | Yes | Final rotary position reached by the move. |
| `busy` | boolean | Yes | `false` when the move completes. |
| `homed` | boolean | Yes | `true` for a valid absolute move result. |
| `faulted` | boolean | Yes | `false` if motion completed without fault. |

### Error outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `code` | string | Yes | Error code such as `NOT_HOMED`, `SENSOR_MISSING`, `INVALID_ARG`, or `BUSY`. |
| `message` | string | Yes | Human-readable reason for the failure. |

### Notes / preconditions

- The machine must be homed before absolute rotary motion is accepted.
- Homing zeroes the rotary counter used by this command.
- Rotary feedback depends on the AS5600 encoder being present and healthy.

### Example

```text
> ROTATE_ABS cmd_id=103 deg=90
< ACK 103 ROTATE_ABS
< DONE 103 ROTATE_ABS rot_deg=90.0 busy=false homed=true faulted=false
```

## `MOVE_REL`

### Purpose

Perform a relative move on one axis for setup, recovery, or debug workflows.

### Command

```text
MOVE_REL axis=<X|Z|ROT> delta=<float> [feed=<float>] [cmd_id=<int>]
```

### Required inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `axis` | string | Yes | Axis to move: `X`, `Z`, or `ROT`. |
| `delta` | float | Yes | Signed relative move amount in mm for `X` and `Z`, or degrees for `ROT`. |

### Optional inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `feed` | float | No | Optional motion speed or feed override for this move. |
| `cmd_id` | integer | No | Caller-supplied command identifier used to correlate responses. |

### Success outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `axis` | string | Yes | Axis that was moved. |
| `x_mm` | float | No | Updated X position when `axis=X`. |
| `z_mm` | float | No | Updated Z position when `axis=Z`. |
| `rot_deg` | float | No | Updated cumulative rotary position when `axis=ROT`. |
| `busy` | boolean | Yes | `false` when the move completes. |
| `homed` | boolean | Yes | Current machine homing state after the move. |
| `faulted` | boolean | Yes | `false` if motion completed without fault. |

### Error outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `code` | string | Yes | Error code such as `INVALID_ARG`, `NOT_HOMED`, `BUSY`, or `LIMIT_HIT`. |
| `message` | string | Yes | Human-readable reason for the failure. |

### Notes / preconditions

- `axis` must be one of `X`, `Z`, or `ROT`.
- Relative moves should still obey configured soft limits and safety rules.
- If the implementation requires homing for relative motion on a given axis, it should reject the command with `NOT_HOMED`.

### Example

```text
> MOVE_REL cmd_id=104 axis=X delta=-5.0
< ACK 104 MOVE_REL
< DONE 104 MOVE_REL axis=X x_mm=37.0 busy=false homed=true faulted=false
```

## `SET_BLADE`

### Purpose

Turn the ultrasonic blade relay on or off.

### Command

```text
SET_BLADE state=<ON|OFF> [cmd_id=<int>]
```

### Required inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `state` | string | Yes | Desired blade state: `ON` or `OFF`. |

### Optional inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `cmd_id` | integer | No | Caller-supplied command identifier used to correlate responses. |

### Success outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `blade_on` | boolean | Yes | `true` when the blade relay is on, `false` when off. |
| `busy` | boolean | Yes | Current machine busy state after the command. |
| `homed` | boolean | Yes | Current machine homing state after the command. |
| `faulted` | boolean | Yes | Current machine fault state after the command. |

### Error outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `code` | string | Yes | Error code such as `INVALID_ARG` or `FAULTED`. |
| `message` | string | Yes | Human-readable reason for the failure. |

### Notes / preconditions

- `SET_BLADE` is defined as a machine-output control primitive, not a motion command.
- The implementation may reject `state=ON` if the machine is faulted or in an unsafe condition.

### Example

```text
> SET_BLADE cmd_id=105 state=ON
< ACK 105 SET_BLADE
< DONE 105 SET_BLADE blade_on=true busy=false homed=true faulted=false
```

## `CLOSE_VISE`

### Purpose

Close the vise as a guarded clamp action and complete when the machine determines the vise is closed.

### Command

```text
CLOSE_VISE [target_force_kg=<float>] [cmd_id=<int>]
```

### Required inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| None | N/A | No | This command does not require any input arguments. |

### Optional inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `target_force_kg` | float | No | Optional clamp-force target. Defaults to `4.0 kg`. |
| `cmd_id` | integer | No | Caller-supplied command identifier used to correlate responses. |

### Success outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `vise_state` | string | Yes | Final vise state, expected to be `CLOSED`. |
| `force_kg` | float | Yes | Final measured clamp force in kilograms. |
| `busy` | boolean | Yes | `false` when the action completes. |
| `homed` | boolean | Yes | Current machine homing state after the action. |
| `faulted` | boolean | Yes | `false` if the action completed without fault. |

### Error outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `code` | string | Yes | Error code such as `BUSY`, `SENSOR_MISSING`, `TIMEOUT`, `FAULTED`, or `INVALID_STATE`. |
| `message` | string | Yes | Human-readable reason for the failure. |

### Notes / preconditions

- Vise operations are guarded and may require a safe machine pose before motion is allowed.
- If the load cell is part of the completion logic, missing force-sensor data should cause an error.
- This command is defined as a compound machine action, not a raw step jog.
- The vise command timeout is 5 minutes.
- A `TIMEOUT` or similar motion failure leaves the machine in `faulted=true`; the next mutating command will be rejected until `CLEAR_FAULTS` succeeds.

### Example

```text
> CLOSE_VISE cmd_id=106 target_force_kg=4.0
< ACK 106 CLOSE_VISE
< DONE 106 CLOSE_VISE vise_state=CLOSED force_kg=4.1 busy=false homed=true faulted=false
```

## `OPEN_VISE`

### Purpose

Open the vise as a guarded release action and complete when the machine determines the vise is open.

### Command

```text
OPEN_VISE [target_force_kg=<float>] [cmd_id=<int>]
```

### Required inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| None | N/A | No | This command does not require any input arguments. |

### Optional inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `target_force_kg` | float | No | Optional release threshold if the implementation supports force-based completion. |
| `cmd_id` | integer | No | Caller-supplied command identifier used to correlate responses. |

### Success outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `vise_state` | string | Yes | Final vise state, expected to be `OPEN`. |
| `force_kg` | float | Yes | Final measured force in kilograms after release. |
| `busy` | boolean | Yes | `false` when the action completes. |
| `homed` | boolean | Yes | Current machine homing state after the action. |
| `faulted` | boolean | Yes | `false` if the action completed without fault. |

### Error outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `code` | string | Yes | Error code such as `BUSY`, `SENSOR_MISSING`, `TIMEOUT`, `FAULTED`, or `INVALID_STATE`. |
| `message` | string | Yes | Human-readable reason for the failure. |

### Notes / preconditions

- Vise operations are guarded and may require a safe machine pose before motion is allowed.
- If the load cell is part of the completion logic, missing force-sensor data should cause an error.
- This command is defined as a compound machine action, not a raw step jog.
- The vise command timeout is 5 minutes.
- A `TIMEOUT` or similar motion failure leaves the machine in `faulted=true`; the next mutating command will be rejected until `CLEAR_FAULTS` succeeds.

### Example

```text
> OPEN_VISE cmd_id=107 target_force_kg=0.2
< ACK 107 OPEN_VISE
< DONE 107 OPEN_VISE vise_state=OPEN force_kg=0.1 busy=false homed=true faulted=false
```

## `STOP_ALL`

### Purpose

Immediately stop all motion, clear the active machine command, and force the blade off.

### Command

```text
STOP_ALL [cmd_id=<int>]
```

### Required inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| None | N/A | No | This command does not require any input arguments. |

### Optional inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `cmd_id` | integer | No | Caller-supplied command identifier used to correlate responses. |

### Success outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `blade_on` | boolean | Yes | Must be `false` after `STOP_ALL`. |
| `busy` | boolean | Yes | Must be `false` after the stop takes effect. |
| `homed` | boolean | Yes | Current machine homing state after the stop. |
| `faulted` | boolean | Yes | Current machine fault state after the stop. |

### Error outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `code` | string | Yes | Error code if the stop command itself cannot be applied. |
| `message` | string | Yes | Human-readable reason for the failure. |

### Notes / preconditions

- `STOP_ALL` is an interrupt-style safety primitive and should take precedence over ongoing motion.
- There should be no delayed motion completion response after a successful stop.

### Example

```text
> STOP_ALL cmd_id=108
< ACK 108 STOP_ALL
< DONE 108 STOP_ALL blade_on=false busy=false homed=true faulted=false
```

## `GET_STATUS`

### Purpose

Return a single snapshot of machine state for coordination, monitoring, and recovery.

### Command

```text
GET_STATUS [cmd_id=<int>]
```

### Required inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| None | N/A | No | This command does not require any input arguments. |

### Optional inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `cmd_id` | integer | No | Caller-supplied command identifier used to correlate responses. |

### Success outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `homed` | boolean | Yes | Current homing state. |
| `busy` | boolean | Yes | Current busy state. |
| `faulted` | boolean | Yes | Current fault state. |
| `x_mm` | float | Yes | Current X machine position in millimeters. |
| `z_mm` | float | Yes | Current Z machine position in millimeters. |
| `rot_deg` | float | Yes | Current cumulative rotary position. This counter is reset to `0.0` when the machine homes. |
| `blade_on` | boolean | Yes | Current blade relay state. |
| `vise_state` | string | Yes | Current vise state such as `OPEN`, `MOVING`, `CLOSED`, or `UNKNOWN`. |
| `force_kg` | float | Yes | Current measured vise force in kilograms. |
| `x_limit` | boolean | Yes | Current X-home limit-switch state. |
| `z_limit` | boolean | Yes | Current Z-home limit-switch state. |
| `active_command` | string | Yes | Active mutating command name, or `NONE` if idle. |

### Error outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `code` | string | Yes | Error code if status cannot be produced. |
| `message` | string | Yes | Human-readable reason for the failure. |

### Notes / preconditions

- `GET_STATUS` is a query command and should return a single immediate response.
- This is the main machine-state query primitive for the upstream robot.

### Example

```text
> GET_STATUS cmd_id=109
< DONE 109 GET_STATUS homed=true busy=false faulted=false x_mm=42.0 z_mm=12.5 rot_deg=90.0 blade_on=false vise_state=CLOSED force_kg=4.1 x_limit=false z_limit=false active_command=NONE
```

## `GET_FORCE`

### Purpose

Return the current force reading and interpreted vise state.

### Command

```text
GET_FORCE [cmd_id=<int>]
```

### Required inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| None | N/A | No | This command does not require any input arguments. |

### Optional inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `cmd_id` | integer | No | Caller-supplied command identifier used to correlate responses. |

### Success outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `force_kg` | float | Yes | Current measured vise force in kilograms. |
| `vise_state` | string | Yes | Current interpreted vise state such as `OPEN`, `MOVING`, `CLOSED`, or `UNKNOWN`. |
| `busy` | boolean | Yes | Current machine busy state. |
| `homed` | boolean | Yes | Current machine homing state. |
| `faulted` | boolean | Yes | Current machine fault state. |

### Error outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `code` | string | Yes | Error code such as `SENSOR_MISSING` or `TIMEOUT`. |
| `message` | string | Yes | Human-readable reason for the failure. |

### Notes / preconditions

- `GET_FORCE` is a query command and should return a single immediate response.
- If the load cell is unavailable, the command should return an error instead of a guessed force value.

### Example

```text
> GET_FORCE cmd_id=110
< DONE 110 GET_FORCE force_kg=4.1 vise_state=CLOSED busy=false homed=true faulted=false
```

## `CLEAR_FAULTS`

### Purpose

Attempt to clear the machine fault state after the underlying error condition has been resolved.

### Command

```text
CLEAR_FAULTS [cmd_id=<int>]
```

### Required inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| None | N/A | No | This command does not require any input arguments. |

### Optional inputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `cmd_id` | integer | No | Caller-supplied command identifier used to correlate responses. |

### Success outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `faulted` | boolean | Yes | `false` when the fault state has been cleared successfully. |
| `busy` | boolean | Yes | Current machine busy state after the clear attempt. |
| `homed` | boolean | Yes | Current machine homing state after the clear attempt. |

### Error outputs

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `code` | string | Yes | Error code such as `FAULTED` or `INVALID_STATE`. |
| `message` | string | Yes | Human-readable reason the fault could not be cleared. |

### Notes / preconditions

- `CLEAR_FAULTS` should not hide an active hardware problem.
- If the root cause is still present, this command should return an error and leave `faulted=true`.
- In the current firmware, `CLEAR_FAULTS` is rejected only if another command is still active or if the X/Z limit switch is still active.

### Example

```text
> CLEAR_FAULTS cmd_id=111
< DONE 111 CLEAR_FAULTS faulted=false busy=false homed=true
```

## Error Codes

The following standard error codes should be used consistently across the primitive interface:

| Code | Meaning |
| --- | --- |
| `NOT_HOMED` | Command requires a valid homed coordinate system, but the machine is not homed. |
| `BUSY` | Another mutating command is already in progress. |
| `FAULTED` | The machine is in a fault state and the command cannot proceed. |
| `INVALID_ARG` | A required argument is missing, malformed, or out of allowed range. |
| `LIMIT_HIT` | Motion encountered a limit switch or travel boundary unexpectedly. |
| `SENSOR_MISSING` | Required sensor data is unavailable, such as the AS5600 or load cell. |
| `TIMEOUT` | The command did not complete within its allowed time window. |
| `INVALID_STATE` | The machine is in the wrong state for the requested action, such as an unsafe vise pose. |

## State Model

The upstream robot should treat the machine as a single-command state machine with the following key fields:

- `busy`: a mutating command is currently active
- `homed`: machine zero is valid and absolute coordinates can be trusted
- `faulted`: the machine has entered a recoverable or blocking error state
- `blade_on`: the ultrasonic blade relay output is energized
- `vise_state`: interpreted vise state, expected to be one of `OPEN`, `MOVING`, `CLOSED`, or `UNKNOWN`

Only one active mutating command is allowed at a time. Query commands may be serviced immediately, but they should report the current `busy` state truthfully.
