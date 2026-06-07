"""Serial client for the cutting-machine Arduino primitive API.

Implements the contract in ``planning/specifications.md`` §2 and the wire format in
``../../XZ Stage Code v2/PRIMITIVES.md``. One mutating command at a time; queries
return immediately. Responses are line-based ASCII at 115200 baud.
"""

from __future__ import annotations

import argparse
import glob
import io
import os
import re
import threading
import time
from typing import Iterable

try:
    import serial as pyserial  # pyserial
except ImportError:  # pragma: no cover - pyserial is a declared dep
    pyserial = None


SERIAL_TIMEOUT_S = 0.1  # short readline timeout so we can poll for overall deadline
DEFAULT_BAUD = 115200
DEFAULT_DONE_TIMEOUT_S = 30.0
# Opening the serial port DTR-resets the Arduino; the firmware then emits a READY
# banner. Real-hardware callers should pass ready_timeout_s>0 so connect() blocks
# until the banner is seen, otherwise the first command races the reboot and is lost.
DEFAULT_READY_TIMEOUT_S = 5.0

_QUERY_COMMANDS = {"GET_STATUS", "GET_FORCE"}

_RESP_RE = re.compile(r"^(ACK|DONE|ERR)\s+(\d+)\s+([A-Z_]+)(.*)$")
_KV_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=(\"[^\"]*\"|\S+)")


class ArduinoError(Exception):
    pass


class ArduinoTimeoutError(ArduinoError):
    """No DONE/ERR received within done_timeout_s for a command."""


class ArduinoCommandError(ArduinoError):
    """Arduino returned ERR for a command."""

    def __init__(self, code: str, message: str, cmd_id: int, command: str) -> None:
        super().__init__(f"{command}#{cmd_id}: {code}: {message}")
        self.code = code
        self.message = message
        self.cmd_id = cmd_id
        self.command = command


def _parse_value(raw: str):
    """Parse a single ``key=value`` value into a Python type."""
    if len(raw) >= 2 and raw.startswith('"') and raw.endswith('"'):
        return raw[1:-1]
    low = raw.lower()
    if low == "true":
        return True
    if low == "false":
        return False
    try:
        return int(raw)
    except ValueError:
        pass
    try:
        return float(raw)
    except ValueError:
        pass
    return raw


def parse_kv_tail(tail: str) -> dict:
    """Parse the trailing ``key=value [key=value ...]`` section of a response."""
    result: dict = {}
    for match in _KV_RE.finditer(tail):
        key = match.group(1)
        result[key] = _parse_value(match.group(2))
    return result


def parse_response_line(line: str):
    """Parse one ACK/DONE/ERR line.

    Returns ``(kind, cmd_id, command, fields)`` where ``fields`` is a dict.
    Returns ``None`` for lines that are not a recognized response.
    """
    match = _RESP_RE.match(line.strip())
    if match is None:
        return None
    kind, cmd_id, command, tail = match.group(1), int(match.group(2)), match.group(3), match.group(4)
    return kind, cmd_id, command, parse_kv_tail(tail)


def parse_ack_line(line: str):
    parsed = parse_response_line(line)
    if parsed is None or parsed[0] != "ACK":
        return None
    return parsed[1], parsed[2]


def parse_done_line(line: str):
    parsed = parse_response_line(line)
    if parsed is None or parsed[0] != "DONE":
        return None
    _, cmd_id, command, fields = parsed
    return cmd_id, command, fields


def parse_err_line(line: str):
    parsed = parse_response_line(line)
    if parsed is None or parsed[0] != "ERR":
        return None
    _, cmd_id, command, fields = parsed
    code = fields.get("code", "")
    message = fields.get("message", "")
    return cmd_id, command, str(code), str(message)


def _format_value(value) -> str:
    if isinstance(value, bool):
        return "ON" if value else "OFF"
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)


def format_command(command: str, cmd_id: int, args: dict | None = None) -> str:
    """Build the wire string for a command, terminated with a newline."""
    parts = [command, f"cmd_id={cmd_id}"]
    if args:
        for key, value in args.items():
            if value is None:
                continue
            parts.append(f"{key}={_format_value(value)}")
    return " ".join(parts) + "\n"


def autodetect_port() -> str | None:
    """Best-effort port discovery: prefer /dev/serial/by-id/*Arduino*, else /dev/ttyACM*."""
    by_id_candidates = sorted(glob.glob("/dev/serial/by-id/*Arduino*"))
    if by_id_candidates:
        return by_id_candidates[0]
    by_id_candidates = sorted(glob.glob("/dev/serial/by-id/*USB*"))
    if by_id_candidates:
        return by_id_candidates[0]
    acm = sorted(glob.glob("/dev/ttyACM*"))
    if acm:
        return acm[0]
    usb = sorted(glob.glob("/dev/ttyUSB*"))
    if usb:
        return usb[0]
    return None


class ArduinoClient:
    """Line-based serial client for the cutting machine."""

    def __init__(
        self,
        port: str | None = None,
        baud: int = DEFAULT_BAUD,
        done_timeout_s: float = DEFAULT_DONE_TIMEOUT_S,
        logger=None,
        serial_factory=None,
        ready_timeout_s: float = 0.0,
    ) -> None:
        self._port = port
        self._baud = baud
        self._done_timeout_s = done_timeout_s
        self._ready_timeout_s = ready_timeout_s
        self._logger = logger
        self._serial_factory = serial_factory
        self._serial = None
        self._cmd_id = 0
        self._lock = threading.Lock()
        self._stray_lines: list[str] = []

    # ----- lifecycle -----

    def connect(self) -> None:
        if self._serial is not None:
            return
        if self._serial_factory is not None:
            self._serial = self._serial_factory()
        else:
            if pyserial is None:
                raise ArduinoError(
                    "pyserial is not installed; cannot open a real serial port"
                )
            port = self._port or autodetect_port()
            if port is None:
                raise ArduinoError(
                    "No Arduino serial port found (looked under /dev/serial/by-id/ "
                    "and /dev/ttyACM*); pass --arduino-port"
                )
            self._serial = pyserial.Serial(port=port, baudrate=self._baud, timeout=SERIAL_TIMEOUT_S)
            self._port = port
        self._info(f"Arduino serial open: {self._port} @ {self._baud}")
        if self._ready_timeout_s > 0:
            self._wait_for_ready(self._ready_timeout_s)

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    # ----- public mutating primitives -----

    def home_all(self) -> dict:
        return self._send("HOME_ALL", {})

    def move_x_abs(self, x_mm: float, feed_mm_s: float | None = None) -> dict:
        args = {"x_mm": float(x_mm)}
        if feed_mm_s is not None:
            args["feed"] = float(feed_mm_s)
        return self._send("MOVE_X_ABS", args)

    def move_z_abs(self, z_mm: float, feed_mm_s: float | None = None) -> dict:
        args = {"z_mm": float(z_mm)}
        if feed_mm_s is not None:
            args["feed"] = float(feed_mm_s)
        return self._send("MOVE_Z_ABS", args)

    def rotate_abs(self, deg: float, speed: float | None = None) -> dict:
        args = {"deg": float(deg)}
        if speed is not None:
            args["speed"] = float(speed)
        return self._send("ROTATE_ABS", args)

    def move_rel(self, axis: str, delta: float, feed: float | None = None) -> dict:
        axis_u = str(axis).upper()
        if axis_u not in {"X", "Z", "ROT"}:
            raise ValueError(f"axis must be X, Z, or ROT (got {axis!r})")
        args = {"axis": axis_u, "delta": float(delta)}
        if feed is not None:
            args["feed"] = float(feed)
        return self._send("MOVE_REL", args)

    def set_blade(self, on: bool) -> dict:
        return self._send("SET_BLADE", {"state": bool(on)})

    def close_vise(self, target_force_kg: float = 4.0) -> dict:
        return self._send(
            "CLOSE_VISE",
            {"target_force_kg": float(target_force_kg)},
            override_timeout_s=max(self._done_timeout_s, 5 * 60.0 + 5.0),
        )

    def open_vise(self, target_force_kg: float = 0.2) -> dict:
        return self._send(
            "OPEN_VISE",
            {"target_force_kg": float(target_force_kg)},
            override_timeout_s=max(self._done_timeout_s, 5 * 60.0 + 5.0),
        )

    def cut_height(self, z_mm: float, x_mm: float, deg: float) -> dict:
        return self._send(
            "CUT_HEIGHT",
            {"z_mm": float(z_mm), "x_mm": float(x_mm), "deg": float(deg)},
            override_timeout_s=max(self._done_timeout_s, 5 * 60.0 + 5.0),
        )

    def stop_all(self) -> dict:
        return self._send("STOP_ALL", {})

    def clear_faults(self) -> dict:
        return self._send("CLEAR_FAULTS", {})

    # ----- queries -----

    def get_status(self) -> dict:
        return self._send("GET_STATUS", {})

    def get_force(self) -> dict:
        return self._send("GET_FORCE", {})

    # ----- diagnostics -----

    def drain_stray_lines(self) -> list[str]:
        out = list(self._stray_lines)
        self._stray_lines.clear()
        return out

    @property
    def port(self) -> str | None:
        return self._port

    # ----- internals -----

    def _send(self, command: str, args: dict, override_timeout_s: float | None = None) -> dict:
        if self._serial is None:
            raise ArduinoError("ArduinoClient is not connected; call connect() first")

        timeout = override_timeout_s if override_timeout_s is not None else self._done_timeout_s

        with self._lock:
            self._cmd_id += 1
            cmd_id = self._cmd_id
            wire = format_command(command, cmd_id, args)
            self._debug(f"-> {wire.rstrip()}")
            self._write_line(wire)

            try:
                fields = self._read_completion(cmd_id, command, timeout)
            except ArduinoTimeoutError:
                self._info(f"{command} #{cmd_id} timed out; sending STOP_ALL")
                self._best_effort_stop_all()
                raise

        fields["command"] = command
        fields["cmd_id"] = cmd_id
        return fields

    def _write_line(self, wire: str) -> None:
        data = wire.encode("ascii", errors="replace")
        self._serial.write(data)
        flush = getattr(self._serial, "flush", None)
        if callable(flush):
            try:
                flush()
            except Exception:
                pass

    def _read_completion(self, cmd_id: int, command: str, timeout_s: float) -> dict:
        deadline = time.monotonic() + max(0.0, timeout_s)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ArduinoTimeoutError(
                    f"Timed out after {timeout_s:.3f}s waiting for DONE/ERR "
                    f"for {command} cmd_id={cmd_id}"
                )
            line = self._read_line(remaining)
            if line is None:
                continue
            self._debug(f"<- {line.rstrip()}")
            parsed = parse_response_line(line)
            if parsed is None:
                if line.strip():
                    self._stray_lines.append(line.rstrip("\r\n"))
                continue

            kind, resp_id, resp_cmd, fields = parsed
            if resp_id != cmd_id:
                # Response for a different command (shouldn't happen with monotonic ids,
                # but tolerate it). Stash for diagnostics and keep waiting.
                self._stray_lines.append(line.rstrip("\r\n"))
                continue
            if kind == "ACK":
                # Consume and ignore.
                continue
            if kind == "ERR":
                code = str(fields.get("code", ""))
                message = str(fields.get("message", ""))
                raise ArduinoCommandError(code, message, cmd_id, command)
            if kind == "DONE":
                return fields

    def _read_line(self, remaining_timeout_s: float) -> str | None:
        readline = self._serial.readline
        line = readline()
        if not line:
            return None
        if isinstance(line, (bytes, bytearray)):
            try:
                return line.decode("ascii", errors="replace")
            except Exception:
                return line.decode("latin-1", errors="replace")
        return str(line)

    def _wait_for_ready(self, timeout_s: float) -> bool:
        """Block until a line starting with ``READY`` is seen or timeout expires.

        Opening the serial port DTR-resets the Arduino; sending any command before
        the firmware finishes booting drops it. Call this after open to swallow
        the banner. Returns True if READY was observed, False on timeout (firmware
        may already be running — proceed anyway).
        """
        deadline = time.monotonic() + max(0.0, timeout_s)
        while time.monotonic() < deadline:
            line = self._read_line(deadline - time.monotonic())
            if line is None:
                continue
            self._debug(f"<- {line.rstrip()}")
            if line.lstrip().startswith("READY"):
                return True
            if line.strip():
                self._stray_lines.append(line.rstrip("\r\n"))
        self._info(f"READY banner not seen within {timeout_s:.1f}s; proceeding")
        return False

    def _best_effort_stop_all(self) -> None:
        try:
            self._cmd_id += 1
            stop_id = self._cmd_id
            wire = format_command("STOP_ALL", stop_id, {})
            self._debug(f"-> {wire.rstrip()} (best-effort)")
            self._write_line(wire)
        except Exception as exc:  # noqa: BLE001 - best effort
            self._info(f"STOP_ALL emit failed: {exc}")

    def _info(self, msg: str) -> None:
        if self._logger is not None and hasattr(self._logger, "info"):
            self._logger.info(msg)

    def _debug(self, msg: str) -> None:
        if self._logger is not None and hasattr(self._logger, "debug"):
            self._logger.debug(msg)


# --- standalone smoke entrypoint (M1 acceptance) -------------------------------------

_VERB_HANDLERS = {
    "home": lambda c, _a: c.home_all(),
    "status": lambda c, _a: c.get_status(),
    "force": lambda c, _a: c.get_force(),
    "clear": lambda c, _a: c.clear_faults(),
    "stop": lambda c, _a: c.stop_all(),
}


def _smoke_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="arduino_client",
        description="Smoke commands against the cutting-machine Arduino.",
    )
    parser.add_argument("verb", choices=sorted(_VERB_HANDLERS), help="Command to send")
    parser.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--timeout", type=float, default=DEFAULT_DONE_TIMEOUT_S)
    parser.add_argument(
        "--ready-timeout",
        type=float,
        default=DEFAULT_READY_TIMEOUT_S,
        help="Seconds to wait for the firmware's READY banner after opening the port",
    )
    args = parser.parse_args(argv)

    class _StdoutLogger:
        def info(self, msg):
            print(f"[info] {msg}")

        def debug(self, msg):
            print(f"[trace] {msg}")

    client = ArduinoClient(
        port=args.port,
        baud=args.baud,
        done_timeout_s=args.timeout,
        logger=_StdoutLogger(),
        ready_timeout_s=args.ready_timeout,
    )
    try:
        client.connect()
        result = _VERB_HANDLERS[args.verb](client, args)
    finally:
        client.close()
    print(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(_smoke_main())
