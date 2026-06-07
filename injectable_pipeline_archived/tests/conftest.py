"""Shared pytest fixtures for the injectable_pipeline test suite."""

from __future__ import annotations

import os
import sys
import threading
import time
from collections import deque

import pytest


# Make the package directory importable without installation.
HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(HERE)
if PACKAGE_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_DIR)


class FakeSerial:
    """Minimal in-memory serial stand-in.

    The producer side (the test) calls ``feed_response(...)`` with the lines the
    Arduino would emit in response to whichever command the client wrote last.
    The consumer side (ArduinoClient) sees bytes written via ``write`` and reads
    them via ``readline`` with a short timeout, mirroring pyserial semantics.
    """

    def __init__(self, *, read_timeout: float = 0.1, auto_responder=None) -> None:
        self.read_timeout = read_timeout
        self._tx_buffer = bytearray()
        self._tx_lines: list[str] = []
        self._rx_queue: deque[bytes] = deque()
        # RLock so an auto_responder triggered inside ``write`` can call
        # feed_response() (which also locks) without deadlocking.
        self._lock = threading.RLock()
        self._auto_responder = auto_responder
        self.closed = False

    # -- writer side (consumed by tests) --
    @property
    def writes(self) -> list[str]:
        return list(self._tx_lines)

    def write(self, data) -> int:
        if isinstance(data, str):
            data = data.encode("ascii")
        with self._lock:
            self._tx_buffer.extend(data)
            while b"\n" in self._tx_buffer:
                idx = self._tx_buffer.index(b"\n")
                line = bytes(self._tx_buffer[: idx + 1]).decode("ascii", errors="replace")
                del self._tx_buffer[: idx + 1]
                self._tx_lines.append(line.rstrip("\r\n"))
                if self._auto_responder is not None:
                    self._auto_responder(self, line.rstrip("\r\n"))
        return len(data)

    def flush(self) -> None:
        pass

    # -- reader side (consumed by client) --
    def feed_response(self, line: str) -> None:
        if not line.endswith("\n"):
            line = line + "\n"
        with self._lock:
            self._rx_queue.append(line.encode("ascii"))

    def readline(self):
        # Mirror pyserial's ``Serial(timeout=...).readline()`` behavior: block up to
        # ``read_timeout`` seconds, return the next pending line if any, else b"".
        deadline = time.monotonic() + self.read_timeout
        while True:
            with self._lock:
                if self._rx_queue:
                    return self._rx_queue.popleft()
            if time.monotonic() >= deadline:
                return b""
            time.sleep(0.005)

    def close(self) -> None:
        self.closed = True


@pytest.fixture
def fake_serial_factory():
    """Return a factory that builds a FakeSerial and lets the test drive it."""

    fakes: list[FakeSerial] = []

    def factory(auto_responder=None, read_timeout=0.02):
        fake = FakeSerial(read_timeout=read_timeout, auto_responder=auto_responder)
        fakes.append(fake)
        return fake

    yield factory
    for fake in fakes:
        fake.close()
