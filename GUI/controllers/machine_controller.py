"""X/Z stage + rotary + vise + cutting-machine control via ArduinoClient.

The Arduino serializes commands and a ``cut``/vise op can block for minutes, so a
background poll thread refreshes a status cache only while no mutating command is
active. The dashboard reads the cache (never the serial port directly).
"""

from __future__ import annotations

import threading
import time

import config

from helper.arduino_client import ArduinoClient


class MachineController:
    def __init__(self, logger, executor) -> None:
        self._logger = logger
        self._executor = executor
        self._client: ArduinoClient | None = None
        self._cache: dict = {}
        self._active_cmd: str | None = None
        self._cache_lock = threading.Lock()
        self._poll_stop = threading.Event()
        self._poll_thread: threading.Thread | None = None

    # ----- lifecycle -----

    def connect(self) -> None:
        self._client = ArduinoClient(
            port=config.ARDUINO_PORT,
            baud=config.ARDUINO_BAUD,
            logger=self._logger,
            ready_timeout_s=config.ARDUINO_READY_TIMEOUT_S,
        )
        self._client.connect()
        try:
            with self._cache_lock:
                self._cache = self._client.get_status()
        except Exception as exc:  # noqa: BLE001 - initial status is best-effort
            self._logger.warn(f"initial machine status failed: {exc}")
        self._poll_thread = threading.Thread(
            target=self._poll_loop, name="machine-poll", daemon=True
        )
        self._poll_thread.start()

    def close(self) -> None:
        self._poll_stop.set()
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=2.0)
        if self._client is not None:
            self._client.close()

    # ----- status cache -----

    def _poll_loop(self) -> None:
        while not self._poll_stop.is_set():
            if self._active_cmd is None:
                try:
                    status = self._client.get_status()
                    with self._cache_lock:
                        self._cache = status
                except Exception as exc:  # noqa: BLE001 - keep last good cache
                    self._logger.debug(f"machine poll error: {exc}")
            self._poll_stop.wait(config.MACHINE_POLL_INTERVAL_S)

    def status(self) -> dict:
        with self._cache_lock:
            cache = dict(self._cache)
        cache["active_command"] = self._active_cmd
        cache["connected"] = self._client is not None
        return cache

    # ----- control ops (via executor) -----

    def _machine_op(self, name: str, fn):
        def op():
            self._active_cmd = name
            try:
                result = fn(self._client)
                if isinstance(result, dict):
                    with self._cache_lock:
                        self._cache.update(result)
                return result
            finally:
                self._active_cmd = None
        return self._executor.submit(name, op)

    def home(self) -> bool:
        return self._machine_op("machine.home_all", lambda c: c.home_all())

    def move_x(self, x_mm: float, feed=None) -> bool:
        return self._machine_op("machine.move_x", lambda c: c.move_x_abs(x_mm, feed))

    def move_z(self, z_mm: float, feed=None) -> bool:
        return self._machine_op("machine.move_z", lambda c: c.move_z_abs(z_mm, feed))

    def rotate(self, deg: float, speed=None) -> bool:
        return self._machine_op("machine.rotate", lambda c: c.rotate_abs(deg, speed))

    def vise(self, action: str, force_kg: float | None = None) -> bool:
        if action == "close":
            f = 4.0 if force_kg is None else force_kg
            return self._machine_op("machine.close_vise", lambda c: c.close_vise(f))
        f = 0.2 if force_kg is None else force_kg
        return self._machine_op("machine.open_vise", lambda c: c.open_vise(f))

    def blade(self, on: bool) -> bool:
        return self._machine_op(f"machine.blade:{'on' if on else 'off'}",
                                lambda c: c.set_blade(on))

    def cut(self, z_mm: float, x_mm: float, deg: float) -> bool:
        return self._machine_op("machine.cut_height",
                                lambda c: c.cut_height(z_mm, x_mm, deg))

    def clear_faults(self) -> dict:
        result = self._client.clear_faults()
        return result

    def stop(self) -> dict:
        # Best-effort immediate stop; bypasses the executor on purpose.
        return self._client.stop_all()
