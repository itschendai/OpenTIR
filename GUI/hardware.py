"""Hardware ownership + the single-operation executor for the GUI backend.

One process owns the robot, gripper, camera, and Arduino for its whole lifetime.
The robot can only perform one motion at a time and ``ArduinoClient`` serializes
its commands, so every *mutating* operation (robot move, gripper, floating,
recipe run/step, machine move, vise, cut) is funnelled through a single
``OperationExecutor`` worker. Status reads run outside it so the dashboard stays
live while a long operation is in flight.
"""

from __future__ import annotations

import collections
import sys
import threading
import time
from pathlib import Path

import config


# ----- logging --------------------------------------------------------------

class BufferLogger:
    """spdlog-shaped logger (info/warn/debug) that also keeps a ring buffer.

    The ring buffer backs the recipe log panel; ``records()`` returns the tail.
    """

    def __init__(self, maxlen: int = 500) -> None:
        self._buf: collections.deque[tuple[float, str, str]] = collections.deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._seq = 0
        self._observers: list = []

    def add_observer(self, fn) -> None:
        """Register ``fn(level, msg)`` called on every emitted record.

        Used by the recipe runner to follow ``=== phase_* ===`` markers and
        advance the live phase highlight without disturbing the recipe flow.
        """
        self._observers.append(fn)

    def _emit(self, level: str, msg: str) -> None:
        text = str(msg)
        with self._lock:
            self._seq += 1
            self._buf.append((self._seq, level, text))
        for fn in self._observers:
            try:
                fn(level, text)
            except Exception:  # noqa: BLE001 - an observer must never break logging
                pass
        print(f"[{level}] {msg}", flush=True)

    def info(self, msg: str) -> None:
        self._emit("info", msg)

    def warn(self, msg: str) -> None:
        self._emit("warn", msg)

    def warning(self, msg: str) -> None:
        self._emit("warn", msg)

    def debug(self, msg: str) -> None:
        self._emit("debug", msg)

    def records(self, after_seq: int = 0) -> list[dict]:
        with self._lock:
            return [
                {"seq": seq, "level": level, "message": msg}
                for (seq, level, msg) in self._buf
                if seq > after_seq
            ]


# ----- single-operation executor -------------------------------------------

class OperationExecutor:
    """Run at most one mutating operation at a time.

    ``submit`` returns immediately: ``True`` if the operation was accepted and
    is now running on the worker thread, ``False`` if another operation is busy.
    """

    def __init__(self, logger: BufferLogger) -> None:
        self._logger = logger
        self._lock = threading.Lock()
        self._busy_name: str | None = None
        self._last_result = None
        self._last_error: str | None = None
        self._started_at: float | None = None
        self.stop_event = threading.Event()

    def submit(self, name: str, fn) -> bool:
        if not self._lock.acquire(blocking=False):
            return False
        self._busy_name = name
        self._started_at = time.time()
        self._last_error = None
        self.stop_event.clear()
        thread = threading.Thread(
            target=self._run, args=(name, fn), name=f"op-{name}", daemon=True
        )
        thread.start()
        return True

    def _run(self, name: str, fn) -> None:
        try:
            self._last_result = fn()
            self._logger.info(f"operation '{name}' finished")
        except Exception as exc:  # noqa: BLE001 - surfaced to the UI
            self._last_error = f"{type(exc).__name__}: {exc}"
            self._logger.warn(f"operation '{name}' failed: {self._last_error}")
        finally:
            self._busy_name = None
            self._started_at = None
            self._lock.release()

    @property
    def busy(self) -> bool:
        return self._busy_name is not None

    def request_stop(self) -> None:
        """Signal the running operation (e.g. a recipe loop) to stop early."""
        self.stop_event.set()

    def snapshot(self) -> dict:
        return {
            "busy": self.busy,
            "operation": self._busy_name,
            "started_at": self._started_at,
            "last_error": self._last_error,
        }


# ----- hardware hub ---------------------------------------------------------

class HardwareHub:
    """Builds and owns every hardware handle plus the shared executor."""

    def __init__(self) -> None:
        # Make project helpers / camera / recipe modules importable.
        project = str(config.PROJECT_DIR)
        if project not in sys.path:
            sys.path.insert(0, project)

        self.logger = BufferLogger()
        self.executor = OperationExecutor(self.logger)
        self.robot = None          # RobotController
        self.machine = None        # MachineController
        self.camera = None         # CameraBroker
        self.recipes = None        # RecipeRunner
        self._closables: list = []

    def start(self) -> None:
        # Imported here so an import error names the failing subsystem clearly.
        from controllers.robot_controller import RobotController
        from controllers.machine_controller import MachineController
        from controllers.camera_broker import CameraBroker
        from controllers.recipe_runner import RecipeRunner

        self.logger.info("Starting hardware: robot ...")
        self.robot = RobotController(self.logger, self.executor)
        self.robot.connect()
        self._closables.append(self.robot)

        self.logger.info("Starting hardware: Arduino cutting machine ...")
        self.machine = MachineController(self.logger, self.executor)
        self.machine.connect()
        self._closables.append(self.machine)

        self.logger.info("Starting hardware: RealSense camera ...")
        self.camera = CameraBroker(self.logger)
        self.camera.start()
        self._closables.append(self.camera)

        self.recipes = RecipeRunner(
            self.logger, self.executor, self.robot, self.machine, self.camera
        )
        self.logger.info("Hardware ready.")

    def close(self) -> None:
        for obj in reversed(self._closables):
            try:
                obj.close()
            except Exception as exc:  # noqa: BLE001 - best-effort shutdown
                self.logger.warn(f"shutdown of {obj!r} raised: {exc}")
