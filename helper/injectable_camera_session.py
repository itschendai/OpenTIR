"""Shared RealSense RGB-D session for injectable detection + POV recording."""

from __future__ import annotations

from contextlib import contextmanager
import os
import subprocess
import sys
import threading
import time
import weakref
from pathlib import Path
from typing import Any

if __name__ == "helper.injectable_camera_session":
    sys.modules.setdefault("injectable_camera_session", sys.modules[__name__])
elif __name__ == "injectable_camera_session":
    sys.modules.setdefault("helper.injectable_camera_session", sys.modules[__name__])

_ACTIVE_SESSION_LOCK = threading.Lock()
_ACTIVE_SHARED_CAMERA_SESSIONS: dict[str, weakref.ReferenceType["SharedInjectableCameraSession"]] = {}


def _lookup_shared_camera_session(serial: str | None):
    if not serial:
        return None
    with _ACTIVE_SESSION_LOCK:
        ref = _ACTIVE_SHARED_CAMERA_SESSIONS.get(str(serial))
    if ref is None:
        return None
    session = ref()
    if session is None:
        with _ACTIVE_SESSION_LOCK:
            _ACTIVE_SHARED_CAMERA_SESSIONS.pop(str(serial), None)
    return session


@contextmanager
def temporarily_release_shared_camera(serial: str | None):
    session = _lookup_shared_camera_session(serial)
    released = False
    if session is not None:
        session._info("Temporarily releasing shared RealSense session for external capture")
        released = session.pause()
    try:
        yield
    finally:
        if session is not None and released:
            session._info("Resuming shared RealSense session after external capture")
            session.resume()


class SharedInjectableCameraSession:
    """Own one RealSense pipeline and expose the latest aligned RGB-D frame.

    A background thread continuously reads frames, aligns depth to color, caches
    the latest RGB-D snapshot, and optionally writes the color stream to video.
    This lets a recipe record POV footage and run detection from the same camera
    owner instead of racing two independent pipelines.
    """

    def __init__(
        self,
        *,
        detector_module,
        serial: str | None = None,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        warmup_frames: int = 30,
        exposure: float | None = None,
        gain: float | None = None,
        white_balance: float | None = None,
        logger=None,
        record_path: str | None = None,
    ) -> None:
        self._detector = detector_module
        self._serial = serial
        self._width = int(width)
        self._height = int(height)
        self._fps = int(fps)
        self._warmup_frames = int(warmup_frames)
        self._exposure = exposure
        self._gain = gain
        self._white_balance = white_balance
        self._logger = logger
        self._record_path = record_path

        self._pipeline = None
        self._align = None
        self._profile = None
        self._rs = None
        self._depth_scale = None
        self._camera = None
        self._writer = None
        self._thread = None
        self._stop_event = threading.Event()
        self._ready_event = threading.Event()
        self._lock = threading.Lock()
        self._latest_frame = None
        self._last_error = None
        self._closed = False

    def _info(self, msg: str) -> None:
        if self._logger is not None:
            self._logger.info(msg)

    def _warn(self, msg: str) -> None:
        if self._logger is not None:
            self._logger.warn(msg)

    def _register_active_session(self) -> None:
        if not self._serial:
            return
        with _ACTIVE_SESSION_LOCK:
            _ACTIVE_SHARED_CAMERA_SESSIONS[str(self._serial)] = weakref.ref(self)

    def _unregister_active_session(self) -> None:
        if not self._serial:
            return
        with _ACTIVE_SESSION_LOCK:
            existing = _ACTIVE_SHARED_CAMERA_SESSIONS.get(str(self._serial))
            if existing is None:
                return
            target = existing()
            if target is None or target is self:
                _ACTIVE_SHARED_CAMERA_SESSIONS.pop(str(self._serial), None)

    def _video_device_holders(self) -> str:
        try:
            result = subprocess.run(
                ["bash", "-lc", "lsof /dev/video* 2>/dev/null || true"],
                check=False,
                capture_output=True,
                text=True,
            )
        except Exception:
            return ""
        output = (result.stdout or "").strip()
        if not output:
            return ""

        current_pid = str(os.getpid())
        filtered_lines = []
        for line in output.splitlines():
            parts = line.split()
            if len(parts) > 1 and parts[1] == current_pid:
                continue
            filtered_lines.append(line)
        if not filtered_lines or filtered_lines == ["COMMAND    PID USER   FD   TYPE DEVICE SIZE/OFF NODE NAME"]:
            return ""
        return "\n".join(filtered_lines)

    def _camera_runtime_metadata(self) -> dict[str, str]:
        if self._camera is None or self._rs is None:
            return {}
        device_handle = self._camera.get("device")
        if device_handle is None:
            serial = self._camera.get("serial")
            if serial:
                for device in self._rs.context().query_devices():
                    try:
                        if device.get_info(self._rs.camera_info.serial_number) == serial:
                            device_handle = device
                            self._camera["device"] = device
                            break
                    except RuntimeError:
                        continue
        if device_handle is None:
            return {}
        info_keys = {
            "usb_type": "usb_type_descriptor",
            "physical_port": "physical_port",
        }
        metadata: dict[str, str] = {}
        for out_key, info_name in info_keys.items():
            info = getattr(self._rs.camera_info, info_name, None)
            if info is None:
                continue
            try:
                metadata[out_key] = device_handle.get_info(info)
            except RuntimeError:
                continue
        return metadata

    def _format_frame_timeout(self, exc: Exception, *, phase: str) -> str:
        metadata = self._camera_runtime_metadata()
        lines = [
            f"Camera frame timeout during {phase}: {exc}",
        ]
        if self._camera is not None:
            lines.append(
                "Camera details: {name} | serial: {serial} | firmware: {firmware}".format(
                    **self._camera
                )
            )
        usb_type = metadata.get("usb_type")
        if usb_type:
            lines.append(f"USB descriptor: {usb_type}")
            if usb_type.startswith("2."):
                lines.append(
                    "Warning: camera is enumerated as USB 2.x. RealSense RGB-D streaming may time out "
                    "unless the camera is connected through a SuperSpeed USB 3 port/cable/hub."
                )
        physical_port = metadata.get("physical_port")
        if physical_port:
            lines.append(f"Physical port: {physical_port}")
        holders = self._video_device_holders()
        if holders:
            lines.append("Current /dev/video holders:")
            lines.append(holders)
        else:
            lines.append("No current /dev/video holder process was detected.")
        return "\n".join(lines)

    def _ensure_camera_context(self):
        try:
            import pyrealsense2 as rs  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "Missing dependency: pyrealsense2. Install it with: pip install pyrealsense2"
            ) from exc
        self._rs = rs

        devices = self._detector._realsense_devices(rs)
        if not devices:
            raise RuntimeError("No Intel RealSense camera detected.")

        camera = self._detector._select_camera(devices, self._serial)
        if camera is None:
            raise RuntimeError(
                f"No RealSense camera found with serial number: {self._serial}"
            )
        self._camera = camera
        self._serial = camera["serial"]
        return rs, camera

    def _open_pipeline(self) -> None:
        rs, camera = self._ensure_camera_context()

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(camera["serial"])
        config.enable_stream(
            rs.stream.color,
            self._width,
            self._height,
            rs.format.bgr8,
            self._fps,
        )
        config.enable_stream(
            rs.stream.depth,
            self._width,
            self._height,
            rs.format.z16,
            self._fps,
        )
        try:
            profile = pipeline.start(config)
        except RuntimeError as exc:
            holders = self._video_device_holders()
            holder_suffix = f"\nCurrent /dev/video holders:\n{holders}" if holders else ""
            raise RuntimeError(
                f"Could not start camera stream: {exc}. "
                f"Try width={self._width} height={self._height} fps={self._fps}"
                f"{holder_suffix}"
            ) from exc

        self._detector._configure_color_sensor(
            rs,
            profile,
            exposure=self._exposure,
            gain=self._gain,
            white_balance=self._white_balance,
        )
        align = rs.align(rs.stream.color)

        self._info(
            "Shared camera session: "
            f"{camera['name']} serial={camera['serial']} firmware={camera['firmware']}"
        )
        usb_type = self._camera_runtime_metadata().get("usb_type")
        if usb_type and usb_type.startswith("2."):
            self._warn(
                "Shared camera session warning: camera is enumerated as USB 2.x; "
                "RealSense RGB-D frames may time out until it is moved to a SuperSpeed USB 3 connection."
            )

        try:
            for _ in range(max(self._warmup_frames, 1)):
                frames = pipeline.wait_for_frames(timeout_ms=5000)
                aligned = align.process(frames)
                color_frame = aligned.get_color_frame()
                depth_frame = aligned.get_depth_frame()
                if color_frame and depth_frame:
                    self._store_frame(color_frame, depth_frame, profile)
        except RuntimeError as exc:
            try:
                pipeline.stop()
            except Exception:
                pass
            raise RuntimeError(self._format_frame_timeout(exc, phase="startup warmup")) from exc

        self._pipeline = pipeline
        self._align = align
        self._profile = profile
        if self._writer is None:
            self._start_writer()
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._reader_loop,
            name="injectable-camera-session",
            daemon=True,
        )
        self._thread.start()
        self._ready_event.set()
        self._register_active_session()

    def start(self) -> None:
        if self._thread is not None or self._pipeline is not None:
            return
        self._closed = False
        self._last_error = None
        self._open_pipeline()

    def _start_writer(self) -> None:
        if not self._record_path:
            return
        import cv2

        record_path = Path(self._record_path)
        record_path.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(
            str(record_path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            float(self._fps),
            (self._width, self._height),
        )
        if not writer.isOpened():
            raise RuntimeError(f"Could not open POV video writer: {record_path}")
        self._writer = writer
        self._info(f"POV recording -> {record_path}")

    def _store_frame(self, color_frame, depth_frame, profile) -> None:
        import numpy as np

        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = float(depth_sensor.get_depth_scale())
        intrinsics = color_frame.profile.as_video_stream_profile().get_intrinsics()
        color_image = np.asanyarray(color_frame.get_data()).copy()
        depth_image = np.asanyarray(depth_frame.get_data()).copy()
        frame_data = {
            "timestamp": time.time(),
            "color_image": color_image,
            "depth_image": depth_image,
            "intrinsics": intrinsics,
            "depth_scale": depth_scale,
        }
        with self._lock:
            self._latest_frame = frame_data
            self._depth_scale = depth_scale
        if self._writer is not None:
            self._writer.write(color_image)

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=5000)
                aligned = self._align.process(frames)
                color_frame = aligned.get_color_frame()
                depth_frame = aligned.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue
                self._store_frame(color_frame, depth_frame, self._profile)
            except Exception as exc:  # noqa: BLE001 - background reader reports via state
                if isinstance(exc, RuntimeError) and "Frame didn't arrive" in str(exc):
                    msg = self._format_frame_timeout(exc, phase="background capture")
                    wrapped = RuntimeError(msg)
                    self._last_error = wrapped
                    self._warn(msg)
                else:
                    self._last_error = exc
                    self._warn(f"Shared camera session reader error: {exc}")
                break

    def get_latest_aligned_rgbd(
        self,
        timeout_s: float = 5.0,
        *,
        min_timestamp: float | None = None,
    ):
        if self._pipeline is None and self._thread is None and not self._closed:
            self.resume()
        deadline = time.monotonic() + max(0.0, timeout_s)
        while True:
            if self._last_error is not None:
                raise RuntimeError(
                    f"Shared camera session is unavailable: {self._last_error}"
                ) from self._last_error
            with self._lock:
                frame = self._latest_frame
            if frame is not None and (
                min_timestamp is None or float(frame["timestamp"]) >= float(min_timestamp)
            ):
                return (
                    frame["color_image"].copy(),
                    frame["depth_image"].copy(),
                    frame["intrinsics"],
                    float(frame["depth_scale"]),
                )
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Timed out waiting for shared camera frame within {timeout_s:.1f}s"
                )
            time.sleep(0.05)

    def pause(self) -> bool:
        if self._pipeline is None and self._thread is None:
            return False
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
        self._pipeline = None
        self._align = None
        self._profile = None
        self._ready_event.clear()
        return True

    def resume(self) -> None:
        if self._closed:
            raise RuntimeError("Shared camera session is closed")
        if self._pipeline is not None or self._thread is not None:
            return
        self._last_error = None
        self._open_pipeline()

    def close(self) -> None:
        self._unregister_active_session()
        self._closed = True
        self.pause()
        if self._writer is not None:
            try:
                self._writer.release()
            except Exception:
                pass
            self._writer = None
        self._rs = None
