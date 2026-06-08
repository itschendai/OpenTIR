"""RealSense RGB frame broker for the live view.

Owns one ``SharedInjectableCameraSession`` (the same one recipes use) and turns
its latest aligned colour frame into JPEG / MJPEG. Depth is intentionally
ignored. ``overlay`` is the documented seam: today it returns the frame
unchanged (raw RGB); an injectable or tag detector can be slotted in later by
swapping this method without touching the streaming code.
"""

from __future__ import annotations

import importlib
import time

import cv2

import config

from helper.injectable_camera_session import SharedInjectableCameraSession


class CameraBroker:
    def __init__(self, logger) -> None:
        self._logger = logger
        # detect_injectable_static supplies the device-discovery helpers the
        # shared session calls (_realsense_devices/_select_camera/_configure_color_sensor).
        self._detector = importlib.import_module("camera.detect_injectable_static")
        self._session: SharedInjectableCameraSession | None = None
        self._encode_params = [cv2.IMWRITE_JPEG_QUALITY, config.CAMERA_JPEG_QUALITY]
        self._last_jpeg: bytes | None = None

    def start(self) -> None:
        self._session = SharedInjectableCameraSession(
            detector_module=self._detector,
            serial=config.CAMERA_SERIAL,
            width=config.CAMERA_WIDTH,
            height=config.CAMERA_HEIGHT,
            fps=config.CAMERA_FPS,
            warmup_frames=config.CAMERA_WARMUP_FRAMES,
            logger=self._logger,
        )
        self._session.start()

    def close(self) -> None:
        if self._session is not None:
            self._session.close()
            self._session = None

    def overlay(self, color_bgr):
        """Hook for future injectable/tag overlays. Raw passthrough for now."""
        return color_bgr

    def _latest_color(self):
        """Read the latest cached colour frame WITHOUT resuming the pipeline.

        ``get_latest_aligned_rgbd`` auto-resumes a paused session; that would let
        the live-view stream steal the (USB-2) camera back from a recipe that has
        temporarily released it for an external capture (e.g. tag calibration).
        We deliberately stay passive: read the cached frame, and if the session is
        paused or has none yet, the caller keeps serving the last good JPEG.
        """
        sess = self._session
        if sess is None:
            return None
        with sess._lock:
            frame = sess._latest_frame
        return None if frame is None else frame["color_image"]

    def get_jpeg(self) -> bytes | None:
        color = self._latest_color()
        if color is None:
            return self._last_jpeg
        ok, buf = cv2.imencode(".jpg", self.overlay(color), self._encode_params)
        if ok:
            self._last_jpeg = buf.tobytes()
        return self._last_jpeg

    def mjpeg_frames(self):
        boundary = b"--frame\r\n"
        interval = 1.0 / max(config.CAMERA_FPS, 1)
        while True:
            try:
                jpeg = self.get_jpeg()
            except Exception as exc:  # noqa: BLE001 - stream survives transient errors
                self._logger.debug(f"camera frame error: {exc}")
                time.sleep(0.2)
                continue
            if jpeg is not None:
                yield boundary + b"Content-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
            time.sleep(interval)
