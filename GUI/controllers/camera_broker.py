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

    def get_jpeg(self) -> bytes | None:
        color, _depth, _intr, _scale = self._session.get_latest_aligned_rgbd(timeout_s=5.0)
        frame = self.overlay(color)
        ok, buf = cv2.imencode(".jpg", frame, self._encode_params)
        if not ok:
            return None
        return buf.tobytes()

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
