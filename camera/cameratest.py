#!/usr/bin/env python3

"""Check connection to an Intel RealSense D405 and open a live view."""

import argparse
import os
import subprocess
import sys

try:
    import cv2
except ImportError:
    print("Missing dependency: opencv-python. Install it with: pip install opencv-python")
    sys.exit(1)

try:
    import numpy as np
    import pyrealsense2 as rs
except ImportError:
    print("Missing dependency: pyrealsense2. Install it with: pip install pyrealsense2")
    sys.exit(1)


def list_realsense_devices():
    """Return connected RealSense devices as dictionaries."""
    context = rs.context()
    devices = []

    for device in context.query_devices():
        product_line = device.get_info(rs.camera_info.product_line)
        name = device.get_info(rs.camera_info.name)
        serial = device.get_info(rs.camera_info.serial_number)
        firmware = device.get_info(rs.camera_info.firmware_version)
        devices.append(
            {
                "device": device,
                "name": name,
                "product_line": product_line,
                "serial": serial,
                "firmware": firmware,
            }
        )

    return devices


def find_camera(devices, requested_serial=None):
    if requested_serial:
        for device in devices:
            if device["serial"] == requested_serial:
                return device
        return None

    for device in devices:
        if "D405" in device["name"]:
            return device

    return devices[0] if devices else None


def make_colorized_depth(depth_frame):
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)
    return cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)


def find_color_sensor(profile):
    for sensor in profile.get_device().query_sensors():
        if (
            sensor.supports(rs.option.white_balance)
            or sensor.supports(rs.option.enable_auto_white_balance)
            or sensor.supports(rs.option.exposure)
        ):
            return sensor
    return None


def set_sensor_option(sensor, option, value, label):
    if value is None:
        return
    if not sensor.supports(option):
        print(f"Color sensor does not support {label}.")
        return

    option_range = sensor.get_option_range(option)
    if value < option_range.min or value > option_range.max:
        print(
            f"Warning: {label}={value} is outside supported range "
            f"{option_range.min:g}..{option_range.max:g}"
        )

    sensor.set_option(option, float(value))
    actual = sensor.get_option(option)
    print(f"Set {label}: {actual:g}")


def configure_color_sensor(profile, exposure=None, gain=None, white_balance=None):
    color_sensor = find_color_sensor(profile)
    if color_sensor is None:
        print("Could not find a configurable color sensor.")
        return

    if exposure is not None and color_sensor.supports(rs.option.enable_auto_exposure):
        color_sensor.set_option(rs.option.enable_auto_exposure, 0)
        print("Set auto exposure: off")
    set_sensor_option(color_sensor, rs.option.exposure, exposure, "exposure")
    set_sensor_option(color_sensor, rs.option.gain, gain, "gain")

    if white_balance is not None and color_sensor.supports(rs.option.enable_auto_white_balance):
        color_sensor.set_option(rs.option.enable_auto_white_balance, 0)
        print("Set auto white balance: off")
    set_sensor_option(color_sensor, rs.option.white_balance, white_balance, "white balance")


def camera_runtime_metadata(device):
    info_keys = {
        "usb_type": "usb_type_descriptor",
        "physical_port": "physical_port",
    }
    metadata = {}
    for out_key, info_name in info_keys.items():
        info = getattr(rs.camera_info, info_name, None)
        if info is None:
            continue
        try:
            metadata[out_key] = device.get_info(info)
        except RuntimeError:
            continue
    return metadata


def video_device_holders():
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


def format_frame_timeout_diagnostics(camera, exc):
    metadata = camera_runtime_metadata(camera["device"])
    lines = [
        f"Camera stopped delivering frames: {exc}",
        "Camera details: {name} | serial: {serial} | firmware: {firmware}".format(
            **camera
        ),
    ]

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

    holders = video_device_holders()
    if holders:
        lines.append("Current /dev/video holders:")
        lines.append(holders)
    else:
        lines.append("No current /dev/video holder process was detected.")

    lines.append("Try reconnecting the camera to a USB 3 port, avoiding passive hubs/docks, then rerun the test.")
    return "\n".join(lines)


def run_live_view(camera, width, height, fps, exposure=None, gain=None, white_balance=None):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(camera["serial"])
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

    align_to_color = rs.align(rs.stream.color)

    try:
        profile = pipeline.start(config)
    except RuntimeError as exc:
        print(f"Could not start camera streams: {exc}")
        print("Try a different resolution/FPS, for example: --width 640 --height 480 --fps 30")
        return 1

    configure_color_sensor(profile, exposure, gain, white_balance)

    runtime_metadata = camera_runtime_metadata(camera["device"])
    usb_type = runtime_metadata.get("usb_type")

    print(
        "Connected to {name} (serial: {serial}, firmware: {firmware})".format(
            **camera
        )
    )
    if usb_type:
        print(f"USB descriptor: {usb_type}")
        if usb_type.startswith("2."):
            print(
                "Warning: camera is enumerated as USB 2.x; live RGB-D may time out until it is moved "
                "to a SuperSpeed USB 3 connection."
            )
    print("Live view is running. Press 'q' or Esc to exit.")

    try:
        while True:
            try:
                frames = pipeline.wait_for_frames(timeout_ms=5000)
            except RuntimeError as exc:
                print(format_frame_timeout_diagnostics(camera, exc))
                return 1
            aligned_frames = align_to_color.process(frames)

            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_colormap = make_colorized_depth(depth_frame)

            if depth_colormap.shape[:2] != color_image.shape[:2]:
                depth_colormap = cv2.resize(
                    depth_colormap,
                    (color_image.shape[1], color_image.shape[0]),
                    interpolation=cv2.INTER_AREA,
                )

            live_view = np.hstack((color_image, depth_colormap))
            cv2.imshow("RealSense D405 Live View: color | depth", live_view)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Check connection to an Intel RealSense D405 and open a live view."
    )
    parser.add_argument("--serial", help="Specific RealSense serial number to use")
    parser.add_argument("--width", type=int, default=640, help="Stream width")
    parser.add_argument("--height", type=int, default=480, help="Stream height")
    parser.add_argument("--fps", type=int, default=30, help="Stream frame rate")
    parser.add_argument(
        "--exposure",
        type=float,
        help="Manual color exposure. Setting this turns off auto exposure.",
    )
    parser.add_argument("--gain", type=float, help="Manual color gain")
    parser.add_argument(
        "--white-balance",
        type=float,
        help="Manual white balance in Kelvin. Setting this turns off auto white balance.",
    )
    args = parser.parse_args()

    devices = list_realsense_devices()
    if not devices:
        print("No Intel RealSense camera detected.")
        return 1

    print("Detected RealSense devices:")
    for device in devices:
        marker = " <-- D405" if "D405" in device["name"] else ""
        print(
            "  - {name} | serial: {serial} | firmware: {firmware}{marker}".format(
                marker=marker,
                **device,
            )
        )

    camera = find_camera(devices, args.serial)
    if camera is None:
        print(f"No RealSense camera found with serial number: {args.serial}")
        return 1

    if "D405" not in camera["name"]:
        print(f"Warning: selected camera is not a D405: {camera['name']}")

    return run_live_view(
        camera,
        args.width,
        args.height,
        args.fps,
        exposure=args.exposure,
        gain=args.gain,
        white_balance=args.white_balance,
    )


if __name__ == "__main__":
    sys.exit(main())
