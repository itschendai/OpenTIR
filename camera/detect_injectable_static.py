import argparse
import cv2
import numpy as np
import sys
from pathlib import Path
from contextlib import nullcontext

TEAL_LOW  = np.array([84, 110,  70])
TEAL_HIGH = np.array([100, 255, 255])
PINK_LOW  = np.array([125, 40,  40])
PINK_HIGH = np.array([175, 255, 255])

MIN_PINK_AREA  = 300
MIN_TEAL_AREA  = 150
DIST_TOLERANCE = 0.30  # accept pairings within ±30% of expected distance
AXIS_COS_MIN   = 0.70  # teal must lie within ~45° of pink's long axis

_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))


def _temporarily_release_shared_camera(serial: str | None):
    project_dir = Path(__file__).resolve().parent.parent
    if str(project_dir) not in sys.path:
        sys.path.insert(0, str(project_dir))
    try:
        from helper.injectable_camera_session import temporarily_release_shared_camera as _release
    except ImportError:
        return nullcontext()
    return _release(serial)


def _clip_hsv_triplet(values):
    arr = np.asarray(values, dtype=int).reshape(3)
    return np.array(
        [
            int(np.clip(arr[0], 0, 179)),
            int(np.clip(arr[1], 0, 255)),
            int(np.clip(arr[2], 0, 255)),
        ],
        dtype=np.uint8,
    )


def _parse_hsv_triplet(text):
    parts = str(text).replace(",", " ").split()
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f"Expected HSV triplet like '75,70,60', got: {text!r}"
        )
    try:
        values = [int(part) for part in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"HSV triplet must contain integers, got: {text!r}"
        ) from exc
    return _clip_hsv_triplet(values)


def _format_hsv_triplet(values):
    return ",".join(str(int(v)) for v in np.asarray(values, dtype=int).reshape(3))


def current_detector_config():
    return {
        "teal_low": TEAL_LOW.copy(),
        "teal_high": TEAL_HIGH.copy(),
        "pink_low": PINK_LOW.copy(),
        "pink_high": PINK_HIGH.copy(),
        "min_pink_area": int(MIN_PINK_AREA),
        "min_teal_area": int(MIN_TEAL_AREA),
        "dist_tolerance": float(DIST_TOLERANCE),
        "axis_cos_min": float(AXIS_COS_MIN),
    }


def apply_detector_config(
    *,
    teal_low=None,
    teal_high=None,
    pink_low=None,
    pink_high=None,
    min_pink_area=None,
    min_teal_area=None,
    dist_tolerance=None,
    axis_cos_min=None,
):
    global TEAL_LOW, TEAL_HIGH, PINK_LOW, PINK_HIGH
    global MIN_PINK_AREA, MIN_TEAL_AREA, DIST_TOLERANCE, AXIS_COS_MIN

    if teal_low is not None:
        TEAL_LOW = _clip_hsv_triplet(teal_low)
    if teal_high is not None:
        TEAL_HIGH = _clip_hsv_triplet(teal_high)
    if pink_low is not None:
        PINK_LOW = _clip_hsv_triplet(pink_low)
    if pink_high is not None:
        PINK_HIGH = _clip_hsv_triplet(pink_high)
    if min_pink_area is not None:
        MIN_PINK_AREA = int(min_pink_area)
    if min_teal_area is not None:
        MIN_TEAL_AREA = int(min_teal_area)
    if dist_tolerance is not None:
        DIST_TOLERANCE = float(dist_tolerance)
    if axis_cos_min is not None:
        AXIS_COS_MIN = float(axis_cos_min)


def build_color_masks(
    img: np.ndarray,
    *,
    teal_low=None,
    teal_high=None,
    pink_low=None,
    pink_high=None,
):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    teal_low = TEAL_LOW if teal_low is None else _clip_hsv_triplet(teal_low)
    teal_high = TEAL_HIGH if teal_high is None else _clip_hsv_triplet(teal_high)
    pink_low = PINK_LOW if pink_low is None else _clip_hsv_triplet(pink_low)
    pink_high = PINK_HIGH if pink_high is None else _clip_hsv_triplet(pink_high)

    not_black = (hsv[:, :, 2] > 40).astype(np.uint8) * 255
    pink_mask = cv2.morphologyEx(
        cv2.bitwise_and(cv2.inRange(hsv, pink_low, pink_high), not_black),
        cv2.MORPH_CLOSE,
        _kernel,
    )
    teal_mask = cv2.morphologyEx(
        cv2.bitwise_and(cv2.inRange(hsv, teal_low, teal_high), not_black),
        cv2.MORPH_CLOSE,
        _kernel,
    )
    return hsv, pink_mask, teal_mask


def _realsense_devices(rs):
    devices = []
    for device in rs.context().query_devices():
        devices.append(
            {
                "name": device.get_info(rs.camera_info.name),
                "serial": device.get_info(rs.camera_info.serial_number),
                "firmware": device.get_info(rs.camera_info.firmware_version),
            }
        )
    return devices


def _select_camera(devices, requested_serial=None):
    if requested_serial:
        for device in devices:
            if device["serial"] == requested_serial:
                return device
        return None

    for device in devices:
        if "D405" in device["name"]:
            return device

    return devices[0] if devices else None


def _find_color_sensor(rs, profile):
    for sensor in profile.get_device().query_sensors():
        if (
            sensor.supports(rs.option.white_balance)
            or sensor.supports(rs.option.enable_auto_white_balance)
            or sensor.supports(rs.option.exposure)
        ):
            return sensor
    return None


def _set_sensor_option(rs, sensor, option, value, label):
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


def _configure_color_sensor(rs, profile, exposure=None, gain=None, white_balance=None):
    color_sensor = _find_color_sensor(rs, profile)
    if color_sensor is None:
        print("Could not find a configurable color sensor.")
        return

    if exposure is not None and color_sensor.supports(rs.option.enable_auto_exposure):
        color_sensor.set_option(rs.option.enable_auto_exposure, 0)
        print("Set auto exposure: off")
    _set_sensor_option(rs, color_sensor, rs.option.exposure, exposure, "exposure")
    _set_sensor_option(rs, color_sensor, rs.option.gain, gain, "gain")

    if white_balance is not None and color_sensor.supports(rs.option.enable_auto_white_balance):
        color_sensor.set_option(rs.option.enable_auto_white_balance, 0)
        print("Set auto white balance: off")
    _set_sensor_option(rs, color_sensor, rs.option.white_balance, white_balance, "white balance")


def capture_robot_camera(
    serial: str | None = None,
    width: int = 640,
    height: int = 480,
    fps: int = 30,
    warmup_frames: int = 30,
    exposure: float | None = None,
    gain: float | None = None,
    white_balance: float | None = None,
) -> np.ndarray:
    try:
        import pyrealsense2 as rs
    except ImportError:
        print("Missing dependency: pyrealsense2. Install it with: pip install pyrealsense2")
        sys.exit(1)

    devices = _realsense_devices(rs)
    if not devices:
        print("No Intel RealSense camera detected.")
        sys.exit(1)

    camera = _select_camera(devices, serial)
    if camera is None:
        print(f"No RealSense camera found with serial number: {serial}")
        sys.exit(1)

    print(
        "Using camera: {name} | serial: {serial} | firmware: {firmware}".format(
            **camera
        )
    )

    with _temporarily_release_shared_camera(camera["serial"]):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(camera["serial"])
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        try:
            profile = pipeline.start(config)
        except RuntimeError as exc:
            print(f"Could not start camera stream: {exc}")
            print("Try a different resolution/FPS, for example: --width 640 --height 480 --fps 30")
            sys.exit(1)

        _configure_color_sensor(rs, profile, exposure, gain, white_balance)

        try:
            color_frame = None
            for _ in range(max(warmup_frames, 1)):
                frames = pipeline.wait_for_frames(timeout_ms=5000)
                color_frame = frames.get_color_frame()

            if not color_frame:
                print("Camera did not return a color frame.")
                sys.exit(1)

            return np.asanyarray(color_frame.get_data()).copy()
        finally:
            pipeline.stop()


def _centroid(cnt):
    M = cv2.moments(cnt)
    if M["m00"] == 0:
        return None
    return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


def _long_axis_vec(cnt):
    _, (w, h), angle = cv2.minAreaRect(cnt)
    a = np.radians(angle + 90 if h > w else angle)
    return np.array([np.cos(a), np.sin(a)])


def detect(
    img: np.ndarray,
    *,
    teal_low=None,
    teal_high=None,
    pink_low=None,
    pink_high=None,
    min_pink_area=None,
    min_teal_area=None,
    dist_tolerance=None,
    axis_cos_min=None,
) -> tuple[np.ndarray, list[dict]]:
    _, pink_mask, teal_mask = build_color_masks(
        img,
        teal_low=teal_low,
        teal_high=teal_high,
        pink_low=pink_low,
        pink_high=pink_high,
    )
    min_pink_area = MIN_PINK_AREA if min_pink_area is None else int(min_pink_area)
    min_teal_area = MIN_TEAL_AREA if min_teal_area is None else int(min_teal_area)
    dist_tolerance = (
        DIST_TOLERANCE if dist_tolerance is None else float(dist_tolerance)
    )
    axis_cos_min = AXIS_COS_MIN if axis_cos_min is None else float(axis_cos_min)

    def blobs(mask, min_area, with_axis=False):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        result = []
        for c in cnts:
            if cv2.contourArea(c) < min_area:
                continue
            b = {"contour": c, "center": _centroid(c)}
            if with_axis:
                b["axis"] = _long_axis_vec(c)
            result.append(b)
        return result

    pink_blobs = blobs(pink_mask, min_pink_area, with_axis=True)
    teal_blobs = blobs(teal_mask, min_teal_area)

    # All pairwise (distance, axis_alignment, pink_idx, teal_idx)
    pairs = []
    for i, pb in enumerate(pink_blobs):
        if pb["center"] is None:
            continue
        for j, tb in enumerate(teal_blobs):
            if tb["center"] is None:
                continue
            dx = tb["center"][0] - pb["center"][0]
            dy = tb["center"][1] - pb["center"][1]
            d  = np.hypot(dx, dy)
            if d < 1:
                continue
            cos = float(abs(np.dot(pb["axis"], [dx / d, dy / d])))
            pairs.append((d, cos, i, j))
    pairs.sort()

    if not pairs:
        return img.copy(), []

    # Estimate expected intra-injectable distance from the N shortest unique pairs
    n = min(len(pink_blobs), len(teal_blobs))
    used_p, used_t, seeds = set(), set(), []
    for d, _, i, j in pairs:
        if i not in used_p and j not in used_t:
            seeds.append(d); used_p.add(i); used_t.add(j)
        if len(seeds) == n:
            break

    exp   = float(np.median(seeds))
    lo, hi = exp * (1 - dist_tolerance), exp * (1 + dist_tolerance)

    # Greedy global assignment within distance window + axis constraint
    used_p, used_t, matched = set(), set(), []
    for d, cos, i, j in pairs:
        if d > hi:
            break
        if d < lo or cos < axis_cos_min or i in used_p or j in used_t:
            continue
        used_p.add(i); used_t.add(j)
        matched.append((i, j, d))

    if not matched:
        return img.copy(), []

    # Rotated bounding boxes: angle from pink->teal vector, size fixed to median
    boxes = []
    for pi, ti, d in matched:
        pb, tb = pink_blobs[pi], teal_blobs[ti]
        pc, tc = np.array(pb["center"], float), np.array(tb["center"], float)
        angle  = float(np.degrees(np.arctan2(tc[1] - pc[1], tc[0] - pc[0])))
        (cx, cy), (w, h), _ = cv2.minAreaRect(np.vstack([pb["contour"], tb["contour"]]))
        boxes.append({"cx": cx, "cy": cy, "angle": angle,
                      "long": max(w, h), "short": min(w, h),
                      "pink": pb["center"], "teal": tb["center"]})

    ref_long  = float(np.median([b["long"]  for b in boxes]))
    ref_short = float(np.median([b["short"] for b in boxes]))

    result, detections = img.copy(), []
    for i, b in enumerate(boxes):
        rect    = ((b["cx"], b["cy"]), (ref_long, ref_short), b["angle"])
        box_pts = cv2.boxPoints(rect).astype(np.int32)
        cv2.drawContours(result, [box_pts], 0, (0, 220, 0), 3)
        cv2.circle(result, b["pink"], 8, (255, 0, 200), -1)
        cv2.circle(result, b["teal"], 8, (0, 200, 150), -1)
        top = tuple(box_pts[box_pts[:, 1].argmin()])
        cv2.putText(result, f"#{i+1}", (top[0], max(top[1] - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 0), 2)
        detections.append({"box": box_pts, "rect": rect,
                            "pink_center": b["pink"], "teal_center": b["teal"]})

    return result, detections


def parse_args():
    parser = argparse.ArgumentParser(
        description="Capture a robot-camera still image and detect injectables."
    )
    parser.add_argument(
        "image",
        nargs="?",
        help="Optional existing image path. If omitted, capture from the robot camera.",
    )
    parser.add_argument(
        "--capture",
        action="store_true",
        help="Capture from the robot camera even if an image path is provided.",
    )
    parser.add_argument("--serial", help="Specific RealSense serial number to use")
    parser.add_argument("--width", type=int, default=640, help="Camera stream width")
    parser.add_argument("--height", type=int, default=480, help="Camera stream height")
    parser.add_argument("--fps", type=int, default=30, help="Camera stream frame rate")
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=30,
        help="Frames to discard before taking the still image.",
    )
    parser.add_argument(
        "--capture-out",
        default="injectable_capture.jpg",
        help="Path to save the raw captured image.",
    )
    parser.add_argument(
        "--out",
        help="Path to save the annotated detection image. Defaults to <input>_detected.jpg.",
    )
    parser.add_argument(
        "--exposure",
        type=float,
        default=19000,
        help="Manual color exposure. Setting this turns off auto exposure.",
    )
    parser.add_argument("--gain", type=float, help="Manual color gain")
    parser.add_argument(
        "--white-balance",
        type=float,
        help="Manual white balance in Kelvin. Setting this turns off auto white balance.",
    )
    parser.add_argument(
        "--teal-low",
        type=_parse_hsv_triplet,
        default=TEAL_LOW.copy(),
        help=f"Lower HSV bound for teal, default: {_format_hsv_triplet(TEAL_LOW)}",
    )
    parser.add_argument(
        "--teal-high",
        type=_parse_hsv_triplet,
        default=TEAL_HIGH.copy(),
        help=f"Upper HSV bound for teal, default: {_format_hsv_triplet(TEAL_HIGH)}",
    )
    parser.add_argument(
        "--pink-low",
        type=_parse_hsv_triplet,
        default=PINK_LOW.copy(),
        help=f"Lower HSV bound for pink, default: {_format_hsv_triplet(PINK_LOW)}",
    )
    parser.add_argument(
        "--pink-high",
        type=_parse_hsv_triplet,
        default=PINK_HIGH.copy(),
        help=f"Upper HSV bound for pink, default: {_format_hsv_triplet(PINK_HIGH)}",
    )
    parser.add_argument(
        "--min-pink-area",
        type=int,
        default=MIN_PINK_AREA,
        help=f"Minimum contour area for pink blobs, default: {MIN_PINK_AREA}",
    )
    parser.add_argument(
        "--min-teal-area",
        type=int,
        default=MIN_TEAL_AREA,
        help=f"Minimum contour area for teal blobs, default: {MIN_TEAL_AREA}",
    )
    parser.add_argument(
        "--dist-tolerance",
        type=float,
        default=DIST_TOLERANCE,
        help=f"Relative distance tolerance for pink/teal pairing, default: {DIST_TOLERANCE}",
    )
    parser.add_argument(
        "--axis-cos-min",
        type=float,
        default=AXIS_COS_MIN,
        help=f"Minimum axis alignment cosine for teal pairing, default: {AXIS_COS_MIN}",
    )
    parser.add_argument(
        "--mask-dir",
        help="Optional directory to save pink/teal mask images for manual tuning.",
    )
    parser.add_argument(
        "--print-config",
        action="store_true",
        help="Print the effective detector config and a reusable CLI snippet.",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    if args.image and not args.capture:
        input_path = Path(args.image)
        img = cv2.imread(str(input_path))
        if img is None:
            print(f"Cannot read: {input_path}")
            return 1
    else:
        img = capture_robot_camera(
            serial=args.serial,
            width=args.width,
            height=args.height,
            fps=args.fps,
            warmup_frames=args.warmup_frames,
            exposure=args.exposure,
            gain=args.gain,
            white_balance=args.white_balance,
        )
        input_path = Path(args.capture_out)
        input_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(input_path), img):
            print(f"Could not save captured image: {input_path}")
            return 1
        print(f"Captured -> {input_path}")

    result, found = detect(
        img,
        teal_low=args.teal_low,
        teal_high=args.teal_high,
        pink_low=args.pink_low,
        pink_high=args.pink_high,
        min_pink_area=args.min_pink_area,
        min_teal_area=args.min_teal_area,
        dist_tolerance=args.dist_tolerance,
        axis_cos_min=args.axis_cos_min,
    )
    print(f"Detected {len(found)} injectable(s)")

    _, pink_mask, teal_mask = build_color_masks(
        img,
        teal_low=args.teal_low,
        teal_high=args.teal_high,
        pink_low=args.pink_low,
        pink_high=args.pink_high,
    )
    print(
        "Mask pixels: "
        f"pink={int(np.count_nonzero(pink_mask))} "
        f"teal={int(np.count_nonzero(teal_mask))}"
    )

    out = Path(args.out) if args.out else input_path.with_name(f"{input_path.stem}_detected.jpg")
    out.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(out), result):
        print(f"Could not save detection image: {out}")
        return 1
    print(f"Saved -> {out}")

    if args.mask_dir:
        mask_dir = Path(args.mask_dir)
        mask_dir.mkdir(parents=True, exist_ok=True)
        pink_path = mask_dir / f"{input_path.stem}_pink_mask.png"
        teal_path = mask_dir / f"{input_path.stem}_teal_mask.png"
        if not cv2.imwrite(str(pink_path), pink_mask):
            print(f"Could not save pink mask: {pink_path}")
            return 1
        if not cv2.imwrite(str(teal_path), teal_mask):
            print(f"Could not save teal mask: {teal_path}")
            return 1
        print(f"Saved -> {pink_path}")
        print(f"Saved -> {teal_path}")

    if args.print_config:
        print("Detector config:")
        print(f"  teal_low={_format_hsv_triplet(args.teal_low)}")
        print(f"  teal_high={_format_hsv_triplet(args.teal_high)}")
        print(f"  pink_low={_format_hsv_triplet(args.pink_low)}")
        print(f"  pink_high={_format_hsv_triplet(args.pink_high)}")
        print(f"  min_pink_area={int(args.min_pink_area)}")
        print(f"  min_teal_area={int(args.min_teal_area)}")
        print(f"  dist_tolerance={float(args.dist_tolerance):.3f}")
        print(f"  axis_cos_min={float(args.axis_cos_min):.3f}")
        print("Reuse with:")
        print(
            "  "
            f"--teal-low {_format_hsv_triplet(args.teal_low)} "
            f"--teal-high {_format_hsv_triplet(args.teal_high)} "
            f"--pink-low {_format_hsv_triplet(args.pink_low)} "
            f"--pink-high {_format_hsv_triplet(args.pink_high)} "
            f"--min-pink-area {int(args.min_pink_area)} "
            f"--min-teal-area {int(args.min_teal_area)} "
            f"--dist-tolerance {float(args.dist_tolerance):.3f} "
            f"--axis-cos-min {float(args.axis_cos_min):.3f}"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
