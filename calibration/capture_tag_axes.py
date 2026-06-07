#!/usr/bin/env python3

"""Capture a ChArUco board image and overlay the board-frame axes."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from calibrate_eye_in_hand import (
    DEFAULT_ROBOT_SN,
    DEFAULT_BOARD_PATH,
    DEFAULT_CALIBRATION_PATH,
    PROJECT_DIR,
    capture_realsense_color,
    camera_matrix_and_dist,
    create_charuco_board,
    detect_charuco_corners,
    estimate_charuco_pose,
    invert_transform,
    load_board_config,
    matrix_to_quat_wxyz,
    pose_vec_to_transform,
    read_current_tcp_pose,
    relative_to_repo,
    read_yaml,
    require_cv2,
    transform_record,
    timestamp_slug,
    utc_timestamp,
    vector_to_list,
)


HERE = Path(__file__).resolve().parent
DEFAULT_OUTPUT_DIR = HERE / "captures"
DEFAULT_VISE_KEY_POSITION = PROJECT_DIR / "key_positions" / "Vise_accurate.json"
DEFAULT_VISE_REFERENCE_PATH = HERE / "tag_01_to_vise_tcp.json"

# Each tag has a board YAML plus a list of key positions rigidly attached to it.
# Each key-position entry names the ground-truth JSON (read at --save-tag-reference
# time), the board-to-keypos reference JSON to write, a name_slug used for log
# lines, and whether to apply the tag-1-specific vise axis-alignment rule.
TAG_REGISTRY = {
    1: {
        "board": HERE / "tag_01.yaml",
        "key_positions": [
            {
                "name_slug": "vise",
                "key_position": PROJECT_DIR / "key_positions" / "Vise_accurate.json",
                "reference": HERE / "tag_01_to_vise_tcp.json",
                "constrain_x_vertical": True,
            },
        ],
    },
    2: {
        "board": HERE / "tag_02.yaml",
        "key_positions": [
            {
                "name_slug": "plate",
                "key_position": PROJECT_DIR / "key_positions" / "Plate_accurate.json",
                "reference": HERE / "tag_02_to_plate_tcp.json",
                "constrain_x_vertical": False,
            },
        ],
    },
    3: {
        "board": HERE / "tag_03.yaml",
        "key_positions": [
            {
                "name_slug": "spring",
                "key_position": PROJECT_DIR / "key_positions" / "Spring_accurate.json",
                "reference": HERE / "tag_03_to_spring_tcp.json",
                "constrain_x_vertical": False,
            },
            {
                "name_slug": "plastic",
                "key_position": PROJECT_DIR / "key_positions" / "Plastic_accurate.json",
                "reference": HERE / "tag_03_to_plastic_tcp.json",
                "constrain_x_vertical": False,
            },
            {
                "name_slug": "glass",
                "key_position": PROJECT_DIR / "key_positions" / "Glass_accurate.json",
                "reference": HERE / "tag_03_to_glass_tcp.json",
                "constrain_x_vertical": False,
            },
        ],
    },
}


def draw_detection_overlay(
    cv2,
    image: np.ndarray,
    board,
    dictionary,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    axis_length_m: float,
    min_charuco_corners: int,
):
    display = image.copy()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    pose = estimate_charuco_pose(
        cv2,
        image,
        board,
        dictionary,
        camera_matrix,
        dist_coeffs,
        min_charuco_corners=min_charuco_corners,
    )

    charuco_corners, charuco_ids, marker_corners, marker_ids = detect_charuco_corners(
        cv2, gray, board, dictionary, camera_matrix, dist_coeffs
    )

    if marker_ids is not None and len(marker_ids) > 0:
        cv2.aruco.drawDetectedMarkers(display, marker_corners, marker_ids)
    if charuco_ids is not None and len(charuco_ids) > 0:
        cv2.aruco.drawDetectedCornersCharuco(display, charuco_corners, charuco_ids)

    cv2.drawFrameAxes(
        display,
        camera_matrix,
        dist_coeffs,
        pose.rvec,
        pose.tvec,
        axis_length_m,
        2,
    )

    lines = [
        f"markers={pose.marker_count}  charuco={pose.charuco_count}",
        f"reproj={pose.reprojection_error_px:.2f}px",
        "axes: X red, Y green, Z blue",
        (
            "tvec_m="
            f"[{pose.tvec[0, 0]:.4f}, {pose.tvec[1, 0]:.4f}, {pose.tvec[2, 0]:.4f}]"
        ),
    ]
    y = 28
    for line in lines:
        cv2.putText(
            display,
            line,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            display,
            line,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (30, 30, 30),
            1,
            cv2.LINE_AA,
        )
        y += 26

    return display, pose


def load_intrinsics(calibration_path: Path) -> tuple[np.ndarray, np.ndarray]:
    calibration = read_yaml(calibration_path)
    return camera_matrix_and_dist(calibration)


def write_json(data: dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def load_key_position(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return data


def key_position_pose(path: Path) -> list[float]:
    data = load_key_position(path)
    pose = data.get("tcp_pose_world", {}).get("values", [])
    values = [float(v) for v in pose]
    if len(values) != 7:
        raise ValueError(f"{path} tcp_pose_world.values must have 7 values")
    return values


def key_position_name(path: Path) -> str:
    data = load_key_position(path)
    return str(data.get("key_position_name") or data.get("name") or path.stem)


def key_position_robot_sn(path: Path) -> str | None:
    data = load_key_position(path)
    robot_sn = data.get("robot_sn")
    return str(robot_sn) if robot_sn else None


def pose_record(values: list[float]) -> dict:
    return {
        "unit": "m, quaternion",
        "order": ["x", "y", "z", "qw", "qx", "qy", "qz"],
        "values": [float(v) for v in values],
    }


def transform_to_pose_record(T: np.ndarray) -> dict:
    T = np.asarray(T, dtype=float).reshape(4, 4)
    return pose_record(
        vector_to_list(T[:3, 3]) + vector_to_list(matrix_to_quat_wxyz(T[:3, :3]))
    )


def load_tcp_camera_transform(calibration_path: Path) -> np.ndarray:
    calibration = read_yaml(calibration_path)
    return np.asarray(calibration["T_tcp_camera"], dtype=float).reshape(4, 4)


def compute_world_board_pose(
    calibration_path: Path,
    tcp_pose_world: list[float],
    T_camera_board: np.ndarray,
) -> np.ndarray:
    T_world_tcp = pose_vec_to_transform(tcp_pose_world)
    T_tcp_camera = load_tcp_camera_transform(calibration_path)
    return T_world_tcp @ T_tcp_camera @ T_camera_board


def constrain_vise_tcp_to_tag_axes(
    T_world_vise_tcp: np.ndarray,
    T_world_board: np.ndarray,
) -> np.ndarray:
    """Align the vise TCP axes to the detected tag-1 board axes.

    Desired convention from hardware setup:
    - tag +X = TCP -Y
    - tag +Y = TCP -Z
    Therefore TCP +X = tag +Z.
    """
    constrained = np.asarray(T_world_vise_tcp, dtype=float).reshape(4, 4).copy()
    board = np.asarray(T_world_board, dtype=float).reshape(4, 4)

    board_x = board[:3, 0]
    board_z = board[:3, 2]
    x_axis = board_z / float(np.linalg.norm(board_z))
    y_hint = -board_x
    y_hint = y_hint / float(np.linalg.norm(y_hint))
    z_axis = np.cross(x_axis, y_hint)
    z_axis = z_axis / float(np.linalg.norm(z_axis))
    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / float(np.linalg.norm(y_axis))

    constrained[:3, :3] = np.column_stack([x_axis, y_axis, z_axis])
    return constrained


def save_tag_reference_json(
    out_path: Path,
    *,
    board_cfg: dict,
    calibration_path: Path,
    key_position_path: Path,
    capture_tcp_pose_world: list[float],
    T_world_board: np.ndarray,
    T_world_keypos_tcp: np.ndarray,
    constrain_x_vertical: bool,
) -> np.ndarray:
    if constrain_x_vertical:
        T_world_keypos_tcp = constrain_vise_tcp_to_tag_axes(
            T_world_keypos_tcp,
            T_world_board,
        )
    T_board_keypos_tcp = invert_transform(T_world_board) @ T_world_keypos_tcp
    T_keypos_tcp_board = invert_transform(T_board_keypos_tcp)
    keypos_name = key_position_name(key_position_path)
    data = {
        "schema_version": 1,
        "created_at": utc_timestamp(),
        "reference_type": "charuco_board_to_key_position_tcp",
        "description": (
            "Rigid transform from the detected ChArUco board frame to the saved "
            f"TCP pose for the {keypos_name} key position."
        ),
        "frame_convention": "T_a_b maps coordinates from frame b into frame a",
        "board": {
            "name": board_cfg["name"],
            "path": relative_to_repo(Path(board_cfg["path"])),
        },
        "calibration": {
            "path": relative_to_repo(calibration_path),
        },
        "source_key_position": {
            "name": keypos_name,
            "path": relative_to_repo(key_position_path),
            "robot_sn": key_position_robot_sn(key_position_path),
        },
        "capture_robot_tcp_pose_world": pose_record(capture_tcp_pose_world),
        "transforms": {
            "T_world_board": transform_record(T_world_board, "world", "board"),
            "T_world_keypos_tcp_reference": transform_record(
                T_world_keypos_tcp, "world", "keypos_tcp"
            ),
            "T_board_keypos_tcp": transform_record(
                T_board_keypos_tcp, "board", "keypos_tcp"
            ),
            "T_keypos_tcp_board": transform_record(
                T_keypos_tcp_board, "keypos_tcp", "board"
            ),
        },
        "reference_tcp_pose_world": transform_to_pose_record(T_world_keypos_tcp),
    }
    write_json(data, out_path)
    return T_board_keypos_tcp


def load_tag_reference_transform(path: Path) -> np.ndarray:
    data = load_key_position(path)
    transforms_dict = data.get("transforms", {})
    matrix = (
        transforms_dict.get("T_board_keypos_tcp", {}).get("matrix")
        or transforms_dict.get("T_board_vise_tcp", {}).get("matrix")
    )
    if matrix is None:
        raise ValueError(
            f"{path} does not contain transforms.T_board_keypos_tcp.matrix"
        )
    return np.asarray(matrix, dtype=float).reshape(4, 4)


# Backward-compat aliases for older callers.
def save_vise_reference_json(out_path, *, vise_key_position_path, T_world_vise_tcp, **kwargs):
    return save_tag_reference_json(
        out_path,
        key_position_path=vise_key_position_path,
        T_world_keypos_tcp=T_world_vise_tcp,
        constrain_x_vertical=True,
        **kwargs,
    )


def load_vise_reference_transform(path: Path) -> np.ndarray:
    return load_tag_reference_transform(path)


def load_reference_tcp_transform(path: Path) -> np.ndarray:
    data = load_key_position(path)
    values = data.get("reference_tcp_pose_world", {}).get("values")
    if values is None:
        raise ValueError(f"{path} does not contain reference_tcp_pose_world.values")
    pose = [float(v) for v in values]
    if len(pose) != 7:
        raise ValueError(f"{path} reference_tcp_pose_world.values must have 7 values")
    return pose_vec_to_transform(pose)


def save_predicted_keypos_pose_json(
    out_path: Path,
    *,
    reference_path: Path,
    board_cfg: dict,
    calibration_path: Path,
    capture_tcp_pose_world: list[float],
    T_world_board: np.ndarray,
    T_world_keypos_tcp: np.ndarray,
) -> None:
    data = {
        "schema_version": 1,
        "created_at": utc_timestamp(),
        "prediction_type": "key_position_tcp_from_charuco_board",
        "frame_convention": "T_a_b maps coordinates from frame b into frame a",
        "reference_path": relative_to_repo(reference_path),
        "board": {
            "name": board_cfg["name"],
            "path": relative_to_repo(Path(board_cfg["path"])),
        },
        "calibration": {
            "path": relative_to_repo(calibration_path),
        },
        "capture_robot_tcp_pose_world": pose_record(capture_tcp_pose_world),
        "transforms": {
            "T_world_board": transform_record(T_world_board, "world", "board"),
            "T_world_keypos_tcp": transform_record(
                T_world_keypos_tcp, "world", "keypos_tcp"
            ),
        },
        "predicted_tcp_pose_world": transform_to_pose_record(T_world_keypos_tcp),
    }
    write_json(data, out_path)


# Backward-compat alias for older callers.
def save_predicted_vise_pose_json(out_path, *, T_world_vise_tcp, **kwargs):
    return save_predicted_keypos_pose_json(
        out_path, T_world_keypos_tcp=T_world_vise_tcp, **kwargs
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Capture one RealSense color image, detect the ChArUco board from tag_01.yaml, "
            "and overlay the board-frame x/y/z axes."
        )
    )
    parser.add_argument(
        "--board",
        default=str(DEFAULT_BOARD_PATH),
        help="Path to the ChArUco board YAML.",
    )
    parser.add_argument(
        "--calibration",
        default=str(DEFAULT_CALIBRATION_PATH),
        help=(
            "Calibration YAML used only when --use-calibration-intrinsics is set."
        ),
    )
    parser.add_argument(
        "--use-calibration-intrinsics",
        action="store_true",
        help="Use camera intrinsics stored in the calibration YAML instead of the live stream.",
    )
    parser.add_argument("--serial", help="Specific RealSense serial number to use.")
    parser.add_argument("--width", type=int, default=640, help="Capture width.")
    parser.add_argument("--height", type=int, default=480, help="Capture height.")
    parser.add_argument("--fps", type=int, default=30, help="Capture frame rate.")
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=30,
        help="Number of frames to discard before capturing.",
    )
    parser.add_argument(
        "--exposure",
        type=float,
        help="Manual color exposure. Setting this turns off auto exposure.",
    )
    parser.add_argument("--gain", type=float, help="Manual color gain.")
    parser.add_argument(
        "--white-balance",
        type=float,
        help="Manual white balance in Kelvin. Setting this turns off auto white balance.",
    )
    parser.add_argument(
        "--min-charuco-corners",
        type=int,
        default=8,
        help="Minimum number of ChArUco corners required for pose estimation.",
    )
    parser.add_argument(
        "--axis-length-m",
        type=float,
        help="Axis length in meters. Defaults to 2 board squares.",
    )
    parser.add_argument(
        "--output-dir",
        default=str(DEFAULT_OUTPUT_DIR),
        help="Directory for raw and annotated captures.",
    )
    parser.add_argument(
        "--basename",
        help="Base filename for outputs. Defaults to a timestamp slug.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show the annotated image in a window after capture.",
    )
    parser.add_argument(
        "--tag",
        type=int,
        default=1,
        choices=sorted(TAG_REGISTRY),
        help=(
            "ChArUco tag id to operate on. 1=Vise, 2=Plate, 3=Spring. "
            "Selects board YAML, ground-truth key position, and reference JSON paths."
        ),
    )
    parser.add_argument(
        "--save-tag-reference",
        action="store_true",
        help=(
            "Read the current robot TCP and the tag's key-position JSON, then save the "
            "board-to-key-position TCP reference transform for later relocation recovery."
        ),
    )
    parser.add_argument(
        "--apply-tag-reference",
        action="store_true",
        help=(
            "Load a previously saved board-to-key-position TCP reference and estimate the "
            "current world-frame key-position TCP pose from the live tag observation."
        ),
    )
    parser.add_argument(
        "--key-position",
        default=None,
        help=(
            "Override the path to the saved key-position JSON. "
            "Defaults to the file from TAG_REGISTRY for --tag."
        ),
    )
    parser.add_argument(
        "--reference-json",
        default=None,
        help=(
            "Override the path to the board-to-key-position reference JSON. "
            "Defaults to the file from TAG_REGISTRY for --tag."
        ),
    )
    parser.add_argument(
        "--predicted-json",
        default=None,
        help="Optional output path for the predicted world-frame key-position TCP pose JSON.",
    )
    # Backward-compat aliases for --tag 1 (Vise).
    parser.add_argument(
        "--save-vise-reference",
        dest="save_vise_reference",
        action="store_true",
        help="Backward-compat alias for --save-tag-reference --tag 1.",
    )
    parser.add_argument(
        "--apply-vise-reference",
        dest="apply_vise_reference",
        action="store_true",
        help="Backward-compat alias for --apply-tag-reference --tag 1.",
    )
    parser.add_argument(
        "--vise-key-position",
        dest="vise_key_position",
        default=None,
        help="Backward-compat alias for --key-position (forces --tag 1).",
    )
    parser.add_argument(
        "--vise-reference-json",
        dest="vise_reference_json",
        default=None,
        help="Backward-compat alias for --reference-json (forces --tag 1).",
    )
    parser.add_argument(
        "--predicted-vise-json",
        dest="predicted_vise_json",
        default=None,
        help="Backward-compat alias for --predicted-json (forces --tag 1).",
    )
    parser.add_argument(
        "--robot-sn",
        help="Robot serial number for reading the current TCP pose.",
    )
    parser.add_argument(
        "--operational-timeout-s",
        type=float,
        default=10.0,
        help="Timeout when connecting to the robot to read the current TCP pose.",
    )
    args = parser.parse_args()

    # Promote backward-compat aliases. If the user passed a --vise-* flag we
    # force tag 1 (since the legacy behavior only ever covered Vise) and lift
    # any explicit path overrides into the new flag names.
    legacy_used = any(
        [
            args.save_vise_reference,
            args.apply_vise_reference,
            args.vise_key_position is not None,
            args.vise_reference_json is not None,
            args.predicted_vise_json is not None,
        ]
    )
    if legacy_used:
        if args.tag != 1:
            raise SystemExit(
                "Cannot use legacy --vise-* flags with --tag != 1. "
                "Use --save-tag-reference / --apply-tag-reference instead."
            )
        if args.save_vise_reference:
            args.save_tag_reference = True
        if args.apply_vise_reference:
            args.apply_tag_reference = True
        if args.vise_key_position is not None and args.key_position is None:
            args.key_position = args.vise_key_position
        if args.vise_reference_json is not None and args.reference_json is None:
            args.reference_json = args.vise_reference_json
        if args.predicted_vise_json is not None and args.predicted_json is None:
            args.predicted_json = args.predicted_vise_json

    tag_entry = TAG_REGISTRY[args.tag]
    # --board CLI override falls back to the registry default for the chosen tag.
    if args.board == str(DEFAULT_BOARD_PATH):
        args.board = str(tag_entry["board"])
    # If the user pinned --key-position or --reference-json, treat the run as a
    # single-entry override (legacy single-keypos flow). Otherwise iterate over
    # every key position rigidly attached to this tag.
    if args.key_position is not None or args.reference_json is not None:
        base = tag_entry["key_positions"][0]
        keypos_entries = [
            {
                "name_slug": base["name_slug"],
                "key_position": Path(args.key_position or base["key_position"]),
                "reference": Path(args.reference_json or base["reference"]),
                "constrain_x_vertical": base["constrain_x_vertical"],
            }
        ]
    else:
        keypos_entries = [
            {
                "name_slug": kp["name_slug"],
                "key_position": Path(kp["key_position"]),
                "reference": Path(kp["reference"]),
                "constrain_x_vertical": kp["constrain_x_vertical"],
            }
            for kp in tag_entry["key_positions"]
        ]

    cv2 = require_cv2()

    board_cfg = load_board_config(Path(args.board))
    board, dictionary = create_charuco_board(cv2, board_cfg)
    axis_length_m = (
        float(args.axis_length_m)
        if args.axis_length_m is not None
        else 2.0 * float(board_cfg["square_length_m"])
    )

    image, live_intrinsics, camera = capture_realsense_color(
        serial=args.serial,
        width=args.width,
        height=args.height,
        fps=args.fps,
        warmup_frames=args.warmup_frames,
        exposure=args.exposure,
        gain=args.gain,
        white_balance=args.white_balance,
    )

    if args.use_calibration_intrinsics:
        camera_matrix, dist_coeffs = load_intrinsics(Path(args.calibration))
    else:
        camera_matrix = np.asarray(live_intrinsics["camera_matrix"], dtype=float).reshape(3, 3)
        dist_coeffs = np.asarray(live_intrinsics["dist_coeffs"], dtype=float).reshape(-1, 1)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    basename = args.basename or timestamp_slug()
    raw_path = output_dir / f"{basename}_raw.png"
    annotated_path = output_dir / f"{basename}_axes.png"

    cv2.imwrite(str(raw_path), image)

    try:
        annotated, pose = draw_detection_overlay(
            cv2,
            image,
            board,
            dictionary,
            camera_matrix,
            dist_coeffs,
            axis_length_m,
            args.min_charuco_corners,
        )
    except Exception as exc:
        print(f"Saved raw capture to: {raw_path}")
        raise SystemExit(f"Board detection failed: {exc}") from exc

    cv2.imwrite(str(annotated_path), annotated)

    print(
        "Captured board with camera: "
        f"{camera['name']} | serial: {camera['serial']} | firmware: {camera['firmware']}"
    )
    print(f"Saved raw image: {raw_path}")
    print(f"Saved annotated image: {annotated_path}")
    print(
        "Board pose (camera frame, meters): "
        f"tvec=[{pose.tvec[0, 0]:.4f}, {pose.tvec[1, 0]:.4f}, {pose.tvec[2, 0]:.4f}]"
    )
    print(
        f"Detected {pose.marker_count} markers and {pose.charuco_count} ChArUco corners "
        f"(reprojection error {pose.reprojection_error_px:.2f} px)."
    )
    print("Axis colors: X red, Y green, Z blue")

    need_robot_pose = args.save_tag_reference or args.apply_tag_reference
    if need_robot_pose:
        robot_sn = args.robot_sn
        if not robot_sn:
            for kp in keypos_entries:
                if kp["key_position"].exists():
                    robot_sn = key_position_robot_sn(kp["key_position"])
                    if robot_sn:
                        break
        if not robot_sn:
            robot_sn = DEFAULT_ROBOT_SN

        tcp_pose_world = read_current_tcp_pose(robot_sn, args.operational_timeout_s)
        calibration_path = Path(args.calibration)
        T_world_board = compute_world_board_pose(
            calibration_path, tcp_pose_world, pose.T_camera_board
        )
        print(
            "Board pose (world frame, meters): "
            f"xyz=[{T_world_board[0, 3]:.4f}, {T_world_board[1, 3]:.4f}, "
            f"{T_world_board[2, 3]:.4f}]"
        )

        # For --apply with --predicted-json + multiple key positions, distinguish
        # outputs by suffixing the name slug rather than overwriting one file.
        predicted_base = Path(args.predicted_json) if args.predicted_json else None
        emit_multi = len(keypos_entries) > 1

        for kp in keypos_entries:
            name_slug = kp["name_slug"]
            key_position_path = kp["key_position"]
            reference_path = kp["reference"]
            constrain_x_vertical = kp["constrain_x_vertical"]

            if args.save_tag_reference:
                T_world_keypos_tcp = pose_vec_to_transform(
                    key_position_pose(key_position_path)
                )
                T_board_keypos_tcp = save_tag_reference_json(
                    reference_path,
                    board_cfg=board_cfg,
                    calibration_path=calibration_path,
                    key_position_path=key_position_path,
                    capture_tcp_pose_world=tcp_pose_world,
                    T_world_board=T_world_board,
                    T_world_keypos_tcp=T_world_keypos_tcp,
                    constrain_x_vertical=constrain_x_vertical,
                )
                print(
                    f"Saved board-to-{name_slug} TCP reference JSON: {reference_path}"
                )
                print(
                    f"Board->{name_slug} TCP offset (meters): "
                    f"xyz=[{T_board_keypos_tcp[0, 3]:.4f}, "
                    f"{T_board_keypos_tcp[1, 3]:.4f}, "
                    f"{T_board_keypos_tcp[2, 3]:.4f}]"
                )

            if args.apply_tag_reference:
                T_board_keypos_tcp = load_tag_reference_transform(reference_path)
                T_world_keypos_tcp = T_world_board @ T_board_keypos_tcp
                if constrain_x_vertical:
                    T_world_keypos_tcp = constrain_vise_tcp_to_tag_axes(
                        T_world_keypos_tcp,
                        T_world_board,
                    )
                keypos_pose_world = transform_to_pose_record(T_world_keypos_tcp)
                print(
                    f"Predicted {name_slug} TCP pose (world): "
                    f"{keypos_pose_world['values']}"
                )
                if predicted_base is not None:
                    if emit_multi:
                        out_path = predicted_base.with_name(
                            f"{predicted_base.stem}_{name_slug}{predicted_base.suffix}"
                        )
                    else:
                        out_path = predicted_base
                    save_predicted_keypos_pose_json(
                        out_path,
                        reference_path=reference_path,
                        board_cfg=board_cfg,
                        calibration_path=calibration_path,
                        capture_tcp_pose_world=tcp_pose_world,
                        T_world_board=T_world_board,
                        T_world_keypos_tcp=T_world_keypos_tcp,
                    )
                    print(f"Saved predicted {name_slug} TCP JSON: {out_path}")

    if args.show:
        cv2.imshow("Tag Axes Overlay", annotated)
        print("Press any key in the image window to close.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
