"""Shared YAML pose-file schema for the trainer and orchestrator.

See ``planning/specifications.md`` §5 for the canonical schema. This module owns:

- The pose / path checklist that the trainer walks and the orchestrator validates.
- Helpers to build, write, and re-read the YAML file.
- The validator that raises ``PoseFileError`` on missing or malformed entries.

The trainer and orchestrator both import from here so the contract stays in one
place — a divergence between the two becomes a single-edit problem instead of two.
"""

from __future__ import annotations

import datetime as _dt
import math
import os
import tempfile
from dataclasses import dataclass, field
from typing import Iterable

try:
    import yaml  # PyYAML
except ImportError as exc:  # pragma: no cover
    raise ImportError("pyyaml is required: pip install pyyaml") from exc


SCHEMA_VERSION = 1


# Required pose names — order = trainer checklist order.
REQUIRED_POSES: tuple[str, ...] = (
    "home",
    "pickup_pre_grasp",
    "pickup_grasp",
    "pickup_lifted",
    "above_vise",
    "safe_intermediate",
    "disposal",
)

# Optional per-component disposal poses. If captured, the matching path may also be
# captured.
OPTIONAL_POSES: tuple[str, ...] = (
    "disposal_top",
    "disposal_spring",
    "disposal_body",
)


@dataclass(frozen=True)
class PathSpec:
    name: str
    from_pose: str
    to_pose: str


REQUIRED_PATHS: tuple[PathSpec, ...] = (
    PathSpec("lifted_to_above_vise", "pickup_lifted", "above_vise"),
    PathSpec("above_vise_to_safe_intermediate", "above_vise", "safe_intermediate"),
    PathSpec("safe_intermediate_to_above_vise", "safe_intermediate", "above_vise"),
    PathSpec("above_vise_to_disposal", "above_vise", "disposal"),
    PathSpec("disposal_to_home", "disposal", "home"),
)

OPTIONAL_PATHS: tuple[PathSpec, ...] = (
    PathSpec("above_vise_to_disposal_top", "above_vise", "disposal_top"),
    PathSpec("above_vise_to_disposal_spring", "above_vise", "disposal_spring"),
    PathSpec("above_vise_to_disposal_body", "above_vise", "disposal_body"),
)


TCP_POSE_ORDER: tuple[str, ...] = ("x", "y", "z", "qw", "qx", "qy", "qz")


class PoseFileError(Exception):
    """Raised when the pose YAML fails schema validation."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(f"{code}: {message}")
        self.code = code
        self.message = message


# ---------- low-level pose / path entry builders ----------


def _to_floats(values: Iterable[float]) -> list[float]:
    return [float(v) for v in values]


def build_pose_entry(
    q_rad: Iterable[float],
    tcp_pose: Iterable[float],
    gripper_width_m: float,
    gripper_force_n: float,
) -> dict:
    """Construct a single pose entry. Validates length up front."""
    q_rad_list = _to_floats(q_rad)
    tcp_list = _to_floats(tcp_pose)
    if len(q_rad_list) != 7:
        raise PoseFileError("E_POSE_SCHEMA", f"q_rad must have 7 values, got {len(q_rad_list)}")
    if len(tcp_list) != 7:
        raise PoseFileError("E_POSE_SCHEMA", f"tcp_pose must have 7 values, got {len(tcp_list)}")
    return {
        "q_rad": q_rad_list,
        "q_deg": [math.degrees(v) for v in q_rad_list],
        "tcp_pose_world": {
            "order": list(TCP_POSE_ORDER),
            "values": tcp_list,
        },
        "gripper_state": {
            "width_m": float(gripper_width_m),
            "force_n": float(gripper_force_n),
        },
    }


def build_waypoint_entry(
    name: str,
    q_rad: Iterable[float],
    tcp_pose: Iterable[float],
    gripper_width_m: float,
    gripper_force_n: float,
) -> dict:
    entry = build_pose_entry(q_rad, tcp_pose, gripper_width_m, gripper_force_n)
    entry["name"] = name
    return entry


def build_path_entry(
    from_pose: str, to_pose: str, waypoints: list[dict] | None = None
) -> dict:
    return {
        "from": from_pose,
        "to": to_pose,
        "waypoints": list(waypoints or []),
    }


def empty_document(
    robot_sn: str,
    trainer_version: str = "",
    captured_at: str | None = None,
) -> dict:
    return {
        "schema_version": SCHEMA_VERSION,
        "trainer_version": trainer_version,
        "robot_sn": robot_sn,
        "captured_at": captured_at or _dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "poses": {},
        "paths": {},
    }


# ---------- file I/O ----------


def write_yaml(payload: dict, path: str) -> None:
    """Atomically write the document — temp file in the same dir, then rename."""
    directory = os.path.dirname(os.path.abspath(path)) or "."
    os.makedirs(directory, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(prefix=".pose_tmp_", dir=directory, suffix=".yaml")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            yaml.safe_dump(
                payload, handle, sort_keys=False, default_flow_style=False, allow_unicode=True
            )
        os.replace(tmp_path, path)
    except Exception:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass
        raise


def read_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if data is None:
        raise PoseFileError("E_POSE_SCHEMA", f"{path}: file is empty")
    if not isinstance(data, dict):
        raise PoseFileError("E_POSE_SCHEMA", f"{path}: top-level must be a mapping")
    data.setdefault("poses", {})
    data.setdefault("paths", {})
    return data


# ---------- validation ----------


def _is_seven_floats(value) -> bool:
    if not isinstance(value, list) or len(value) != 7:
        return False
    return all(isinstance(v, (int, float)) for v in value)


def _validate_pose_entry(name: str, entry, *, missing: list[str], schema: list[str]) -> None:
    if not isinstance(entry, dict):
        schema.append(f"pose {name!r}: must be a mapping")
        return
    q = entry.get("q_rad") or entry.get("q_deg")
    if q is None:
        missing.append(f"pose {name!r}: q_rad or q_deg")
    elif not _is_seven_floats(q):
        schema.append(f"pose {name!r}: q must be 7 numeric values")
    tcp = (entry.get("tcp_pose_world") or {}).get("values")
    if tcp is None:
        missing.append(f"pose {name!r}: tcp_pose_world.values")
    elif not _is_seven_floats(tcp):
        schema.append(f"pose {name!r}: tcp_pose_world.values must be 7 numeric values")


def _validate_path_entry(
    name: str, entry, *, poses: set, missing: list[str], schema: list[str]
) -> None:
    if not isinstance(entry, dict):
        schema.append(f"path {name!r}: must be a mapping")
        return
    src = entry.get("from")
    dst = entry.get("to")
    if src is None or dst is None:
        missing.append(f"path {name!r}: from/to")
        return
    if src not in poses:
        schema.append(f"path {name!r}: from={src!r} not in poses")
    if dst not in poses:
        schema.append(f"path {name!r}: to={dst!r} not in poses")
    waypoints = entry.get("waypoints", [])
    if not isinstance(waypoints, list):
        schema.append(f"path {name!r}: waypoints must be a list")
        return
    for idx, wp in enumerate(waypoints):
        if not isinstance(wp, dict):
            schema.append(f"path {name!r} waypoint {idx}: must be a mapping")
            continue
        q = wp.get("q_rad") or wp.get("q_deg")
        tcp = (wp.get("tcp_pose_world") or {}).get("values")
        if q is None or not _is_seven_floats(q):
            schema.append(f"path {name!r} waypoint {idx}: q must be 7 numeric values")
        if tcp is None or not _is_seven_floats(tcp):
            schema.append(
                f"path {name!r} waypoint {idx}: tcp_pose_world.values must be 7 numeric values"
            )


def validate(document: dict) -> None:
    """Raise ``PoseFileError`` if the document fails schema validation."""
    missing: list[str] = []
    schema: list[str] = []

    version = document.get("schema_version")
    if version != SCHEMA_VERSION:
        raise PoseFileError(
            "E_POSE_SCHEMA",
            f"schema_version must be {SCHEMA_VERSION}, got {version!r}",
        )

    poses = document.get("poses") or {}
    paths = document.get("paths") or {}
    if not isinstance(poses, dict):
        raise PoseFileError("E_POSE_SCHEMA", "poses must be a mapping")
    if not isinstance(paths, dict):
        raise PoseFileError("E_POSE_SCHEMA", "paths must be a mapping")

    for required in REQUIRED_POSES:
        if required not in poses:
            missing.append(f"pose {required!r}")
        else:
            _validate_pose_entry(required, poses[required], missing=missing, schema=schema)
    for optional in OPTIONAL_POSES:
        if optional in poses:
            _validate_pose_entry(optional, poses[optional], missing=missing, schema=schema)

    pose_names = set(poses.keys())

    for required_path in REQUIRED_PATHS:
        if required_path.name not in paths:
            missing.append(f"path {required_path.name!r}")
        else:
            _validate_path_entry(
                required_path.name,
                paths[required_path.name],
                poses=pose_names,
                missing=missing,
                schema=schema,
            )
    for optional_path in OPTIONAL_PATHS:
        if optional_path.name in paths:
            _validate_path_entry(
                optional_path.name,
                paths[optional_path.name],
                poses=pose_names,
                missing=missing,
                schema=schema,
            )

    if missing:
        raise PoseFileError(
            "E_POSE_MISSING",
            "missing required entries: " + ", ".join(missing),
        )
    if schema:
        raise PoseFileError(
            "E_POSE_SCHEMA",
            "schema errors: " + "; ".join(schema),
        )


# ---------- partial / resume helpers ----------


@dataclass
class TrainerState:
    """Mutable container the trainer uses while walking the checklist."""

    document: dict
    captured_poses: set = field(default_factory=set)
    captured_paths: set = field(default_factory=set)

    @classmethod
    def fresh(cls, robot_sn: str, trainer_version: str = "") -> "TrainerState":
        return cls(document=empty_document(robot_sn, trainer_version=trainer_version))

    @classmethod
    def resume_from(cls, path: str) -> "TrainerState":
        document = read_yaml(path)
        document.setdefault("schema_version", SCHEMA_VERSION)
        return cls(
            document=document,
            captured_poses=set((document.get("poses") or {}).keys()),
            captured_paths=set((document.get("paths") or {}).keys()),
        )

    def record_pose(self, name: str, entry: dict) -> None:
        self.document.setdefault("poses", {})[name] = entry
        self.captured_poses.add(name)

    def record_path(self, name: str, entry: dict) -> None:
        self.document.setdefault("paths", {})[name] = entry
        self.captured_paths.add(name)

    def is_pose_captured(self, name: str) -> bool:
        return name in self.captured_poses

    def is_path_captured(self, name: str) -> bool:
        return name in self.captured_paths


def checklist_entries(
    include_optional: bool = False,
) -> list[tuple[str, str]]:
    """Return the trainer's ordered checklist as (kind, name) tuples.

    Kind is ``"pose"`` or ``"path"``. Order is **cycle-temporal**: each path
    appears between its from-pose and its to-pose so the operator can walk the
    whole cycle in one pass instead of capturing poses then revisiting them
    for paths. The relative order of poses among poses still matches
    ``REQUIRED_POSES`` and paths among paths still matches ``REQUIRED_PATHS``.
    """
    items: list[tuple[str, str]] = [
        ("pose", "home"),
        ("pose", "pickup_pre_grasp"),
        ("pose", "pickup_grasp"),
        ("pose", "pickup_lifted"),
        ("path", "lifted_to_above_vise"),
        ("pose", "above_vise"),
        ("path", "above_vise_to_safe_intermediate"),
        ("pose", "safe_intermediate"),
        ("path", "safe_intermediate_to_above_vise"),
        ("path", "above_vise_to_disposal"),
        ("pose", "disposal"),
        ("path", "disposal_to_home"),
    ]
    if include_optional:
        for name in OPTIONAL_POSES:
            items.append(("pose", name))
        for spec in OPTIONAL_PATHS:
            items.append(("path", spec.name))
    return items


def path_spec_by_name(name: str) -> PathSpec:
    for spec in REQUIRED_PATHS + OPTIONAL_PATHS:
        if spec.name == name:
            return spec
    raise KeyError(name)


# Phase-scoped subsets of the walk-through. ``"all"`` returns the full cycle in
# cycle-temporal order (same as ``checklist_entries()``). The phase names mirror
# the spec's cycle phases:
#   - "pickup": Phase 1 (PICKUP) + Phase 2 (LOAD) — tray to vise.
#   - "cut":    Phase 3 (CUT)  — parking and the cut itself.
#   - "dispose": Phases 4–6 (REMOVE_TOP/SPRING/BODY) — vise to disposal to home.
# Each subset re-includes shared anchor poses (e.g. ``above_vise``) so the
# operator can start a phase cold; ``--resume`` skips ones already captured.
_PHASE_ENTRIES: dict[str, list[tuple[str, str]]] = {
    "pickup": [
        ("pose", "home"),
        ("pose", "pickup_pre_grasp"),
        ("pose", "pickup_grasp"),
        ("pose", "pickup_lifted"),
        ("path", "lifted_to_above_vise"),
        ("pose", "above_vise"),
    ],
    "cut": [
        ("pose", "above_vise"),
        ("path", "above_vise_to_safe_intermediate"),
        ("pose", "safe_intermediate"),
        ("path", "safe_intermediate_to_above_vise"),
    ],
    "dispose": [
        ("pose", "above_vise"),
        ("path", "above_vise_to_disposal"),
        ("pose", "disposal"),
        ("path", "disposal_to_home"),
    ],
}

PHASE_NAMES: tuple[str, ...] = ("pickup", "cut", "dispose", "all")


def phase_entries(
    phase: str, include_optional: bool = False
) -> list[tuple[str, str]]:
    """Return the walk-through entries for ``phase``.

    ``phase`` must be one of ``PHASE_NAMES``. ``"all"`` returns the full
    cycle-temporal walk-through (identical to :func:`checklist_entries`).
    """
    if phase == "all":
        return checklist_entries(include_optional=include_optional)
    if phase not in _PHASE_ENTRIES:
        raise KeyError(f"unknown phase {phase!r}; expected one of {PHASE_NAMES}")
    return list(_PHASE_ENTRIES[phase])
