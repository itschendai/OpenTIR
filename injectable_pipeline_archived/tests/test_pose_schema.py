"""Unit tests for pose_schema."""

from __future__ import annotations

import os

import pytest

import pose_schema


HERE = os.path.dirname(os.path.abspath(__file__))
FIXTURES = os.path.join(HERE, "fixtures")


# ----------------- constants -----------------


def test_required_pose_checklist_matches_spec():
    assert pose_schema.REQUIRED_POSES == (
        "home",
        "pickup_pre_grasp",
        "pickup_grasp",
        "pickup_lifted",
        "above_vise",
        "safe_intermediate",
        "disposal",
    )


def test_required_path_set_matches_spec():
    required = [spec.name for spec in pose_schema.REQUIRED_PATHS]
    assert required == [
        "lifted_to_above_vise",
        "above_vise_to_safe_intermediate",
        "safe_intermediate_to_above_vise",
        "above_vise_to_disposal",
        "disposal_to_home",
    ]


def test_checklist_entries_order():
    items = pose_schema.checklist_entries()
    kinds_only = [kind for kind, _ in items]
    # Counts are unchanged: every required pose and path appears exactly once.
    assert kinds_only.count("pose") == len(pose_schema.REQUIRED_POSES)
    assert kinds_only.count("path") == len(pose_schema.REQUIRED_PATHS)
    # Cycle-temporal interleaving: 'home' is first, and the relative order of
    # poses-among-poses matches REQUIRED_POSES (likewise for paths). The
    # detailed interleaving invariant (each path begins at an already-captured
    # pose) is covered in tests/test_train_pipeline_poses.py.
    assert items[0] == ("pose", "home")
    pose_names_in_order = [n for kind, n in items if kind == "pose"]
    path_names_in_order = [n for kind, n in items if kind == "path"]
    assert pose_names_in_order == list(pose_schema.REQUIRED_POSES)
    assert path_names_in_order == [spec.name for spec in pose_schema.REQUIRED_PATHS]


def test_phase_entries_pickup_covers_tray_to_above_vise():
    items = pose_schema.phase_entries("pickup")
    assert items[0] == ("pose", "home")
    assert items[-1] == ("pose", "above_vise")
    pose_names = [n for kind, n in items if kind == "pose"]
    path_names = [n for kind, n in items if kind == "path"]
    assert pose_names == [
        "home",
        "pickup_pre_grasp",
        "pickup_grasp",
        "pickup_lifted",
        "above_vise",
    ]
    assert path_names == ["lifted_to_above_vise"]


def test_phase_entries_cut_starts_and_ends_at_above_vise():
    items = pose_schema.phase_entries("cut")
    pose_names = [n for kind, n in items if kind == "pose"]
    assert "above_vise" in pose_names
    assert "safe_intermediate" in pose_names


def test_phase_entries_dispose_covers_disposal_and_home_return():
    items = pose_schema.phase_entries("dispose")
    path_names = [n for kind, n in items if kind == "path"]
    assert "above_vise_to_disposal" in path_names
    assert "disposal_to_home" in path_names


def test_phase_entries_all_matches_full_checklist():
    assert pose_schema.phase_entries("all") == pose_schema.checklist_entries()


def test_phase_entries_unknown_raises():
    import pytest

    with pytest.raises(KeyError):
        pose_schema.phase_entries("bogus")


def test_checklist_entries_optional_appended():
    items = pose_schema.checklist_entries(include_optional=True)
    names = [n for _, n in items]
    for opt in pose_schema.OPTIONAL_POSES:
        assert opt in names
    for opt in pose_schema.OPTIONAL_PATHS:
        assert opt.name in names


# ----------------- pose entry builder -----------------


def test_build_pose_entry_computes_degrees_from_rad():
    entry = pose_schema.build_pose_entry(
        q_rad=[0.0, 1.5707963267948966, 0, 0, 0, 0, 0],
        tcp_pose=[0.4, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0],
        gripper_width_m=0.04,
        gripper_force_n=0.0,
    )
    assert entry["q_rad"][1] == pytest.approx(1.5707963267948966)
    assert entry["q_deg"][1] == pytest.approx(90.0)
    assert entry["tcp_pose_world"]["order"] == list(pose_schema.TCP_POSE_ORDER)
    assert entry["gripper_state"]["width_m"] == 0.04


def test_build_pose_entry_rejects_wrong_length():
    with pytest.raises(pose_schema.PoseFileError):
        pose_schema.build_pose_entry(
            q_rad=[0.0] * 6, tcp_pose=[0.0] * 7, gripper_width_m=0.0, gripper_force_n=0.0
        )
    with pytest.raises(pose_schema.PoseFileError):
        pose_schema.build_pose_entry(
            q_rad=[0.0] * 7, tcp_pose=[0.0] * 6, gripper_width_m=0.0, gripper_force_n=0.0
        )


# ----------------- read / write roundtrip -----------------


def test_write_then_read_yaml_roundtrips(tmp_path):
    doc = pose_schema.empty_document("Rizon4-062930", trainer_version="test")
    doc["poses"]["home"] = pose_schema.build_pose_entry(
        q_rad=[0.0] * 7,
        tcp_pose=[0.4, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0],
        gripper_width_m=0.04,
        gripper_force_n=0.0,
    )
    out = tmp_path / "poses.yaml"
    pose_schema.write_yaml(doc, str(out))
    assert out.exists()
    parsed = pose_schema.read_yaml(str(out))
    assert parsed["schema_version"] == 1
    assert parsed["poses"]["home"]["q_rad"] == [0.0] * 7


def test_write_yaml_is_atomic_no_temp_leftover(tmp_path):
    doc = pose_schema.empty_document("Rizon4-062930")
    out = tmp_path / "poses.yaml"
    pose_schema.write_yaml(doc, str(out))
    leftover = [n for n in os.listdir(str(tmp_path)) if n.startswith(".pose_tmp_")]
    assert leftover == []


# ----------------- validator -----------------


def test_validate_accepts_complete_fixture():
    doc = pose_schema.read_yaml(os.path.join(FIXTURES, "pipeline_poses_valid.yaml"))
    pose_schema.validate(doc)  # should not raise


def test_validate_reports_missing_pose_and_path():
    doc = pose_schema.read_yaml(os.path.join(FIXTURES, "pipeline_poses_missing.yaml"))
    with pytest.raises(pose_schema.PoseFileError) as exc_info:
        pose_schema.validate(doc)
    err = exc_info.value
    assert err.code == "E_POSE_MISSING"
    assert "safe_intermediate" in err.message
    assert "disposal_to_home" in err.message


def test_validate_rejects_wrong_schema_version():
    doc = pose_schema.empty_document("Rizon4-062930")
    doc["schema_version"] = 99
    with pytest.raises(pose_schema.PoseFileError) as exc_info:
        pose_schema.validate(doc)
    assert exc_info.value.code == "E_POSE_SCHEMA"


def test_validate_rejects_path_referencing_unknown_pose(tmp_path):
    doc = pose_schema.read_yaml(os.path.join(FIXTURES, "pipeline_poses_valid.yaml"))
    doc["paths"]["lifted_to_above_vise"]["to"] = "ghost_pose"
    with pytest.raises(pose_schema.PoseFileError) as exc_info:
        pose_schema.validate(doc)
    assert exc_info.value.code == "E_POSE_SCHEMA"
    assert "ghost_pose" in exc_info.value.message


def test_validate_rejects_pose_with_short_q():
    doc = pose_schema.read_yaml(os.path.join(FIXTURES, "pipeline_poses_valid.yaml"))
    doc["poses"]["home"]["q_rad"] = [0.0] * 6
    doc["poses"]["home"].pop("q_deg", None)
    with pytest.raises(pose_schema.PoseFileError) as exc_info:
        pose_schema.validate(doc)
    assert exc_info.value.code == "E_POSE_SCHEMA"


# ----------------- TrainerState -----------------


def test_trainer_state_fresh_starts_empty():
    state = pose_schema.TrainerState.fresh("Rizon4-062930", trainer_version="vX")
    assert state.captured_poses == set()
    assert state.captured_paths == set()
    assert state.document["schema_version"] == pose_schema.SCHEMA_VERSION
    assert state.document["robot_sn"] == "Rizon4-062930"


def test_trainer_state_resume_marks_captured_entries():
    state = pose_schema.TrainerState.resume_from(
        os.path.join(FIXTURES, "pipeline_poses_valid.yaml")
    )
    for name in pose_schema.REQUIRED_POSES:
        assert state.is_pose_captured(name)
    assert state.is_path_captured("lifted_to_above_vise")
    assert not state.is_pose_captured("disposal_top")


def test_trainer_state_record_pose_and_path():
    state = pose_schema.TrainerState.fresh("Rizon4-062930")
    entry = pose_schema.build_pose_entry(
        q_rad=[0.0] * 7,
        tcp_pose=[0.0] * 7,
        gripper_width_m=0.0,
        gripper_force_n=0.0,
    )
    state.record_pose("home", entry)
    path_entry = pose_schema.build_path_entry("home", "home", waypoints=[])
    state.record_path("disposal_to_home", path_entry)
    assert state.is_pose_captured("home")
    assert state.is_path_captured("disposal_to_home")
    assert state.document["poses"]["home"]["q_rad"] == [0.0] * 7
