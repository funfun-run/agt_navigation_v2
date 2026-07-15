import importlib.util
import os
from pathlib import Path

from launch import LaunchContext
import pytest


ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("ROS_LOG_DIR", "/tmp/agt_bringup_test_logs")
SYSTEM_PATH = ROOT / "src/agt_bringup/launch/system.launch.py"
NAVIGATION_PATH = ROOT / "src/agt_bringup/launch/navigation_system.launch.py"
BAG_PATH = ROOT / "src/agt_bringup/launch/bag_record.launch.py"


def _load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(**values):
    context = LaunchContext()
    context.launch_configurations.update(
        {name: str(value) for name, value in values.items()}
    )
    return context


def _navigation_values(tmp_path):
    map_path = tmp_path / "map.yaml"
    pcd_path = tmp_path / "map.pcd"
    map_path.write_text("image: map.pgm\n", encoding="utf-8")
    pcd_path.write_text("VERSION 0.7\n", encoding="utf-8")
    return {
        "mode": "navigation",
        "map": map_path,
        "global_map_pcd": pcd_path,
        "start_semantic_map_server": "false",
        "start_coverage_planning": "false",
        "annotation_mode": "false",
    }


def test_original_navigation_remains_valid_when_coverage_is_disabled(tmp_path):
    system = _load_module("agt_system_launch", SYSTEM_PATH)
    values = _navigation_values(tmp_path)

    assert system.validate_mode_arguments(_context(**values)) == []


def test_coverage_cannot_start_without_semantic_server(tmp_path):
    system = _load_module("agt_system_launch_requires_semantics", SYSTEM_PATH)
    values = _navigation_values(tmp_path)
    values["start_coverage_planning"] = "true"

    with pytest.raises(RuntimeError, match="requires start_semantic_map_server"):
        system.validate_mode_arguments(_context(**values))


def test_semantic_coverage_paths_are_validated_before_children_start(tmp_path):
    system = _load_module("agt_system_launch_valid_semantics", SYSTEM_PATH)
    values = _navigation_values(tmp_path)
    semantic_map = tmp_path / "semantic_map.geojson"
    coverage = tmp_path / "coverage.yaml"
    profile = tmp_path / "bunker.yaml"
    for path in (semantic_map, coverage, profile):
        path.write_text("{}\n", encoding="utf-8")
    values.update(
        {
            "start_semantic_map_server": "true",
            "start_coverage_planning": "true",
            "semantic_map": semantic_map,
            "coverage_params": coverage,
            "platform_profile": profile,
        }
    )

    assert system.validate_mode_arguments(_context(**values)) == []

    wrong_coverage = tmp_path / "other.yaml"
    wrong_coverage.write_text("{}\n", encoding="utf-8")
    values["coverage_params"] = wrong_coverage
    with pytest.raises(RuntimeError, match="coverage.yaml beside semantic_map"):
        system.validate_mode_arguments(_context(**values))


def test_direct_navigation_launch_has_the_same_fail_closed_validation(tmp_path):
    navigation = _load_module("agt_navigation_system_launch", NAVIGATION_PATH)
    context = _context(
        start_semantic_map_server="false",
        start_coverage_planning="true",
        annotation_mode="false",
    )

    with pytest.raises(RuntimeError, match="requires start_semantic_map_server"):
        navigation.validate_coverage_arguments(context)


def test_navigation_composes_each_runtime_owner_once():
    source = NAVIGATION_PATH.read_text(encoding="utf-8")
    assert source.count('"navigation.launch.py"') == 1
    assert source.count('"semantic_map_server.launch.py"') == 1
    assert source.count('"coverage_planning.launch.py"') == 1
    assert source.count('"bunker_description.launch.py"') == 1
    assert source.count('"bunker.launch.py"') == 1
    assert '"use_keepout_filter": semantic_enabled' in source
    assert '"execution_enabled": coverage_execution' in source
    assert "ExecuteProcess" not in source


def test_bag_recorder_contains_semantic_coverage_debug_products():
    source = BAG_PATH.read_text(encoding="utf-8")
    for topic in (
        "/agt/map/semantic_markers",
        "/agt/map/keepout_mask",
        "/agt/coverage/path_raw",
        "/agt/coverage/path_validated",
        "/agt/coverage/path_repaired",
        "/agt/coverage/collision_poses",
        "/agt/coverage/status",
        "/agt/coverage/validation_report",
        "/agt/coverage/task_status",
        "/global_costmap/costmap",
        "/local_costmap/costmap",
    ):
        assert f'"{topic}"' in source
