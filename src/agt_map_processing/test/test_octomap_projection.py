import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


def test_launch_files_parse():
    for launch_file in (PACKAGE_ROOT / "launch").glob("*.launch.py"):
        ast.parse(launch_file.read_text(encoding="utf-8"), filename=str(launch_file))


def test_octomap_consumes_lidar_frame_cloud_for_dynamic_sensor_origin():
    launch_source = (
        PACKAGE_ROOT / "launch" / "octomap_projection.launch.py"
    ).read_text(encoding="utf-8")
    assert 'default_value="/agt/mapping/registered_points_lidar"' in launch_source


def test_octomap_projection_uses_v2_frame_contract():
    parameters = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "octomap_projection.yaml").read_text(
            encoding="utf-8"
        )
    )["/**"]["ros__parameters"]

    assert parameters["frame_id"] == "odom"
    assert parameters["base_frame_id"] == "base_footprint"
    assert parameters["resolution"] > 0.0
    assert parameters["pointcloud_min_z"] < parameters["pointcloud_max_z"]
    assert parameters["occupancy_min_z"] < parameters["occupancy_max_z"]


def test_mapping_adapter_output_frame_matches_octomap_frame():
    adapter_config = yaml.safe_load(
        (
            PACKAGE_ROOT.parent
            / "agt_mapping"
            / "config"
            / "fast_livo2_adapter.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    map_config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "octomap_projection.yaml").read_text(
            encoding="utf-8"
        )
    )["/**"]["ros__parameters"]

    assert adapter_config["registered_points_frame"] == map_config["frame_id"]
