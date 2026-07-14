from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
PROFILE_PATH = ROOT / "profiles/platforms/bunker.yaml"
NAV2_CONFIG_PATH = ROOT / "src/agt_navigation/config/nav2_bunker.yaml"
OBSTACLE_CONFIG_PATH = (
    ROOT / "src/agt_perception/config/local_obstacle_filter.yaml"
)


def _load_yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _canonical_footprint():
    profile = _load_yaml(PROFILE_PATH)["platform"]
    return profile, profile["geometry"]["navigation_footprint"]


def _bounds(points):
    return {
        "min_x": min(point[0] for point in points),
        "max_x": max(point[0] for point in points),
        "min_y": min(point[1] for point in points),
        "max_y": max(point[1] for point in points),
    }


def test_bunker_profile_is_the_canonical_navigation_geometry():
    profile, navigation_footprint = _canonical_footprint()
    geometry = profile["geometry"]

    assert profile["footprint_frame"] == "base_footprint"
    assert geometry["outer_dimensions_verified"] is True
    assert len(navigation_footprint) == 4

    physical = _bounds(geometry["footprint"])
    navigation = _bounds(navigation_footprint)
    assert navigation["min_x"] == pytest.approx(physical["min_x"] - 0.08)
    assert navigation["max_x"] == pytest.approx(physical["max_x"] + 0.08)
    assert navigation["min_y"] == pytest.approx(physical["min_y"] - 0.08)
    assert navigation["max_y"] == pytest.approx(physical["max_y"] + 0.08)


def test_nav2_costmaps_match_the_platform_navigation_footprint():
    _, expected = _canonical_footprint()
    nav2 = _load_yaml(NAV2_CONFIG_PATH)

    for costmap_name in ("local_costmap", "global_costmap"):
        parameters = nav2[costmap_name][costmap_name]["ros__parameters"]
        configured = yaml.safe_load(parameters["footprint"])
        assert configured == expected, costmap_name


def test_obstacle_filter_crop_matches_the_platform_navigation_footprint():
    profile, footprint = _canonical_footprint()
    parameters = _load_yaml(OBSTACLE_CONFIG_PATH)[
        "agt_local_obstacle_filter"
    ]["ros__parameters"]
    expected = _bounds(footprint)

    assert parameters["target_frame"] == profile["footprint_frame"]
    assert parameters["robot_min_x"] == pytest.approx(expected["min_x"])
    assert parameters["robot_max_x"] == pytest.approx(expected["max_x"])
    assert parameters["robot_min_y"] == pytest.approx(expected["min_y"])
    assert parameters["robot_max_y"] == pytest.approx(expected["max_y"])
