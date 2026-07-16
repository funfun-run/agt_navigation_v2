import math
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "profiles/platforms/greenhouse_ackermann.yaml"


def test_greenhouse_ackermann_geometry_and_steering_contract():
    platform = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))["platform"]
    geometry = platform["geometry"]

    assert platform["kinematics"] == "ackermann"
    assert geometry["wheelbase"] == pytest.approx(0.600)
    assert geometry["wheel_track"] == pytest.approx(0.517)
    assert geometry["length"] == pytest.approx(0.840)
    assert geometry["width"] == pytest.approx(0.600)
    assert geometry["wheel_radius"] == pytest.approx(0.120)
    assert geometry["min_turning_radius"] == pytest.approx(1.500)
    assert geometry["steering"]["max_equivalent_steering_angle"] == pytest.approx(
        math.atan(0.600 / 1.500)
    )
    assert platform["coverage_repair"] == {
        "enabled": True,
        "planner_id": "CoverageRepairHybrid",
        "planner_family": "hybrid_a_star",
        "allow_in_place_rotation": False,
    }


def test_greenhouse_ackermann_footprint_has_no_hidden_margin():
    geometry = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))["platform"][
        "geometry"
    ]
    footprint = geometry["navigation_footprint"]
    width = max(point[1] for point in footprint) - min(point[1] for point in footprint)
    length = max(point[0] for point in footprint) - min(point[0] for point in footprint)

    assert width == pytest.approx(geometry["width"])
    assert length == pytest.approx(geometry["length"])
