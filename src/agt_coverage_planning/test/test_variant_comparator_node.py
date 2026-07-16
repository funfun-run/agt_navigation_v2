import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import rclpy


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPOSITORY_ROOT / "src/agt_ui_bridge"))
SCRIPT = PACKAGE_ROOT / "scripts/coverage_variant_comparator.py"
SPEC = importlib.util.spec_from_file_location("coverage_variant_comparator", SCRIPT)
COMPARATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(COMPARATOR)


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_comparator_exposes_only_visualization_and_report_products():
    node = COMPARATOR.CoverageVariantComparator()
    try:
        topics = dict(node.get_topic_names_and_types())
        assert topics["/agt/coverage/comparison/markers"] == [
            "visualization_msgs/msg/MarkerArray"
        ]
        assert topics["/agt/coverage/comparison/report"] == ["std_msgs/msg/String"]
        assert "/agt/coverage/path_validated" not in topics
        assert "/agt/chassis/cmd_vel" not in topics
    finally:
        node.destroy_node()


def test_goal_contains_variant_route_path_and_angle_without_execution_fields():
    spec = SimpleNamespace(
        generate_headland=True,
        headland_width=1.5,
        swath_objective="LENGTH",
        swath_mode="SET_ANGLE",
        swath_angle=0.25,
        path_continuity_mode="CONTINUOUS",
        polygons=(((0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 0.0)),),
    )
    variant = SimpleNamespace(
        route_mode="SPIRAL",
        path_mode="REEDS_SHEPP",
        swath_angle_offset=0.10,
        spiral_n=3,
    )

    goal = COMPARATOR._build_goal(spec, variant)

    assert goal.route_mode.mode == "SPIRAL"
    assert goal.route_mode.spiral_n == 3
    assert goal.path_mode.mode == "REEDS_SHEPP"
    assert goal.swath_mode.best_angle == pytest.approx(0.35)
    assert goal.generate_path
    assert goal.frame_id == "map"
