import importlib.util
import math
from pathlib import Path
import sys

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.parameter import Parameter
import pytest
import rclpy


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPOSITORY_ROOT / "src/agt_ui_bridge"))
SCRIPT = PACKAGE_ROOT / "scripts/coverage_time_simulator.py"
SPEC = importlib.util.spec_from_file_location("coverage_time_simulator", SCRIPT)
SIMULATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SIMULATOR)
PROFILE = REPOSITORY_ROOT / "profiles/platforms/bunker.yaml"


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _path():
    message = NavPath()
    message.header.frame_id = "map"
    for x, yaw in ((0.0, 0.0), (2.0, 0.0), (1.0, 0.0)):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        message.poses.append(pose)
    return message


def test_profile_limits_are_loaded_from_canonical_platform_file():
    limits = SIMULATOR._load_motion_limits(PROFILE)

    assert limits.max_forward_velocity == pytest.approx(0.5)
    assert limits.max_reverse_velocity == pytest.approx(0.25)
    assert limits.max_angular_velocity == pytest.approx(0.6)


def test_node_publishes_fail_closed_geometric_estimate_without_semantics():
    node = SIMULATOR.CoverageTimeSimulator(
        parameter_overrides=[Parameter("platform_profile", value=str(PROFILE))]
    )
    try:
        node._path_callback(_path())
        assert node.last_report["status"] == "ESTIMATED"
        assert node.last_report["classification_source"] == "geometric_fallback"
        assert node.last_report["semantic_classification_error"] == (
            "path_semantics_unavailable"
        )
        assert node.last_report["direction_change_count"] == 1
        topics = dict(node.get_topic_names_and_types())
        assert topics["/agt/coverage/simulation_report"] == ["std_msgs/msg/String"]
    finally:
        node.destroy_node()
