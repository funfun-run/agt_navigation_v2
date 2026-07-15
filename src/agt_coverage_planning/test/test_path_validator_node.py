import importlib.util
import json
from pathlib import Path
import sys

from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as NavPath
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPOSITORY_ROOT / "src/agt_ui_bridge"))
SCRIPT = PACKAGE_ROOT / "scripts/coverage_path_validator.py"
SPEC = importlib.util.spec_from_file_location("coverage_path_validator", SCRIPT)
VALIDATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATOR)

from agt_ui_bridge.platform_profile import load_platform_profile  # noqa: E402
from agt_coverage_planning.path_semantics import (  # noqa: E402
    Pose2D,
    SwathInput,
    build_path_semantics,
)


PROFILE_PATH = REPOSITORY_ROOT / "profiles/platforms/bunker.yaml"
PLATFORM = load_platform_profile(PROFILE_PATH)


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _parameters(**overrides):
    values = {"platform_profile": str(PROFILE_PATH), **overrides}
    return [Parameter(name, value=value) for name, value in values.items()]


def _path(start=(2.0, 2.0), end=(5.0, 2.0)):
    message = NavPath()
    message.header.frame_id = "map"
    for x, y in (start, end):
        stamped = PoseStamped()
        stamped.header.frame_id = "map"
        stamped.pose.position.x = x
        stamped.pose.position.y = y
        stamped.pose.orientation.w = 1.0
        message.poses.append(stamped)
    return message


def _costmap(obstacle=None):
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.width = 100
    message.info.height = 60
    message.info.resolution = 0.1
    message.info.origin.orientation.w = 1.0
    message.data = [0] * (message.info.width * message.info.height)
    if obstacle is not None:
        column, row, cost = obstacle
        message.data[row * message.info.width + column] = cost
    return message


def _published_footprint(scale=1.0):
    message = PolygonStamped()
    message.header.frame_id = "map"
    for x, y in PLATFORM["footprint"]:
        point = Point32()
        point.x = 2.0 + x * scale
        point.y = 2.0 + y * scale
        message.polygon.points.append(point)
    return message


def _semantic_message(path):
    poses = [
        Pose2D(stamped.pose.position.x, stamped.pose.position.y, 0.0)
        for stamped in path.poses
    ]
    semantics = build_path_semantics(
        poses,
        [SwathInput(poses[0], poses[-1])],
        [],
    )
    message = String()
    message.data = semantics.to_json()
    return message


def _feed(node, path=None, costmap=None, footprint=None, semantics=None):
    active_path = path or _path()
    node._path_callback(active_path)
    node._semantics_callback(semantics or _semantic_message(active_path))
    node._costmap_callback(costmap or _costmap())
    node._footprint_callback(footprint or _published_footprint())
    node._validate_if_ready()
    return json.loads(node.last_report_json)


def test_node_publishes_validated_path_and_required_interfaces():
    node = VALIDATOR.CoveragePathValidator(parameter_overrides=_parameters())
    try:
        report = _feed(node)
        assert report["valid"]
        assert report["swath_ids"] == ["swath_0001"]
        assert len(node.last_validated_path.poses) == 2
        assert node.last_collision_poses.poses == []
        topics = dict(node.get_topic_names_and_types())
        assert topics["/agt/coverage/path_validated"] == ["nav_msgs/msg/Path"]
        assert topics["/agt/coverage/collision_poses"] == [
            "geometry_msgs/msg/PoseArray"
        ]
        assert topics["/agt/coverage/footprint_markers"] == [
            "visualization_msgs/msg/MarkerArray"
        ]
        assert topics["/agt/coverage/validation_report"] == ["std_msgs/msg/String"]
        qos = node.get_publishers_info_by_topic(
            "/agt/coverage/path_validated"
        )[0].qos_profile
        assert qos.durability == VALIDATOR.DurabilityPolicy.TRANSIENT_LOCAL
    finally:
        node.destroy_node()


def test_node_clears_validated_path_and_publishes_collisions_on_failure():
    node = VALIDATOR.CoveragePathValidator(parameter_overrides=_parameters())
    try:
        report = _feed(node, costmap=_costmap((35, 20, 100)))
        assert not report["valid"]
        assert report["collision_pose_count"] > 0
        assert report["invalid_swath_ids"] == ["swath_0001"]
        assert node.last_validated_path.poses == []
        assert len(node.last_collision_poses.poses) > 0
        assert len(node.last_footprint_markers.markers) > 1
    finally:
        node.destroy_node()


def test_node_rejects_runtime_footprint_profile_mismatch():
    node = VALIDATOR.CoveragePathValidator(parameter_overrides=_parameters())
    try:
        report = _feed(node, footprint=_published_footprint(scale=1.5))
        assert not report["valid"]
        assert report["error_codes"] == [
            "published_footprint_profile_mismatch"
        ]
        assert node.last_validated_path.poses == []
    finally:
        node.destroy_node()


def test_costmap_update_revalidates_and_invalidates_previous_path():
    node = VALIDATOR.CoveragePathValidator(parameter_overrides=_parameters())
    try:
        assert _feed(node)["valid"]
        node._costmap_callback(_costmap((35, 20, 100)))
        node._validate_if_ready()
        report = json.loads(node.last_report_json)
        assert not report["valid"]
        assert node.last_validated_path.poses == []
    finally:
        node.destroy_node()


def test_node_rejects_semantics_from_a_different_path():
    node = VALIDATOR.CoveragePathValidator(parameter_overrides=_parameters())
    try:
        semantics = _semantic_message(_path(end=(6.0, 2.0)))
        report = _feed(node, semantics=semantics)
        assert not report["valid"]
        assert report["error_codes"] == [
            "path_semantics_fingerprint_mismatch"
        ]
        assert node.last_validated_path.poses == []
    finally:
        node.destroy_node()
