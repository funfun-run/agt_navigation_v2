import importlib.util
import json
from pathlib import Path
import sys
from threading import Thread
import time

from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Point32, PolygonStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPOSITORY_ROOT / "src/agt_ui_bridge"))
SCRIPT = PACKAGE_ROOT / "scripts/coverage_path_repair.py"
SPEC = importlib.util.spec_from_file_location("coverage_path_repair", SCRIPT)
REPAIR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPAIR)

from agt_coverage_planning.path_semantics import (  # noqa: E402
    Pose2D,
    SwathInput,
    TurnInput,
    build_path_semantics,
)
from agt_ui_bridge.platform_profile import load_platform_profile  # noqa: E402


PROFILE_PATH = REPOSITORY_ROOT / "profiles/platforms/bunker.yaml"
PLATFORM = load_platform_profile(PROFILE_PATH)


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _parameters(**overrides):
    values = {
        "platform_profile": str(PROFILE_PATH),
        "planner_action": "/agt/test/unavailable_compute_path",
        **overrides,
    }
    return [Parameter(name, value=value) for name, value in values.items()]


def _products():
    swaths = [
        SwathInput(Pose2D(2, 2, 0), Pose2D(5, 2, 0)),
        SwathInput(Pose2D(5, 4, 3.14), Pose2D(2, 4, 3.14)),
    ]
    turns = [
        TurnInput(
            (Pose2D(5, 2, 0), Pose2D(6, 3, 1.57), Pose2D(5, 4, 3.14))
        )
    ]
    raw = [
        Pose2D(2, 2, 0),
        Pose2D(5, 2, 0),
        Pose2D(6, 3, 1.57),
        Pose2D(5, 4, 3.14),
        Pose2D(2, 4, 3.14),
    ]
    semantics = build_path_semantics(raw, swaths, turns, swath_sample_step=0.5)
    path = REPAIR._nav_path(semantics.reconstructed_poses, REPAIR.rclpy.time.Time().to_msg())
    semantic_message = String()
    semantic_message.data = semantics.to_json()
    return path, semantic_message


def _validation(path_fingerprint, valid=True):
    message = String()
    message.data = json.dumps(
        {
            "valid": valid,
            "path_fingerprint": path_fingerprint,
            "invalid_component_ids": [] if valid else ["connection_0001"],
            "invalid_swath_ids": [],
        }
    )
    return message


def _costmap(obstacle=None):
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.width = 100
    message.info.height = 80
    message.info.resolution = 0.1
    message.info.origin.orientation.w = 1.0
    message.data = [0] * (message.info.width * message.info.height)
    if obstacle:
        column, row, cost = obstacle
        message.data[row * message.info.width + column] = cost
    return message


def _footprint():
    message = PolygonStamped()
    message.header.frame_id = "map"
    for x, y in PLATFORM["footprint"]:
        message.polygon.points.append(Point32(x=float(x), y=float(y)))
    return message


def _semantic_status(state="LOADED"):
    message = DiagnosticArray()
    status = DiagnosticStatus()
    status.name = "agt_semantic_map_server"
    status.message = state
    message.status = [status]
    return message


def _feed(
    node,
    valid=True,
    semantic_state="LOADED",
    costmap=None,
    keepout_mask=None,
):
    path, semantics = _products()
    node._path_callback(path)
    node._semantics_callback(semantics)
    fingerprint = json.loads(semantics.data)["path_fingerprint"]
    node._validation_callback(_validation(fingerprint, valid))
    node._costmap_callback(costmap or _costmap())
    node._footprint_callback(_footprint())
    node._semantic_status_callback(_semantic_status(semantic_state))
    node._keepout_mask_callback(keepout_mask or _costmap())
    return path


def test_valid_path_is_published_without_calling_nav2_planner():
    node = REPAIR.CoveragePathRepair(parameter_overrides=_parameters())
    try:
        source = _feed(node, valid=True)
        success, detail = node._start_repair()
        report = json.loads(node.last_report_json)
        assert success
        assert "no repair" in detail
        assert report["success"]
        assert report["repaired_segment_count"] == 0
        assert report["swath_coordinates_unchanged"]
        assert len(node.last_repaired_path.poses) == len(source.poses)
        topics = dict(node.get_topic_names_and_types())
        assert topics["/agt/coverage/path_repaired"] == ["nav_msgs/msg/Path"]
        assert topics["/agt/coverage/repair_report"] == ["std_msgs/msg/String"]
    finally:
        node.destroy_node()


def test_invalid_connection_fails_closed_when_planner_is_unavailable():
    node = REPAIR.CoveragePathRepair(parameter_overrides=_parameters())
    try:
        _feed(node, valid=False, costmap=_costmap((60, 30, 100)))
        success, _detail = node._start_repair()
        report = json.loads(node.last_report_json)
        assert not success
        assert report["error_code"] == "planner_action_unavailable"
        assert node.last_repaired_path.poses == []
    finally:
        node.destroy_node()


def test_semantic_status_must_be_loaded_before_repair():
    node = REPAIR.CoveragePathRepair(parameter_overrides=_parameters())
    try:
        _feed(node, valid=False, semantic_state="UNLOADED")
        success, _detail = node._start_repair()
        report = json.loads(node.last_report_json)
        assert not success
        assert report["error_code"] == "semantic_map_not_loaded"
        assert node.last_repaired_path.poses == []
    finally:
        node.destroy_node()


def test_candidate_validation_uses_full_footprint_and_global_costmap():
    node = REPAIR.CoveragePathRepair(parameter_overrides=_parameters())
    try:
        _feed(node, valid=False, costmap=_costmap((60, 30, 100)))
        candidate = [Pose2D(5, 2, 0), Pose2D(6, 3, 1.57), Pose2D(5, 4, 3.14)]
        result = node._validate_poses(candidate)
        assert not result.report.valid
        assert result.report.collision_pose_count > 0
    finally:
        node.destroy_node()


def test_candidate_validation_checks_keepout_mask_even_if_costmap_is_free():
    node = REPAIR.CoveragePathRepair(parameter_overrides=_parameters())
    try:
        _feed(node, valid=False, keepout_mask=_costmap((60, 30, 100)))
        candidate = [Pose2D(5, 2, 0), Pose2D(6, 3, 1.57), Pose2D(5, 4, 3.14)]
        result = node._validate_poses(candidate)
        assert not result.report.valid
        assert "semantic_keepout_collision" in result.report.error_codes
    finally:
        node.destroy_node()


def test_async_compute_path_action_repairs_connection_and_preserves_swaths():
    server_node = Node("task12_test_planner")

    def execute(goal_handle):
        result = ComputePathToPose.Result()
        result.path.header.frame_id = "map"
        middle = PoseStamped()
        middle.header.frame_id = "map"
        middle.pose.position.x = 4.2
        middle.pose.position.y = 3.0
        middle.pose.orientation.w = 1.0
        result.path.poses = [goal_handle.request.start, middle, goal_handle.request.goal]
        goal_handle.succeed()
        return result

    action_name = "/agt/test/task12_compute_path"
    action_server = ActionServer(
        server_node, ComputePathToPose, action_name, execute_callback=execute
    )
    node = REPAIR.CoveragePathRepair(
        parameter_overrides=_parameters(planner_action=action_name)
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server_node)
    executor.add_node(node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        _feed(node, valid=False, costmap=_costmap((60, 30, 100)))
        deadline = time.monotonic() + 3.0
        while not node.planner_action.server_is_ready() and time.monotonic() < deadline:
            time.sleep(0.01)
        success, _detail = node._start_repair()
        assert success
        while node.pending is not None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert node.pending is None
        report = json.loads(node.last_report_json)
        assert report["success"]
        assert report["repaired_segment_count"] == 1
        assert report["repaired_component_ids"] == ["connection_0001"]
        assert report["swath_coordinates_unchanged"]
        assert report["final_validation"]["valid"]
        assert node.last_repaired_path.poses
    finally:
        executor.shutdown()
        thread.join(timeout=1.0)
        action_server.destroy()
        node.destroy_node()
        server_node.destroy_node()
