import importlib.util
import json
import math
from pathlib import Path
import sys

from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path as NavPath
from opennav_coverage_msgs.msg import PathComponents, Swath
import pytest
import rclpy


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPOSITORY_ROOT / "src/agt_ui_bridge"))
SCRIPT = PACKAGE_ROOT / "scripts/coverage_request_adapter.py"
SPEC = importlib.util.spec_from_file_location("coverage_request_adapter", SCRIPT)
ADAPTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ADAPTER)


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _pose(x, y, yaw=0.0):
    stamped = PoseStamped()
    stamped.header.frame_id = "map"
    stamped.pose.position.x = float(x)
    stamped.pose.position.y = float(y)
    stamped.pose.orientation.z = math.sin(yaw * 0.5)
    stamped.pose.orientation.w = math.cos(yaw * 0.5)
    return stamped


def _path():
    message = NavPath()
    message.header.frame_id = "map"
    message.poses = [
        _pose(0, 0),
        _pose(2, 0),
        _pose(4, 0),
        _pose(4.5, 0.5, math.pi / 2.0),
        _pose(4, 1, math.pi),
        _pose(2, 1, math.pi),
        _pose(0, 1, math.pi),
    ]
    return message


def _components():
    message = PathComponents()
    message.header.frame_id = "map"
    message.contains_turns = True
    message.swaths_ordered = True
    for start, end in (((0, 0), (4, 0)), ((4, 1), (0, 1))):
        swath = Swath()
        swath.start = Point32(x=float(start[0]), y=float(start[1]))
        swath.end = Point32(x=float(end[0]), y=float(end[1]))
        message.swaths.append(swath)
    turn = NavPath()
    turn.header.frame_id = "map"
    turn.poses = [
        _pose(4, 0),
        _pose(4.5, 0.5, math.pi / 2.0),
        _pose(4, 1, math.pi),
    ]
    message.turns.append(turn)
    return message


def test_real_path_components_produce_transactional_semantic_products():
    semantics, reconstructed, message = ADAPTER._semantic_products(
        _path(), _components(), swath_sample_step=0.5
    )
    document = json.loads(message.data)

    assert semantics.swath_ids == ("swath_0001", "swath_0002")
    assert reconstructed.header.frame_id == "map"
    assert reconstructed.poses
    assert document["raw_pose_count"] == len(_path().poses)
    assert {item["component_type"] for item in document["raw_segments"]} == {
        "SWATH",
        "CONNECTION",
    }
    assert document["length_error"] <= document["length_tolerance"]


def test_semantic_products_reject_unordered_or_wrong_frame_components():
    components = _components()
    components.swaths_ordered = False
    with pytest.raises(ADAPTER.PathSemanticsError) as unordered:
        ADAPTER._semantic_products(_path(), components)
    assert unordered.value.code == "path_components_unordered_swaths"

    components = _components()
    components.header.frame_id = "odom"
    with pytest.raises(ADAPTER.PathSemanticsError) as frame:
        ADAPTER._semantic_products(_path(), components)
    assert frame.value.code == "invalid_path_components_frame"


def test_planning_status_references_stable_swath_ids():
    node = ADAPTER.CoverageRequestAdapter()
    try:
        semantics, _path_message, _semantics_message = ADAPTER._semantic_products(
            _path(), _components()
        )
        node.active_semantics = semantics
        node._publish_status("SUCCEEDED", "semantic path ready")
        values = {item.key: item.value for item in node.last_status.status[0].values}
        assert values["swath_ids"] == "swath_0001,swath_0002"
        assert values["swath_count"] == "2"
        assert values["connection_count"] == "1"
    finally:
        node.destroy_node()


def test_adapter_exposes_non_executable_server_preview_topic():
    node = ADAPTER.CoverageRequestAdapter()
    try:
        topics = dict(node.get_topic_names_and_types())
        assert topics["/agt/coverage/path_preview"] == ["nav_msgs/msg/Path"]
    finally:
        node.destroy_node()
