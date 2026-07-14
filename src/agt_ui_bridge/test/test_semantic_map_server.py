import importlib.util
import json
from pathlib import Path
import shutil
import sys
import time

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import LoadMap
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
SCRIPT = PACKAGE_ROOT / "scripts/semantic_map_server.py"
SPEC = importlib.util.spec_from_file_location("semantic_map_server", SCRIPT)
SERVER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SERVER)

from agt_ui_bridge.semantic_io import sha256_file  # noqa: E402

EXAMPLE_ROOT = REPOSITORY_ROOT / "docs/interfaces/examples/semantic_map"
SEMANTIC_PATH = EXAMPLE_ROOT / "semantic/semantic_map.geojson"
PROFILE_PATH = REPOSITORY_ROOT / "profiles/platforms/bunker.yaml"


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _parameters(semantic_path=SEMANTIC_PATH, auto_load=False):
    return [
        Parameter("semantic_map", value=str(semantic_path)),
        Parameter("platform_profile", value=str(PROFILE_PATH)),
        Parameter("auto_load", value=auto_load),
    ]


def _base_map():
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.resolution = 1.0
    message.info.width = 10
    message.info.height = 10
    message.info.origin.position.x = -1.0
    message.info.origin.position.y = -2.0
    message.info.origin.orientation.w = 1.0
    message.data = [0] * 100
    return message


def _copy_task(tmp_path):
    tmp_path.mkdir(parents=True, exist_ok=True)
    shutil.copy2(EXAMPLE_ROOT / "example_map.pgm", tmp_path / "example_map.pgm")
    shutil.copy2(EXAMPLE_ROOT / "example_map.yaml", tmp_path / "example_map.yaml")
    semantic_dir = tmp_path / "semantic"
    semantic_dir.mkdir()
    shutil.copy2(SEMANTIC_PATH, semantic_dir / "semantic_map.geojson")
    coverage = yaml.safe_load(
        (EXAMPLE_ROOT / "semantic/coverage.yaml").read_text(encoding="utf-8")
    )
    coverage["base_map_sha256"] = sha256_file(tmp_path / "example_map.yaml")
    (semantic_dir / "coverage.yaml").write_text(
        yaml.safe_dump(coverage, sort_keys=False), encoding="utf-8"
    )
    return semantic_dir / "semantic_map.geojson"


def _status(node):
    return node.last_status.status[0]


def test_valid_load_publishes_markers_semantic_mask_and_standard_services():
    node = SERVER.SemanticMapServer(parameter_overrides=_parameters())
    try:
        map_yaml = EXAMPLE_ROOT / "example_map.yaml"
        map_image = EXAMPLE_ROOT / "example_map.pgm"
        original_files = (map_yaml.read_bytes(), map_image.read_bytes())
        source_map = _base_map()
        node._base_map_callback(source_map)
        success, state, _message = node._load_and_activate(str(SEMANTIC_PATH))

        assert success
        assert state == "LOADED"
        assert _status(node).message == "LOADED"
        assert len(node.active_candidate.markers.markers) == 8
        assert node.active_candidate.mask.info.width == 10
        assert node.active_candidate.mask.info.height == 10
        assert set(node.active_candidate.mask.data) == {0, 100}
        assert node.active_candidate.mask.data[0] == 100
        assert node.active_candidate.mask.data[22] == 0
        assert list(source_map.data) == [0] * 100
        assert (map_yaml.read_bytes(), map_image.read_bytes()) == original_files
        values = {item.key: item.value for item in _status(node).values}
        assert values["mask_mode"] == "semantic_keepout_task06"
        services = dict(node.get_service_names_and_types())
        assert services["/agt/map/semantic/load"] == ["nav2_msgs/srv/LoadMap"]
        assert services["/agt/map/semantic/reload"] == ["std_srvs/srv/Trigger"]
        assert services["/agt/map/semantic/validate"] == ["std_srvs/srv/Trigger"]

        assert SERVER.LATCHED_QOS.depth == 1
        for topic in (
            "/agt/map/semantic_markers",
            "/agt/map/keepout_mask",
            "/agt/map/semantic_status",
        ):
            info = node.get_publishers_info_by_topic(topic)[0].qos_profile
            assert info.reliability == SERVER.ReliabilityPolicy.RELIABLE
            assert info.durability == SERVER.DurabilityPolicy.TRANSIENT_LOCAL
    finally:
        node.destroy_node()


def test_failed_load_keeps_previous_active_products_and_reports_categories(
    tmp_path, monkeypatch
):
    node = SERVER.SemanticMapServer(parameter_overrides=_parameters())
    try:
        node._base_map_callback(_base_map())
        assert node._load_and_activate(str(SEMANTIC_PATH))[0]
        active = node.active_candidate

        assert not node._load_and_activate(str(tmp_path / "missing.geojson"))[0]
        assert _status(node).message == "LOAD_FAILED"
        assert node.active_candidate is active

        hash_path = _copy_task(tmp_path / "hash")
        map_path = hash_path.parents[1] / "example_map.yaml"
        map_path.write_text(map_path.read_text() + "# changed\n", encoding="utf-8")
        assert not node._load_and_activate(str(hash_path))[0]
        assert _status(node).message == "HASH_MISMATCH"
        assert node.active_candidate is active

        geometry_path = _copy_task(tmp_path / "geometry")
        document = json.loads(geometry_path.read_text(encoding="utf-8"))
        entry = next(
            feature
            for feature in document["features"]
            if feature["properties"]["feature_type"] == "entry_pose"
        )
        entry["geometry"]["coordinates"] = [9.0, 9.0]
        geometry_path.write_text(json.dumps(document), encoding="utf-8")
        assert not node._load_and_activate(str(geometry_path))[0]
        assert _status(node).message == "GEOMETRY_INVALID"
        assert node.active_candidate is active

        monkeypatch.setattr(
            node,
            "_build_keepout_mask",
            lambda *_args: (_ for _ in ()).throw(RuntimeError("mask failure")),
        )
        assert not node._load_and_activate(str(SEMANTIC_PATH))[0]
        assert _status(node).message == "RASTERIZATION_FAILED"
        assert node.active_candidate is active
    finally:
        node.destroy_node()


def test_late_subscriber_receives_transient_local_mask():
    server = SERVER.SemanticMapServer(parameter_overrides=_parameters())
    listener = Node("semantic_mask_late_listener")
    received = []
    try:
        server._base_map_callback(_base_map())
        assert server._load_and_activate(str(SEMANTIC_PATH))[0]
        listener.create_subscription(
            OccupancyGrid,
            "/agt/map/keepout_mask",
            received.append,
            SERVER.LATCHED_QOS,
        )
        _spin_until([server, listener], lambda: bool(received))
        assert received
        assert received[0].info.width == 10
        assert set(received[0].data) == {0, 100}
    finally:
        listener.destroy_node()
        server.destroy_node()


def test_standard_load_reload_and_validate_callbacks():
    node = SERVER.SemanticMapServer(parameter_overrides=_parameters())
    try:
        node._base_map_callback(_base_map())
        load_request = LoadMap.Request()
        load_request.map_url = SEMANTIC_PATH.as_uri()
        load_response = node._load_callback(load_request, LoadMap.Response())
        assert load_response.result == LoadMap.Response.RESULT_SUCCESS
        assert load_response.map.info.width == 10

        validate_response = node._validate_callback(
            Trigger.Request(), Trigger.Response()
        )
        assert validate_response.success
        assert validate_response.message == "semantic task is valid"

        reload_response = node._reload_callback(
            Trigger.Request(), Trigger.Response()
        )
        assert reload_response.success
        assert _status(node).message == "LOADED"
    finally:
        node.destroy_node()


def test_server_started_late_receives_transient_local_base_map():
    source = Node("semantic_base_map_source")
    publisher = source.create_publisher(
        OccupancyGrid, "/agt/map/global_occupancy", SERVER.LATCHED_QOS
    )
    publisher.publish(_base_map())
    server = SERVER.SemanticMapServer(parameter_overrides=_parameters(auto_load=True))
    try:
        _spin_until(
            [source, server], lambda: server.active_candidate is not None
        )
        assert server.base_map is not None
        assert server.active_candidate is not None
        assert _status(server).message == "LOADED"
    finally:
        server.destroy_node()
        source.destroy_node()


def _spin_until(nodes, predicate, timeout=3.0):
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    deadline = time.monotonic() + timeout
    try:
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
    finally:
        executor.shutdown()
