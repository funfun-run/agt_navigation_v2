import importlib.util
import json
import math
from pathlib import Path
import sys
from threading import Thread
import time

from agt_interfaces.action import ExecuteCoverageTask
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid, Path as NavPath
import pytest
from rcl_interfaces.srv import SetParametersAtomically
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
SCRIPT = PACKAGE_ROOT / "scripts/coverage_task_server.py"
SPEC = importlib.util.spec_from_file_location("coverage_task_server", SCRIPT)
TASK_SERVER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TASK_SERVER)

from agt_coverage_planning.coverage_task import (  # noqa: E402
    ERROR_PLANNING,
    ERROR_REPAIR_DISALLOWED,
    ERROR_SAFETY_NOT_READY,
)
from agt_coverage_planning.path_semantics import (  # noqa: E402
    Pose2D,
    SwathInput,
    TurnInput,
    build_path_semantics,
)


LATCHED_QOS = QoSProfile(
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class Harness:
    counter = 0

    def __init__(
        self,
        validation_valid=True,
        planning_success=True,
        repair_success=False,
        safety=True,
        delay=0.0,
    ):
        type(self).counter += 1
        suffix = str(type(self).counter)
        self.validation_valid = validation_valid
        self.planning_success = planning_success
        self.repair_success = repair_success
        self.safety = safety
        self.delay = delay
        self.follow_calls = 0
        self.child_canceled = False
        self.node = Node(f"task14_harness_{suffix}")
        self.client_node = Node(f"task14_client_{suffix}")
        self.action_name = f"/agt/test/task14/execute_{suffix}"
        self.load_name = f"/agt/test/task14/load_{suffix}"
        self.params_name = f"/agt/test/task14/params_{suffix}"
        self.plan_name = f"/agt/test/task14/plan_{suffix}"
        self.repair_name = f"/agt/test/task14/repair_{suffix}"
        self.follow_name = f"/agt/test/task14/follow_{suffix}"
        self.semantic_path = f"/tmp/task14_semantic_{suffix}.geojson"
        self.path, self.semantics = _products()

        self.semantic_status = self.node.create_publisher(
            DiagnosticArray, "/agt/map/semantic_status", LATCHED_QOS
        )
        self.keepout = self.node.create_publisher(
            OccupancyGrid, "/agt/map/keepout_mask", LATCHED_QOS
        )
        self.coverage_status = self.node.create_publisher(
            DiagnosticArray, "/agt/coverage/status", LATCHED_QOS
        )
        self.semantics_publisher = self.node.create_publisher(
            String, "/agt/coverage/path_semantics", LATCHED_QOS
        )
        self.validation = self.node.create_publisher(
            String, "/agt/coverage/validation_report", LATCHED_QOS
        )
        self.repair_report = self.node.create_publisher(
            String, "/agt/coverage/repair_report", LATCHED_QOS
        )
        self.validated = self.node.create_publisher(
            NavPath, "/agt/coverage/path_validated", LATCHED_QOS
        )
        self.reconstructed = self.node.create_publisher(
            NavPath, "/agt/coverage/path_reconstructed", LATCHED_QOS
        )
        self.repaired = self.node.create_publisher(
            NavPath, "/agt/coverage/path_repaired", LATCHED_QOS
        )
        self.safety_status = self.node.create_publisher(
            DiagnosticArray, "/agt/safety/status", 10
        )
        self.node.create_service(LoadMap, self.load_name, self._load)
        self.node.create_service(SetParametersAtomically, self.params_name, self._params)
        self.node.create_service(Trigger, self.plan_name, self._plan)
        self.node.create_service(Trigger, self.repair_name, self._repair)
        self.follow_server = ActionServer(
            self.node,
            FollowPath,
            self.follow_name,
            execute_callback=self._follow,
            cancel_callback=lambda _request: CancelResponse.ACCEPT,
        )
        parameters = [
            Parameter("action_name", value=self.action_name),
            Parameter("semantic_load_service", value=self.load_name),
            Parameter("adapter_parameter_service", value=self.params_name),
            Parameter("plan_service", value=self.plan_name),
            Parameter("repair_service", value=self.repair_name),
            Parameter("follow_path_action", value=self.follow_name),
            Parameter("execution_enabled", value=True),
            Parameter("service_timeout", value=1.0),
            Parameter("stage_timeout", value=3.0),
            Parameter("safety_status_timeout", value=1.0),
            Parameter("poll_period", value=0.005),
        ]
        self.server = TASK_SERVER.CoverageTaskServer(parameter_overrides=parameters)
        self.client = ActionClient(self.client_node, ExecuteCoverageTask, self.action_name)
        self.executor = MultiThreadedExecutor(num_threads=6)
        for node in (self.node, self.server, self.client_node):
            self.executor.add_node(node)
        self.thread = Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.safety_timer = self.node.create_timer(0.05, self._publish_safety)

    def close(self):
        self.executor.shutdown()
        self.thread.join(timeout=1.0)
        self.follow_server.destroy()
        self.client.destroy()
        self.server.destroy_node()
        self.client_node.destroy_node()
        self.node.destroy_node()

    def goal(self, allow_repair=False):
        goal = ExecuteCoverageTask.Goal()
        goal.semantic_map_uri = self.semantic_path
        goal.field_id = "field_001"
        goal.planning_mode = "polygon"
        goal.controller_id = "FollowPath"
        goal.allow_repair = allow_repair
        return goal

    def send(self, allow_repair=False, feedback=None):
        assert self.client.wait_for_server(timeout_sec=1.0)
        deadline = time.monotonic() + 1.0
        while self.server.safety_status is None and time.monotonic() < deadline:
            time.sleep(0.005)
        assert self.server.safety_status is not None
        future = self.client.send_goal_async(
            self.goal(allow_repair), feedback_callback=feedback
        )
        handle = _wait(future)
        assert handle.accepted
        return handle

    def _load(self, request, response):
        response.result = LoadMap.Response.RESULT_SUCCESS
        self.semantic_status.publish(
            _diagnostic(
                "agt_semantic_map_server",
                "LOADED",
                {"semantic_path": str(Path(request.map_url).resolve())},
            )
        )
        mask = OccupancyGrid()
        mask.header.frame_id = "map"
        self.keepout.publish(mask)
        return response

    def _params(self, _request, response):
        response.result.successful = True
        return response

    def _plan(self, _request, response):
        response.success = True
        if not self.planning_success:
            self.coverage_status.publish(
                _diagnostic(
                    "agt_coverage_request_adapter",
                    "FAILED",
                    {"detail": "synthetic planning failure"},
                )
            )
            return response
        semantic_message = String()
        semantic_message.data = self.semantics.to_json()
        self.semantics_publisher.publish(semantic_message)
        self.reconstructed.publish(_nav_path(self.semantics.reconstructed_poses))
        validated = self.path if self.validation_valid else _nav_path([])
        self.validated.publish(validated)
        report = String()
        report.data = json.dumps(
            {
                "valid": self.validation_valid,
                "path_fingerprint": self.semantics.path_fingerprint,
                "invalid_component_ids": [] if self.validation_valid else ["connection_0001"],
                "invalid_swath_ids": [],
            }
        )
        self.validation.publish(report)
        self.coverage_status.publish(
            _diagnostic("agt_coverage_request_adapter", "SUCCEEDED", {"detail": "ok"})
        )
        return response

    def _repair(self, _request, response):
        response.success = self.repair_success
        response.message = "repair accepted" if self.repair_success else "repair failed"
        if self.repair_success:
            self.repaired.publish(_nav_path(self.semantics.reconstructed_poses))
            report = String()
            report.data = json.dumps(
                {
                    "success": True,
                    "repaired_segment_count": 1,
                    "final_validation": {"valid": True},
                }
            )
            self.repair_report.publish(report)
        return response

    def _publish_safety(self):
        values = {
            "motion_enabled": "true" if self.safety else "false",
            "estop_latched": "false",
        }
        self.safety_status.publish(
            _diagnostic("agt_safety/tracked_controller", "input_timeout", values)
        )

    def _follow(self, goal_handle):
        self.follow_calls += 1
        feedback = FollowPath.Feedback()
        feedback.distance_to_goal = 2.0
        goal_handle.publish_feedback(feedback)
        deadline = time.monotonic() + self.delay
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self.child_canceled = True
                goal_handle.canceled()
                return FollowPath.Result()
            time.sleep(0.005)
        feedback.distance_to_goal = 0.5
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        return FollowPath.Result()


def test_success_uses_nav2_and_reports_stage_and_swath_progress():
    harness = Harness()
    stages = []
    try:
        handle = harness.send(feedback=lambda message: stages.append(message.feedback))
        wrapped = _wait(handle.get_result_async())
        assert wrapped.result.success
        assert wrapped.result.executed_length > 0.0
        assert harness.follow_calls == 1
        names = [feedback.current_stage for feedback in stages]
        assert names[:4] == ["LOADING", "VALIDATING_MAP", "PLANNING", "VALIDATING_PATH"]
        assert "READY" in names
        assert "EXECUTING" in names
        assert names[-1] == "COMPLETED"
        executing = [item for item in stages if item.current_stage == "EXECUTING"]
        assert any(item.total_swaths == 2 for item in executing)
    finally:
        harness.close()


def test_planning_failure_never_enters_nav2_execution():
    harness = Harness(planning_success=False)
    try:
        wrapped = _wait(harness.send().get_result_async())
        assert not wrapped.result.success
        assert wrapped.result.error_code == ERROR_PLANNING
        assert harness.follow_calls == 0
    finally:
        harness.close()


def test_invalid_path_without_repair_fails_immediately():
    harness = Harness(validation_valid=False)
    try:
        wrapped = _wait(harness.send(allow_repair=False).get_result_async())
        assert not wrapped.result.success
        assert wrapped.result.error_code == ERROR_REPAIR_DISALLOWED
        assert harness.follow_calls == 0
    finally:
        harness.close()


def test_safety_not_enabled_blocks_follow_path():
    harness = Harness(safety=False)
    try:
        wrapped = _wait(harness.send().get_result_async())
        assert not wrapped.result.success
        assert wrapped.result.error_code == ERROR_SAFETY_NOT_READY
        assert harness.follow_calls == 0
    finally:
        harness.close()


def test_allowed_repair_uses_repaired_path_before_execution():
    harness = Harness(validation_valid=False, repair_success=True)
    try:
        wrapped = _wait(harness.send(allow_repair=True).get_result_async())
        assert wrapped.result.success
        assert wrapped.result.repaired_segment_count == 1
        assert harness.follow_calls == 1
    finally:
        harness.close()


def test_safety_loss_during_execution_cancels_nav2():
    harness = Harness(delay=1.0)
    stages = []
    try:
        handle = harness.send(
            feedback=lambda message: stages.append(message.feedback.current_stage)
        )
        deadline = time.monotonic() + 2.0
        while "EXECUTING" not in stages and time.monotonic() < deadline:
            time.sleep(0.005)
        harness.safety = False
        wrapped = _wait(handle.get_result_async())
        assert not wrapped.result.success
        assert wrapped.result.error_code == ERROR_SAFETY_NOT_READY
        deadline = time.monotonic() + 1.0
        while not harness.child_canceled and time.monotonic() < deadline:
            time.sleep(0.005)
        assert harness.child_canceled
    finally:
        harness.close()


def test_cancel_propagates_to_active_nav2_goal():
    harness = Harness(delay=1.0)
    stages = []
    try:
        handle = harness.send(
            feedback=lambda message: stages.append(message.feedback.current_stage)
        )
        deadline = time.monotonic() + 2.0
        while "EXECUTING" not in stages and time.monotonic() < deadline:
            time.sleep(0.005)
        cancel_response = _wait(handle.cancel_goal_async())
        assert cancel_response.goals_canceling
        wrapped = _wait(handle.get_result_async())
        assert not wrapped.result.success
        assert wrapped.result.error_code == TASK_SERVER.ERROR_CANCELED
        deadline = time.monotonic() + 1.0
        while not harness.child_canceled and time.monotonic() < deadline:
            time.sleep(0.005)
        assert harness.child_canceled
        assert stages[-1] == "CANCELED"
    finally:
        harness.close()


def _products():
    poses = [
        Pose2D(0, 0, 0),
        Pose2D(2, 0, 0),
        Pose2D(2.5, 0.5, 1.57),
        Pose2D(2, 1, 3.14),
        Pose2D(0, 1, 3.14),
    ]
    semantics = build_path_semantics(
        poses,
        [SwathInput(poses[0], poses[1]), SwathInput(poses[3], poses[4])],
        [TurnInput(tuple(poses[1:4]))],
        contains_turns=True,
        swath_sample_step=1.0,
    )
    return _nav_path(poses), semantics


def _nav_path(poses):
    message = NavPath()
    message.header.frame_id = "map"
    for pose in poses:
        stamped = PoseStamped()
        stamped.header.frame_id = "map"
        stamped.pose.position.x = float(pose.x)
        stamped.pose.position.y = float(pose.y)
        stamped.pose.orientation.z = math.sin(pose.yaw * 0.5)
        stamped.pose.orientation.w = math.cos(pose.yaw * 0.5)
        message.poses.append(stamped)
    return message


def _diagnostic(name, state, values):
    status = DiagnosticStatus()
    status.name = name
    status.message = state
    status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
    message = DiagnosticArray()
    message.status = [status]
    return message


def _wait(future, timeout=4.0):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.005)
    assert future.done()
    return future.result()
