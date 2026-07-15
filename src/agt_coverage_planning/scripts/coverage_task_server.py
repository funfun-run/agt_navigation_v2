#!/usr/bin/env python3
"""Coordinate semantic loading, coverage planning, validation and Nav2 execution."""

from copy import deepcopy
import json
import math
from pathlib import Path
from threading import Lock
import time
from urllib.parse import unquote, urlparse

from action_msgs.msg import GoalStatus
from agt_interfaces.action import ExecuteCoverageTask
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav2_msgs.action import FollowPath
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rcl_interfaces.srv import SetParametersAtomically
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from agt_coverage_planning.coverage_task import (
    ERROR_CANCELED,
    ERROR_EXECUTION,
    ERROR_INTERNAL,
    ERROR_MAP_LOAD,
    ERROR_NONE,
    ERROR_PATH_INVALID,
    ERROR_PLANNING,
    ERROR_REPAIR_DISALLOWED,
    ERROR_REPAIR_FAILED,
    ERROR_SAFETY_NOT_READY,
    CoverageTaskError,
    build_progress_model,
    validate_task_goal,
)
from agt_coverage_planning.path_validator import Pose2D


LATCHED_QOS = QoSProfile(
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class TaskCanceled(RuntimeError):
    pass


class CoverageTaskServer(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__(
            "coverage_task_server",
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter("action_name", "/agt/coverage/execute")
        self.declare_parameter("semantic_load_service", "/agt/map/semantic/load")
        self.declare_parameter(
            "adapter_parameter_service",
            "/coverage_request_adapter/set_parameters_atomically",
        )
        self.declare_parameter("plan_service", "/agt/coverage/plan")
        self.declare_parameter("repair_service", "/agt/coverage/repair")
        self.declare_parameter("follow_path_action", "/follow_path")
        self.declare_parameter("execution_enabled", False)
        self.declare_parameter("service_timeout", 5.0)
        self.declare_parameter("stage_timeout", 120.0)
        self.declare_parameter("execution_timeout", 0.0)
        self.declare_parameter(
            "safety_status_name", "agt_safety/tracked_controller"
        )
        self.declare_parameter("safety_status_timeout", 1.0)
        self.declare_parameter("poll_period", 0.02)

        self.callback_group = ReentrantCallbackGroup()
        self.service_timeout = _positive_parameter(self, "service_timeout")
        self.stage_timeout = _positive_parameter(self, "stage_timeout")
        self.execution_timeout = _nonnegative_parameter(self, "execution_timeout")
        self.safety_timeout = _positive_parameter(self, "safety_status_timeout")
        self.poll_period = _positive_parameter(self, "poll_period")
        self.execution_enabled = bool(self.get_parameter("execution_enabled").value)
        self.safety_status_name = str(
            self.get_parameter("safety_status_name").value
        ).strip()
        if not self.safety_status_name:
            raise RuntimeError("safety_status_name must not be empty")

        self.semantic_status = None
        self.semantic_status_generation = 0
        self.keepout_mask = None
        self.keepout_generation = 0
        self.coverage_status = None
        self.coverage_status_generation = 0
        self.semantics_document = None
        self.semantics_generation = 0
        self.validation_report = None
        self.validation_generation = 0
        self.repair_report = None
        self.repair_generation = 0
        self.validated_path = None
        self.validated_path_generation = 0
        self.repaired_path = None
        self.repaired_path_generation = 0
        self.reconstructed_path = None
        self.reconstructed_path_generation = 0
        self.safety_status = None
        self.safety_received_at = float("-inf")
        self._reservation_lock = Lock()
        self._goal_reserved = False
        self._parent_goal = None
        self._progress_model = None
        self._distance_remaining = 0.0
        self.last_stage = ""

        self.create_subscription(
            DiagnosticArray,
            "/agt/map/semantic_status",
            self._semantic_status_callback,
            LATCHED_QOS,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            OccupancyGrid,
            "/agt/map/keepout_mask",
            self._keepout_callback,
            LATCHED_QOS,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            DiagnosticArray,
            "/agt/coverage/status",
            self._coverage_status_callback,
            LATCHED_QOS,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            "/agt/coverage/path_semantics",
            self._semantics_callback,
            LATCHED_QOS,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            "/agt/coverage/validation_report",
            self._validation_callback,
            LATCHED_QOS,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            "/agt/coverage/repair_report",
            self._repair_callback,
            LATCHED_QOS,
            callback_group=self.callback_group,
        )
        for topic, callback in (
            ("/agt/coverage/path_validated", self._validated_path_callback),
            ("/agt/coverage/path_repaired", self._repaired_path_callback),
            ("/agt/coverage/path_reconstructed", self._reconstructed_path_callback),
        ):
            self.create_subscription(
                NavPath,
                topic,
                callback,
                LATCHED_QOS,
                callback_group=self.callback_group,
            )
        self.create_subscription(
            DiagnosticArray,
            "/agt/safety/status",
            self._safety_callback,
            10,
            callback_group=self.callback_group,
        )

        self.load_client = self.create_client(
            LoadMap,
            str(self.get_parameter("semantic_load_service").value),
            callback_group=self.callback_group,
        )
        self.adapter_parameters = self.create_client(
            SetParametersAtomically,
            str(self.get_parameter("adapter_parameter_service").value),
            callback_group=self.callback_group,
        )
        self.plan_client = self.create_client(
            Trigger,
            str(self.get_parameter("plan_service").value),
            callback_group=self.callback_group,
        )
        self.repair_client = self.create_client(
            Trigger,
            str(self.get_parameter("repair_service").value),
            callback_group=self.callback_group,
        )
        self.follow_path = ActionClient(
            self,
            FollowPath,
            str(self.get_parameter("follow_path_action").value),
            callback_group=self.callback_group,
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray,
            "/agt/coverage/task_status",
            LATCHED_QOS,
        )
        self.action_server = ActionServer(
            self,
            ExecuteCoverageTask,
            str(self.get_parameter("action_name").value),
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )

    def _goal_callback(self, _goal):
        with self._reservation_lock:
            if self._goal_reserved:
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        self._parent_goal = goal_handle
        self._progress_model = None
        self._distance_remaining = 0.0
        repaired_count = 0
        try:
            goal = validate_task_goal(goal_handle.request)
            final_path, semantics, repaired_count = self._prepare_path(goal_handle, goal)
            self._publish_feedback(goal_handle, "READY")
            self._require_execution_ready()
            progress = build_progress_model(
                _path_poses(final_path),
                semantics,
                _path_poses(self.reconstructed_path),
            )
            self._progress_model = progress
            self._distance_remaining = progress.total_length
            self._execute_path(goal_handle, goal.controller_id, final_path)
            self._publish_feedback(goal_handle, "COMPLETED")
            goal_handle.succeed()
            return _result(
                True,
                ERROR_NONE,
                "coverage task completed",
                progress.total_length,
                repaired_count,
            )
        except TaskCanceled as exc:
            self._publish_feedback(goal_handle, "CANCELED")
            goal_handle.canceled()
            return _result(
                False,
                ERROR_CANCELED,
                str(exc),
                self._executed_length(),
                repaired_count,
            )
        except CoverageTaskError as exc:
            self._publish_feedback(goal_handle, "FAILED")
            goal_handle.abort()
            return _result(
                False,
                exc.code,
                str(exc),
                self._executed_length(),
                repaired_count,
            )
        except Exception as exc:
            self._publish_feedback(goal_handle, "FAILED")
            goal_handle.abort()
            return _result(
                False,
                ERROR_INTERNAL,
                str(exc),
                self._executed_length(),
                repaired_count,
            )
        finally:
            self._parent_goal = None
            self._progress_model = None
            with self._reservation_lock:
                self._goal_reserved = False

    def _prepare_path(self, goal_handle, goal):
        self._publish_feedback(goal_handle, "LOADING")
        keepout_baseline = self.keepout_generation
        request = LoadMap.Request()
        request.map_url = goal.semantic_map_uri
        response = self._call_service(
            self.load_client,
            request,
            goal_handle,
            ERROR_MAP_LOAD,
            "semantic map load service",
        )
        if response.result != LoadMap.Response.RESULT_SUCCESS:
            raise CoverageTaskError(
                ERROR_MAP_LOAD,
                f"semantic map load failed with result {response.result}",
            )
        expected_path = _normalized_resource(goal.semantic_map_uri)
        self._publish_feedback(goal_handle, "VALIDATING_MAP")
        self._wait_until(
            lambda: all(
                (
                    self._semantic_loaded(expected_path),
                    self.keepout_generation > keepout_baseline,
                )
            ),
            goal_handle,
            ERROR_MAP_LOAD,
            "semantic products did not become ready",
        )

        parameter_request = SetParametersAtomically.Request()
        parameter_request.parameters = [
            Parameter("semantic_map", value=expected_path).to_parameter_msg(),
            Parameter("expected_field_id", value=goal.field_id).to_parameter_msg(),
            Parameter(
                "expected_planning_mode", value=goal.planning_mode
            ).to_parameter_msg(),
        ]
        parameter_response = self._call_service(
            self.adapter_parameters,
            parameter_request,
            goal_handle,
            ERROR_PLANNING,
            "coverage adapter parameter service",
        )
        if not parameter_response.result.successful:
            raise CoverageTaskError(
                ERROR_PLANNING,
                "coverage adapter rejected task parameters: "
                f"{parameter_response.result.reason}",
            )

        status_baseline = self.coverage_status_generation
        semantics_baseline = self.semantics_generation
        reconstructed_baseline = self.reconstructed_path_generation
        validation_baseline = self.validation_generation
        validated_path_baseline = self.validated_path_generation
        self._publish_feedback(goal_handle, "PLANNING")
        plan_response = self._call_service(
            self.plan_client,
            Trigger.Request(),
            goal_handle,
            ERROR_PLANNING,
            "coverage plan service",
        )
        if not plan_response.success:
            raise CoverageTaskError(ERROR_PLANNING, plan_response.message)
        self._wait_until(
            lambda: all(
                (
                    self.coverage_status_generation > status_baseline,
                    self.coverage_status is not None,
                    self.coverage_status is not None and self.coverage_status[0]
                    in {"SUCCEEDED", "FAILED", "REJECTED"},
                )
            ),
            goal_handle,
            ERROR_PLANNING,
            "coverage planner timed out",
        )
        if self.coverage_status[0] != "SUCCEEDED":
            raise CoverageTaskError(
                ERROR_PLANNING,
                self.coverage_status[1].get("detail", self.coverage_status[0]),
            )
        self._wait_until(
            lambda: all(
                (
                    self.semantics_generation > semantics_baseline,
                    self.reconstructed_path_generation > reconstructed_baseline,
                )
            ),
            goal_handle,
            ERROR_PLANNING,
            "coverage semantics and reconstructed path were not published",
        )
        semantics = deepcopy(self.semantics_document)

        self._publish_feedback(goal_handle, "VALIDATING_PATH")
        fingerprint = semantics.get("path_fingerprint")
        self._wait_until(
            lambda: all(
                (
                    self.validation_generation > validation_baseline,
                    self.validation_report is not None,
                    self.validation_report is not None
                    and self.validation_report.get("path_fingerprint")
                    == fingerprint,
                )
            ),
            goal_handle,
            ERROR_PATH_INVALID,
            "matching path validation report was not published",
        )
        report = deepcopy(self.validation_report)
        if report.get("valid") is True:
            self._wait_until(
                lambda: self.validated_path_generation > validated_path_baseline,
                goal_handle,
                ERROR_PATH_INVALID,
                "validated path was not published",
            )
            if self.validated_path is None or not self.validated_path.poses:
                raise CoverageTaskError(ERROR_PATH_INVALID, "validated path is empty")
            return deepcopy(self.validated_path), semantics, 0
        if not goal.allow_repair:
            raise CoverageTaskError(
                ERROR_REPAIR_DISALLOWED,
                "path is invalid and allow_repair is false",
            )

        repair_baseline = self.repair_generation
        repaired_path_baseline = self.repaired_path_generation
        self._publish_feedback(goal_handle, "REPAIRING")
        repair_response = self._call_service(
            self.repair_client,
            Trigger.Request(),
            goal_handle,
            ERROR_REPAIR_FAILED,
            "coverage repair service",
        )
        if not repair_response.success:
            raise CoverageTaskError(ERROR_REPAIR_FAILED, repair_response.message)
        self._wait_until(
            lambda: all(
                (
                    self.repair_generation > repair_baseline,
                    self.repaired_path_generation > repaired_path_baseline,
                )
            ),
            goal_handle,
            ERROR_REPAIR_FAILED,
            "coverage repair products timed out",
        )
        repair_report = deepcopy(self.repair_report)
        if not repair_report.get("success"):
            raise CoverageTaskError(
                ERROR_REPAIR_FAILED,
                repair_report.get("detail", "coverage repair failed"),
            )
        if self.repaired_path is None or not self.repaired_path.poses:
            raise CoverageTaskError(ERROR_REPAIR_FAILED, "repaired path is empty")
        return (
            deepcopy(self.repaired_path),
            semantics,
            int(repair_report.get("repaired_segment_count", 0)),
        )

    def _require_execution_ready(self):
        if not self.execution_enabled:
            raise CoverageTaskError(
                ERROR_EXECUTION,
                "coverage execution is disabled by configuration",
            )
        if not self._semantic_loaded():
            raise CoverageTaskError(ERROR_MAP_LOAD, "semantic map is no longer LOADED")
        if not self._safety_ready():
            raise CoverageTaskError(
                ERROR_SAFETY_NOT_READY,
                "agt_safety motion must be enabled with emergency stop clear",
            )
        if not self.follow_path.server_is_ready():
            raise CoverageTaskError(ERROR_EXECUTION, "Nav2 FollowPath is not ready")

    def _execute_path(self, parent, controller_id, path):
        self._publish_feedback(parent, "EXECUTING")
        goal = FollowPath.Goal()
        goal.path = deepcopy(path)
        goal.controller_id = controller_id
        future = self.follow_path.send_goal_async(
            goal,
            feedback_callback=self._execution_feedback,
        )
        child = self._wait_future(
            future, parent, ERROR_EXECUTION, "sending Nav2 FollowPath goal"
        )
        if not child.accepted:
            raise CoverageTaskError(ERROR_EXECUTION, "Nav2 rejected FollowPath goal")
        result_future = child.get_result_async()
        deadline = (
            time.monotonic() + self.execution_timeout
            if self.execution_timeout > 0.0
            else None
        )
        while not result_future.done():
            if parent.is_cancel_requested:
                self._cancel_child(child)
                raise TaskCanceled("coverage task canceled during execution")
            if not self._safety_ready():
                self._cancel_child(child)
                raise CoverageTaskError(
                    ERROR_SAFETY_NOT_READY,
                    "agt_safety became unavailable during execution",
                )
            if deadline is not None and time.monotonic() >= deadline:
                self._cancel_child(child)
                raise CoverageTaskError(ERROR_EXECUTION, "Nav2 FollowPath timed out")
            time.sleep(self.poll_period)
        wrapped = result_future.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise CoverageTaskError(
                ERROR_EXECUTION,
                f"Nav2 FollowPath returned action status {wrapped.status}",
            )
        self._distance_remaining = 0.0

    def _execution_feedback(self, message):
        if self._parent_goal is None or self._progress_model is None:
            return
        remaining = float(message.feedback.distance_to_goal)
        if math.isfinite(remaining):
            self._distance_remaining = max(0.0, remaining)
        self._publish_feedback(self._parent_goal, "EXECUTING")

    def _call_service(self, client, request, parent, code, name):
        if not client.wait_for_service(timeout_sec=self.service_timeout):
            raise CoverageTaskError(code, f"{name} is unavailable")
        return self._wait_future(client.call_async(request), parent, code, name)

    def _wait_future(self, future, parent, code, name):
        deadline = time.monotonic() + self.stage_timeout
        while not future.done():
            if parent.is_cancel_requested:
                raise TaskCanceled(f"coverage task canceled while waiting for {name}")
            if time.monotonic() >= deadline:
                raise CoverageTaskError(code, f"{name} timed out")
            time.sleep(self.poll_period)
        try:
            return future.result()
        except Exception as exc:
            raise CoverageTaskError(code, f"{name} failed: {exc}") from exc

    def _wait_until(self, predicate, parent, code, message):
        deadline = time.monotonic() + self.stage_timeout
        while not predicate():
            if parent.is_cancel_requested:
                raise TaskCanceled("coverage task canceled")
            if time.monotonic() >= deadline:
                raise CoverageTaskError(code, message)
            time.sleep(self.poll_period)

    def _cancel_child(self, child):
        future = child.cancel_goal_async()
        deadline = time.monotonic() + self.service_timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(self.poll_period)
        if not future.done():
            raise CoverageTaskError(ERROR_EXECUTION, "Nav2 cancel request timed out")
        response = future.result()
        if not response.goals_canceling:
            raise CoverageTaskError(ERROR_EXECUTION, "Nav2 rejected cancel request")

    def _publish_feedback(self, goal_handle, stage):
        feedback = ExecuteCoverageTask.Feedback()
        feedback.current_stage = stage
        if self._progress_model is not None:
            feedback.total_swaths = self._progress_model.total_swaths
            feedback.current_swath_index = self._progress_model.swath_index(
                self._distance_remaining
            )
            feedback.distance_remaining = self._distance_remaining
        goal_handle.publish_feedback(feedback)
        self.last_stage = stage
        self._publish_status(stage, feedback)

    def _publish_status(self, stage, feedback):
        status = DiagnosticStatus()
        status.name = "agt_coverage/task_server"
        status.hardware_id = "agt_interfaces/ExecuteCoverageTask"
        status.message = stage
        status.level = (
            DiagnosticStatus.OK
            if stage in {"READY", "EXECUTING", "COMPLETED"}
            else DiagnosticStatus.ERROR
            if stage == "FAILED"
            else DiagnosticStatus.WARN
        )
        status.values = [
            KeyValue(key="current_swath_index", value=str(feedback.current_swath_index)),
            KeyValue(key="total_swaths", value=str(feedback.total_swaths)),
            KeyValue(key="distance_remaining", value=f"{feedback.distance_remaining:.6f}"),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.status_publisher.publish(message)

    def _semantic_loaded(self, expected_path=None):
        if self.semantic_status is None or self.semantic_status[0] != "LOADED":
            return False
        if expected_path is None:
            return True
        observed = self.semantic_status[1].get("semantic_path", "")
        return bool(observed) and _normalized_resource(observed) == expected_path

    def _safety_ready(self):
        if time.monotonic() - self.safety_received_at > self.safety_timeout:
            return False
        if self.safety_status is None:
            return False
        _state, values = self.safety_status
        return all(
            (
                values.get("motion_enabled") == "true",
                values.get("estop_latched") == "false",
            )
        )

    def _executed_length(self):
        if self._progress_model is None:
            return 0.0
        return max(0.0, self._progress_model.total_length - self._distance_remaining)

    def _semantic_status_callback(self, message):
        parsed = _diagnostic(message, "agt_semantic_map_server")
        if parsed is not None:
            self.semantic_status = parsed
            self.semantic_status_generation += 1

    def _keepout_callback(self, message):
        self.keepout_mask = deepcopy(message)
        self.keepout_generation += 1

    def _coverage_status_callback(self, message):
        parsed = _diagnostic(message, "agt_coverage_request_adapter")
        if parsed is not None:
            self.coverage_status = parsed
            self.coverage_status_generation += 1

    def _semantics_callback(self, message):
        try:
            self.semantics_document = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.semantics_document = None
        self.semantics_generation += 1

    def _validation_callback(self, message):
        try:
            self.validation_report = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.validation_report = None
        self.validation_generation += 1

    def _repair_callback(self, message):
        try:
            self.repair_report = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.repair_report = None
        self.repair_generation += 1

    def _validated_path_callback(self, message):
        self.validated_path = deepcopy(message)
        self.validated_path_generation += 1

    def _repaired_path_callback(self, message):
        self.repaired_path = deepcopy(message)
        self.repaired_path_generation += 1

    def _reconstructed_path_callback(self, message):
        self.reconstructed_path = deepcopy(message)
        self.reconstructed_path_generation += 1

    def _safety_callback(self, message):
        parsed = _diagnostic(message, self.safety_status_name)
        if parsed is not None:
            self.safety_status = parsed
            self.safety_received_at = time.monotonic()


def _positive_parameter(node, name):
    value = float(node.get_parameter(name).value)
    if not math.isfinite(value) or value <= 0.0:
        raise RuntimeError(f"{name} must be positive")
    return value


def _nonnegative_parameter(node, name):
    value = float(node.get_parameter(name).value)
    if not math.isfinite(value) or value < 0.0:
        raise RuntimeError(f"{name} must be non-negative")
    return value


def _diagnostic(message, name):
    for status in message.status:
        if status.name == name:
            return status.message, {item.key: item.value for item in status.values}
    return None


def _normalized_resource(resource):
    parsed = urlparse(str(resource))
    if parsed.scheme not in {"", "file"}:
        raise CoverageTaskError(ERROR_MAP_LOAD, "only plain paths and file:// are supported")
    value = unquote(parsed.path) if parsed.scheme == "file" else str(resource)
    return str(Path(value).expanduser().resolve())


def _path_poses(message):
    if message is None or message.header.frame_id != "map":
        raise CoverageTaskError(ERROR_PATH_INVALID, "execution path frame must be map")
    output = []
    for stamped in message.poses:
        orientation = stamped.pose.orientation
        norm = math.sqrt(
            sum(
                value**2
                for value in (
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                )
            )
        )
        if norm <= 1e-9:
            raise CoverageTaskError(ERROR_PATH_INVALID, "path orientation is invalid")
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
        )
        output.append(
            Pose2D(
                float(stamped.pose.position.x),
                float(stamped.pose.position.y),
                yaw,
            )
        )
    return output


def _result(success, code, message, executed_length, repaired_count):
    result = ExecuteCoverageTask.Result()
    result.success = bool(success)
    result.error_code = int(code)
    result.message = str(message)
    result.coverage_rate = 0.0
    result.overlap_rate = 0.0
    result.executed_length = float(executed_length)
    result.repaired_segment_count = int(repaired_count)
    return result


def main(args=None):
    rclpy.init(args=args)
    node = CoverageTaskServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
