#!/usr/bin/env python3
"""ROS 2 adapter from validated semantic files to ComputeCoveragePath."""

import math
from pathlib import Path
import tempfile

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PolygonStamped, PoseStamped, Quaternion
from nav_msgs.msg import Path as NavPath
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates, PathComponents
from rcl_interfaces.srv import SetParametersAtomically
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from agt_coverage_planning.coverage_adapter import (
    CoverageAdapterError,
    prepare_coverage_request,
)
from agt_coverage_planning.path_semantics import (
    PathSemanticsError,
    Pose2D,
    SwathInput,
    TurnInput,
    build_path_semantics,
)
from agt_ui_bridge.platform_profile import (
    load_platform_profile,
    resolve_platform_profile,
)
from agt_ui_bridge.semantic_io import load_semantic_task


LATCHED_QOS = QoSProfile(
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

RESULT_ERRORS = {
    ComputeCoveragePath.Result.INTERNAL_F2C_ERROR: "internal_fields2cover_error",
    ComputeCoveragePath.Result.INVALID_MODE_SET: "invalid_mode_set",
    # Humble-v2 assigns both INVALID_REQUEST and INVALID_COORDS the value 803.
    ComputeCoveragePath.Result.INVALID_REQUEST: "invalid_request_or_coordinates",
}


class CoverageRequestAdapter(Node):
    def __init__(self):
        super().__init__("coverage_request_adapter")
        self.declare_parameter("semantic_map", "")
        self.declare_parameter("platform_profile", "")
        self.declare_parameter("expected_field_id", "")
        self.declare_parameter("expected_planning_mode", "")
        self.declare_parameter("plan_on_start", False)
        self.declare_parameter("semantic_swath_sample_step", 0.10)
        self.declare_parameter("semantic_absolute_length_tolerance", 0.05)
        self.declare_parameter("semantic_relative_length_tolerance", 0.005)
        self.declare_parameter(
            "polygon_action_name", "/agt/coverage/polygon/compute_coverage_path"
        )
        self.declare_parameter(
            "row_action_name", "/agt/coverage/rows/compute_coverage_path"
        )
        self.declare_parameter(
            "polygon_server_node", "/agt/coverage/polygon/coverage_server"
        )
        self.declare_parameter(
            "row_server_node", "/agt/coverage/rows/row_coverage_server"
        )

        self.semantic_path = str(self.get_parameter("semantic_map").value)
        self.profile_path = str(self.get_parameter("platform_profile").value)
        self.active_task = None
        self.active_spec = None
        self.active_semantics = None
        self.planning = False
        self._algorithm_field = None
        self._algorithm_planning_field = None
        self.last_status = None
        self._temporary_directory = tempfile.TemporaryDirectory(
            prefix="agt_coverage_"
        )

        self.path_publisher = self.create_publisher(
            NavPath, "/agt/coverage/path_raw", LATCHED_QOS
        )
        self.preview_path_publisher = self.create_publisher(
            NavPath, "/agt/coverage/path_preview", LATCHED_QOS
        )
        self.components_publisher = self.create_publisher(
            PathComponents, "/agt/coverage/path_components", LATCHED_QOS
        )
        self.reconstructed_path_publisher = self.create_publisher(
            NavPath, "/agt/coverage/path_reconstructed", LATCHED_QOS
        )
        self.path_semantics_publisher = self.create_publisher(
            String, "/agt/coverage/path_semantics", LATCHED_QOS
        )
        self.swaths_publisher = self.create_publisher(
            MarkerArray, "/agt/coverage/swaths", LATCHED_QOS
        )
        self.headland_publisher = self.create_publisher(
            MarkerArray, "/agt/coverage/headland", LATCHED_QOS
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray, "/agt/coverage/status", LATCHED_QOS
        )
        self.create_subscription(
            PolygonStamped,
            "/agt/coverage/polygon/coverage_server/field_boundary",
            self._field_boundary_callback,
            10,
        )
        self.create_subscription(
            PolygonStamped,
            "/agt/coverage/polygon/coverage_server/planning_field",
            self._planning_field_callback,
            10,
        )
        self.plan_service = self.create_service(
            Trigger, "/agt/coverage/plan", self._plan_callback
        )

        self.polygon_action = ActionClient(
            self,
            ComputeCoveragePath,
            str(self.get_parameter("polygon_action_name").value),
        )
        self.row_action = ActionClient(
            self,
            ComputeCoveragePath,
            str(self.get_parameter("row_action_name").value),
        )
        self.polygon_parameters = self.create_client(
            SetParametersAtomically,
            f"{self.get_parameter('polygon_server_node').value}/set_parameters_atomically",
        )
        self.row_parameters = self.create_client(
            SetParametersAtomically,
            f"{self.get_parameter('row_server_node').value}/set_parameters_atomically",
        )
        self._publish_status("IDLE", "no coverage request has been sent")

        self._auto_plan_timer = None
        if bool(self.get_parameter("plan_on_start").value):
            self._auto_plan_timer = self.create_timer(0.5, self._auto_plan)

    def destroy_node(self):
        self._temporary_directory.cleanup()
        return super().destroy_node()

    def _auto_plan(self):
        if self.planning:
            return
        success, _message = self._start_plan()
        if success:
            self._auto_plan_timer.cancel()

    def _plan_callback(self, _request, response):
        response.success, response.message = self._start_plan()
        return response

    def _start_plan(self):
        if self.planning:
            return False, "coverage planning is already in progress"
        try:
            task, platform, spec = self._prepare_request()
        except CoverageAdapterError as exc:
            self._publish_status("REJECTED", str(exc), exc.code, exc.object_id)
            return False, f"{exc.code}: {exc}"
        except (KeyError, OSError, TypeError, ValueError) as exc:
            self._publish_status("REJECTED", str(exc), "task_load_failed")
            return False, f"task_load_failed: {exc}"

        action_client, parameter_client = self._clients_for(spec.planning_mode)
        if not action_client.server_is_ready():
            self._publish_status(
                "WAITING_FOR_SERVER",
                f"{spec.planning_mode} coverage action is not ready",
                "action_server_unavailable",
            )
            return False, "coverage action server is not ready"
        if not parameter_client.service_is_ready():
            self._publish_status(
                "WAITING_FOR_SERVER",
                f"{spec.planning_mode} parameter service is not ready",
                "parameter_service_unavailable",
            )
            return False, "coverage parameter service is not ready"

        self.active_task = task
        self.active_spec = spec
        self.active_semantics = None
        empty_preview = NavPath()
        empty_preview.header.frame_id = "map"
        empty_preview.header.stamp = self.get_clock().now().to_msg()
        self.preview_path_publisher.publish(empty_preview)
        self.planning = True
        self._publish_status("PLANNING", "synchronizing coverage server parameters")
        request = SetParametersAtomically.Request()
        request.parameters = [
            Parameter("robot_width", value=spec.robot_width).to_parameter_msg(),
            Parameter("operation_width", value=spec.operation_width).to_parameter_msg(),
            Parameter(
                "min_turning_radius", value=spec.min_turning_radius
            ).to_parameter_msg(),
        ]
        future = parameter_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._parameters_done(completed, action_client, spec)
        )
        return True, "coverage planning request accepted"

    def _prepare_request(self):
        self.semantic_path = str(self.get_parameter("semantic_map").value)
        self.profile_path = str(self.get_parameter("platform_profile").value)
        if not self.semantic_path:
            raise CoverageAdapterError(
                "semantic_map_not_configured", "semantic_map parameter is empty"
            )
        task = load_semantic_task(self.semantic_path)
        profile_path = resolve_platform_profile(
            task.coverage.robot_profile,
            explicit_path=self.profile_path or None,
        )
        if profile_path is None:
            raise CoverageAdapterError(
                "platform_profile_not_found",
                f"platform profile not found: {task.coverage.robot_profile}",
            )
        platform = load_platform_profile(profile_path)
        spec = prepare_coverage_request(task, platform)
        expected_mode = str(self.get_parameter("expected_planning_mode").value)
        if expected_mode and spec.planning_mode != expected_mode:
            raise CoverageAdapterError(
                "planning_mode_goal_mismatch",
                f"semantic task uses {spec.planning_mode}, requested {expected_mode}",
            )
        expected_field = str(self.get_parameter("expected_field_id").value)
        enabled_fields = [
            feature.id
            for feature in task.semantic_map.features
            if feature.enabled and feature.feature_type == "field_boundary"
        ]
        if expected_field and enabled_fields != [expected_field]:
            raise CoverageAdapterError(
                "field_id_goal_mismatch",
                f"enabled field {enabled_fields} does not match {expected_field}",
                expected_field,
            )
        return task, platform, spec

    def _clients_for(self, planning_mode):
        if planning_mode == "annotated_rows":
            return self.row_action, self.row_parameters
        return self.polygon_action, self.polygon_parameters

    def _field_boundary_callback(self, message):
        self._algorithm_field = message
        self._publish_algorithm_headland()

    def _planning_field_callback(self, message):
        self._algorithm_planning_field = message
        self._publish_algorithm_headland()

    def _publish_algorithm_headland(self):
        stamp = self.get_clock().now().to_msg()
        output = _clear_markers(stamp)
        geometries = (
            ("coverage_field", self._algorithm_field, (0.20, 0.64, 0.43)),
            (
                "coverage_planning_field",
                self._algorithm_planning_field,
                (0.93, 0.53, 0.18),
            ),
        )
        for marker_id, (namespace, polygon, color) in enumerate(
            geometries, start=1
        ):
            if polygon is None:
                continue
            marker = _line_marker(namespace, marker_id, stamp, *color)
            marker.points = [_copy_point(point) for point in polygon.polygon.points]
            if marker.points:
                marker.points.append(_copy_point(marker.points[0]))
            output.markers.append(marker)
        self.headland_publisher.publish(output)

    def _parameters_done(self, future, action_client, spec):
        try:
            result = future.result().result
        except Exception as exc:
            self._fail("parameter_update_failed", str(exc))
            return
        if not result.successful:
            self._fail("parameter_update_rejected", result.reason)
            return

        goal = self._build_goal(spec)
        self.headland_publisher.publish(
            _headland_markers(self.active_task.semantic_map, self.get_clock().now().to_msg())
        )
        future = action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _build_goal(self, spec):
        goal = ComputeCoveragePath.Goal()
        goal.generate_headland = spec.generate_headland
        goal.generate_route = True
        goal.generate_path = True
        goal.frame_id = "map"
        goal.headland_mode.mode = "CONSTANT"
        goal.headland_mode.width = spec.headland_width
        goal.route_mode.mode = spec.route_mode
        goal.path_mode.mode = spec.path_mode
        goal.path_mode.continuity_mode = spec.path_continuity_mode
        goal.path_mode.turn_point_distance = 0.10

        if spec.planning_mode == "annotated_rows":
            gml_path = Path(self._temporary_directory.name) / "annotated_rows.gml"
            gml_path.write_text(spec.gml_text, encoding="utf-8")
            goal.use_gml_file = True
            goal.gml_field = str(gml_path)
            goal.row_swath_mode.mode = spec.row_swath_mode
        else:
            goal.use_gml_file = False
            goal.swath_mode.objective = spec.swath_objective
            goal.swath_mode.mode = spec.swath_mode
            goal.swath_mode.best_angle = spec.swath_angle
            for ring in spec.polygons:
                polygon = Coordinates()
                polygon.coordinates = [
                    Coordinate(axis1=point[0], axis2=point[1]) for point in ring
                ]
                goal.polygons.append(polygon)
        return goal

    def _goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._fail("goal_send_failed", str(exc))
            return
        if not goal_handle.accepted:
            self._fail("goal_rejected", "coverage server rejected the action goal")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_done)

    def _result_done(self, future):
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as exc:
            self._fail("result_failed", str(exc))
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or result.error_code != 0:
            code = RESULT_ERRORS.get(result.error_code, "coverage_action_failed")
            self._fail(code, f"coverage server returned error_code={result.error_code}")
            return
        error = _validate_result(result)
        if error is not None:
            self._fail(error[0], error[1])
            return
        self.preview_path_publisher.publish(result.nav_path)

        try:
            semantics, reconstructed_path, semantics_message = _semantic_products(
                result.nav_path,
                result.coverage_path,
                swath_sample_step=float(
                    self.get_parameter("semantic_swath_sample_step").value
                ),
                absolute_length_tolerance=float(
                    self.get_parameter("semantic_absolute_length_tolerance").value
                ),
                relative_length_tolerance=float(
                    self.get_parameter("semantic_relative_length_tolerance").value
                ),
            )
        except PathSemanticsError as exc:
            self._fail(
                exc.code,
                f"{exc}; unvalidated server path is available on "
                "/agt/coverage/path_preview",
            )
            return

        self.components_publisher.publish(result.coverage_path)
        self.reconstructed_path_publisher.publish(reconstructed_path)
        self.path_semantics_publisher.publish(semantics_message)
        self.path_publisher.publish(result.nav_path)
        self.swaths_publisher.publish(
            _swath_markers(result.coverage_path, self.get_clock().now().to_msg())
        )
        self.active_semantics = semantics
        self.planning = False
        self._publish_status(
            "SUCCEEDED",
            f"generated {len(result.nav_path.poses)} path poses",
            planning_time=_duration_seconds(result.planning_time),
        )

    def _fail(self, code, detail):
        self.planning = False
        self._publish_status("FAILED", detail, code)

    def _publish_status(
        self, state, detail, error_code="none", object_id="<document>", planning_time=None
    ):
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "agt_coverage_request_adapter"
        status.hardware_id = "opennav_coverage_humble_v2"
        status.level = (
            DiagnosticStatus.OK
            if state in {"IDLE", "SUCCEEDED"}
            else DiagnosticStatus.WARN
            if state in {"PLANNING", "WAITING_FOR_SERVER"}
            else DiagnosticStatus.ERROR
        )
        status.message = state
        status.values = [
            KeyValue(key="detail", value=str(detail)),
            KeyValue(key="error_code", value=str(error_code)),
            KeyValue(key="object_id", value=str(object_id)),
            KeyValue(
                key="planning_mode",
                value=self.active_spec.planning_mode if self.active_spec else "",
            ),
            KeyValue(key="frame_id", value="map"),
        ]
        if planning_time is not None:
            status.values.append(
                KeyValue(key="planning_time", value=f"{planning_time:.6f}")
            )
        if self.active_semantics is not None:
            swath_ids = self.active_semantics.swath_ids
            status.values.extend(
                [
                    KeyValue(key="swath_ids", value=",".join(swath_ids)),
                    KeyValue(key="swath_count", value=str(len(swath_ids))),
                    KeyValue(
                        key="connection_count",
                        value=str(
                            sum(
                                component.component_type == "CONNECTION"
                                for component in self.active_semantics.components
                            )
                        ),
                    ),
                    KeyValue(
                        key="reconstruction_length_error",
                        value=f"{self.active_semantics.length_error:.9f}",
                    ),
                ]
            )
        message.status = [status]
        self.last_status = message
        self.status_publisher.publish(message)


def _validate_result(result):
    if result.nav_path.header.frame_id != "map":
        return "invalid_result_frame", "coverage path frame_id must be map"
    if not result.nav_path.poses:
        return "empty_coverage_path", "coverage server returned an empty path"
    for index, stamped in enumerate(result.nav_path.poses):
        if stamped.header.frame_id not in {"", "map"}:
            return "invalid_pose_frame", f"path pose {index} is not in map frame"
        orientation = stamped.pose.orientation
        values = (orientation.x, orientation.y, orientation.z, orientation.w)
        norm = math.sqrt(sum(value * value for value in values))
        if not all(math.isfinite(value) for value in values) or norm < 1e-6:
            return "invalid_path_orientation", f"path pose {index} has invalid orientation"
    return None


def _semantic_products(
    nav_path,
    components,
    swath_sample_step=0.10,
    absolute_length_tolerance=0.05,
    relative_length_tolerance=0.005,
):
    if components.header.frame_id != "map":
        raise PathSemanticsError(
            "invalid_path_components_frame", "PathComponents frame must be map"
        )
    raw_poses = [_pose_2d(stamped) for stamped in nav_path.poses]
    swaths = [
        SwathInput(
            Pose2D(float(swath.start.x), float(swath.start.y), 0.0),
            Pose2D(float(swath.end.x), float(swath.end.y), 0.0),
        )
        for swath in components.swaths
    ]
    turns = []
    for turn in components.turns:
        if turn.header.frame_id not in {"", "map"}:
            raise PathSemanticsError(
                "invalid_connection_frame", "connection Path frame must be map"
            )
        turns.append(TurnInput(tuple(_pose_2d(stamped) for stamped in turn.poses)))
    semantics = build_path_semantics(
        raw_poses,
        swaths,
        turns,
        frame_id="map",
        contains_turns=bool(components.contains_turns),
        swaths_ordered=bool(components.swaths_ordered),
        swath_sample_step=swath_sample_step,
        absolute_length_tolerance=absolute_length_tolerance,
        relative_length_tolerance=relative_length_tolerance,
    )
    reconstructed = NavPath()
    reconstructed.header = nav_path.header
    reconstructed.header.frame_id = "map"
    reconstructed.poses = [
        _pose_stamped(pose, reconstructed.header) for pose in semantics.reconstructed_poses
    ]
    semantics_message = String()
    semantics_message.data = semantics.to_json()
    return semantics, reconstructed, semantics_message


def _pose_2d(stamped):
    if stamped.header.frame_id not in {"", "map"}:
        raise PathSemanticsError(
            "invalid_path_pose_frame", "path pose frame must be map"
        )
    orientation = stamped.pose.orientation
    values = (orientation.x, orientation.y, orientation.z, orientation.w)
    if not all(math.isfinite(value) for value in values):
        raise PathSemanticsError(
            "invalid_path_orientation", "path orientation must be finite"
        )
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-9:
        raise PathSemanticsError(
            "invalid_path_orientation", "path orientation norm is zero"
        )
    x, y, z, w = (value / norm for value in values)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return Pose2D(
        float(stamped.pose.position.x), float(stamped.pose.position.y), yaw
    )


def _pose_stamped(pose, header):
    stamped = PoseStamped()
    stamped.header = header
    stamped.pose.position.x = pose.x
    stamped.pose.position.y = pose.y
    stamped.pose.orientation = _yaw_quaternion(pose.yaw)
    return stamped


def _yaw_quaternion(yaw):
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw * 0.5)
    quaternion.w = math.cos(yaw * 0.5)
    return quaternion


def _swath_markers(components, stamp):
    output = _clear_markers(stamp)
    for marker_id, swath in enumerate(components.swaths, start=1):
        marker = _line_marker("coverage_swaths", marker_id, stamp, 0.90, 0.70, 0.18)
        marker.points = [_copy_point(swath.start), _copy_point(swath.end)]
        output.markers.append(marker)
    return output


def _headland_markers(semantic_map, stamp):
    output = _clear_markers(stamp)
    features = [
        feature
        for feature in semantic_map.features
        if feature.enabled and feature.feature_type == "headland_zone"
    ]
    for marker_id, feature in enumerate(features, start=1):
        marker = _line_marker("coverage_headland", marker_id, stamp, 0.84, 0.45, 0.18)
        marker.points = [_point(point) for point in feature.coordinates[0]]
        output.markers.append(marker)
    return output


def _clear_markers(stamp):
    output = MarkerArray()
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = stamp
    marker.action = Marker.DELETEALL
    output.markers.append(marker)
    return output


def _line_marker(namespace, marker_id, stamp, red, green, blue):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.08
    marker.color.r = red
    marker.color.g = green
    marker.color.b = blue
    marker.color.a = 0.95
    return marker


def _point(coordinate):
    point = Point()
    point.x = float(coordinate[0])
    point.y = float(coordinate[1])
    return point


def _copy_point(source):
    point = Point()
    point.x = float(source.x)
    point.y = float(source.y)
    point.z = float(source.z)
    return point


def _duration_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = CoverageRequestAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
