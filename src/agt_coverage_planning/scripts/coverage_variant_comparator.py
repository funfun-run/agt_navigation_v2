#!/usr/bin/env python3
"""Generate and rank multiple visualization-only coverage candidates."""

import json
import math
from pathlib import Path

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates
from rcl_interfaces.srv import SetParametersAtomically
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from agt_coverage_planning.coverage_adapter import prepare_coverage_request
from agt_coverage_planning.path_semantics import (
    PathSemanticsError,
    Pose2D,
    SwathInput,
    TurnInput,
    build_path_semantics,
)
from agt_coverage_planning.time_simulation import (
    MotionLimits,
    SimulationPose,
    simulate_path_time,
)
from agt_coverage_planning.variant_comparison import (
    VariantComparisonError,
    coverage_area_metrics,
    load_variants,
    rank_candidates,
)
from agt_ui_bridge.platform_profile import load_platform_profile
from agt_ui_bridge.semantic_io import load_semantic_task


LATCHED_QOS = QoSProfile(
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
COLORS = (
    (0.84, 0.18, 0.18),
    (0.05, 0.48, 0.78),
    (0.12, 0.65, 0.32),
    (0.95, 0.55, 0.08),
    (0.52, 0.24, 0.72),
    (0.05, 0.68, 0.68),
    (0.72, 0.42, 0.12),
    (0.30, 0.30, 0.30),
)


class CoverageVariantComparator(Node):
    def __init__(self):
        super().__init__("coverage_variant_comparator")
        self.declare_parameter("semantic_map", "")
        self.declare_parameter("platform_profile", "")
        self.declare_parameter("variants_file", "")
        self.declare_parameter("report_path", "")
        self.declare_parameter(
            "action_name", "/agt/coverage/polygon/compute_coverage_path"
        )
        self.declare_parameter(
            "server_node", "/agt/coverage/polygon/coverage_server"
        )
        self.declare_parameter("auto_compare", True)
        self.declare_parameter("semantic_swath_sample_step", 0.10)
        self.declare_parameter("semantic_absolute_length_tolerance", 0.05)
        self.declare_parameter("semantic_relative_length_tolerance", 0.005)

        self.action_client = ActionClient(
            self,
            ComputeCoveragePath,
            str(self.get_parameter("action_name").value),
        )
        self.parameter_client = self.create_client(
            SetParametersAtomically,
            f"{self.get_parameter('server_node').value}/set_parameters_atomically",
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/agt/coverage/comparison/markers", LATCHED_QOS
        )
        self.report_publisher = self.create_publisher(
            String, "/agt/coverage/comparison/report", LATCHED_QOS
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray, "/agt/coverage/comparison/status", LATCHED_QOS
        )
        self.create_service(Trigger, "/agt/coverage/compare", self._compare_callback)

        self.busy = False
        self.completed = False
        self.task = None
        self.base_spec = None
        self.variants = ()
        self.motion_limits = None
        self.candidates = []
        self.paths = []
        self.current_index = 0
        self.last_report = None
        self._publish_status("IDLE", "no comparison has been requested")
        self._auto_timer = None
        if bool(self.get_parameter("auto_compare").value):
            self._auto_timer = self.create_timer(0.5, self._auto_compare)

    def _auto_compare(self):
        if self.completed or self.busy:
            return
        started, _message = self._start_comparison()
        if started and self._auto_timer is not None:
            self._auto_timer.cancel()

    def _compare_callback(self, _request, response):
        response.success, response.message = self._start_comparison()
        return response

    def _start_comparison(self):
        if self.busy:
            return False, "coverage comparison is already running"
        if not self.action_client.server_is_ready():
            self._publish_status("WAITING_FOR_SERVER", "coverage action is not ready")
            return False, "coverage action server is not ready"
        if not self.parameter_client.service_is_ready():
            self._publish_status("WAITING_FOR_SERVER", "parameter service is not ready")
            return False, "coverage parameter service is not ready"
        try:
            self._load_inputs()
        except (KeyError, OSError, TypeError, ValueError) as exc:
            code = getattr(exc, "code", "comparison_input_invalid")
            self._publish_status("FAILED", str(exc), code)
            return False, f"{code}: {exc}"

        self.busy = True
        self.completed = False
        self.candidates = []
        self.paths = []
        self.current_index = 0
        self._publish_markers()
        self._publish_status("COMPARING", "synchronizing Coverage Server parameters")
        request = SetParametersAtomically.Request()
        request.parameters = [
            Parameter("robot_width", value=self.base_spec.robot_width).to_parameter_msg(),
            Parameter(
                "operation_width", value=self.base_spec.operation_width
            ).to_parameter_msg(),
            Parameter(
                "min_turning_radius", value=self.base_spec.min_turning_radius
            ).to_parameter_msg(),
        ]
        future = self.parameter_client.call_async(request)
        future.add_done_callback(self._parameters_done)
        return True, f"comparing {len(self.variants)} coverage variants"

    def _load_inputs(self):
        semantic_path = str(self.get_parameter("semantic_map").value)
        profile_path = str(self.get_parameter("platform_profile").value)
        variants_path = str(self.get_parameter("variants_file").value)
        if not semantic_path or not profile_path or not variants_path:
            raise VariantComparisonError(
                "comparison_paths_required",
                "semantic_map, platform_profile and variants_file are required",
            )
        self.task = load_semantic_task(semantic_path)
        platform = load_platform_profile(profile_path)
        self.base_spec = prepare_coverage_request(self.task, platform)
        if self.base_spec.planning_mode != "polygon":
            raise VariantComparisonError(
                "comparison_mode_unsupported",
                "the first comparison contract supports polygon planning only",
            )
        self.variants = load_variants(variants_path)
        self.motion_limits = _load_motion_limits(profile_path)

    def _parameters_done(self, future):
        try:
            result = future.result().result
        except Exception as exc:
            self._abort("parameter_update_failed", str(exc))
            return
        if not result.successful:
            self._abort("parameter_update_rejected", result.reason)
            return
        self._send_next()

    def _send_next(self):
        if self.current_index >= len(self.variants):
            self._finish()
            return
        variant = self.variants[self.current_index]
        self._publish_status(
            "COMPARING",
            f"planning {variant.variant_id} ({self.current_index + 1}/{len(self.variants)})",
        )
        goal = _build_goal(self.base_spec, variant)
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        variant = self.variants[self.current_index]
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._record_failure(variant, "goal_send_failed", str(exc))
            return
        if not goal_handle.accepted:
            self._record_failure(variant, "goal_rejected", "goal was rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_done)

    def _result_done(self, future):
        variant = self.variants[self.current_index]
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as exc:
            self._record_failure(variant, "result_failed", str(exc))
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or result.error_code != 0:
            self._record_failure(
                variant,
                "coverage_action_failed",
                f"Coverage Server returned error_code={result.error_code}",
            )
            return
        try:
            candidate = self._evaluate_result(variant, result)
        except (PathSemanticsError, VariantComparisonError, ValueError) as exc:
            self._record_failure(
                variant, getattr(exc, "code", "candidate_invalid"), str(exc)
            )
            return
        self.candidates.append(candidate)
        self.paths.append((variant, result.nav_path))
        self._publish_markers()
        self.current_index += 1
        self._send_next()

    def _evaluate_result(self, variant, result):
        poses = _path_poses(result.nav_path)
        if len(poses) < 2:
            raise VariantComparisonError("empty_coverage_path", "path is empty")
        semantics = None
        semantic_error = None
        try:
            semantics = build_path_semantics(
                poses,
                _swath_inputs(result.coverage_path),
                _turn_inputs(result.coverage_path),
                frame_id="map",
                contains_turns=bool(result.coverage_path.contains_turns),
                swaths_ordered=bool(result.coverage_path.swaths_ordered),
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
            semantic_error = exc.code

        simulation_poses = tuple(
            SimulationPose(pose.x, pose.y, pose.yaw) for pose in poses
        )
        if semantics is None:
            simulation = simulate_path_time(simulation_poses, self.motion_limits)
        else:
            simulation = simulate_path_time(
                simulation_poses,
                self.motion_limits,
                path_fingerprint=semantics.path_fingerprint,
                segment_types=tuple(
                    label.component_type for label in semantics.raw_labels
                ),
                component_ids=tuple(label.component_id for label in semantics.raw_labels),
            )
        candidate = {
            **variant.to_dict(),
            "status": "SUCCEEDED",
            "pose_count": len(poses),
            "planning_time": _duration_seconds(result.planning_time),
            "task_time": round(float(result.task_time), 9),
            **simulation.to_dict(),
            "semantic_status": "COMPLETE" if semantics is not None else "REJECTED",
            "semantic_error": semantic_error,
            "eligible_for_execution": False,
            "coverage_rate": None,
            "overlap_rate": None,
            "covered_area": None,
            "missed_area": None,
            "overlap_area": None,
            "target_area": None,
        }
        if semantics is not None:
            candidate.update(_area_metrics(self.task, result.coverage_path))
        return candidate

    def _record_failure(self, variant, code, detail):
        self.candidates.append(
            {
                **variant.to_dict(),
                "status": "FAILED",
                "error_code": str(code),
                "detail": str(detail),
                "eligible_for_execution": False,
                "coverage_rate": None,
                "overlap_rate": None,
            }
        )
        self.current_index += 1
        self._send_next()

    def _finish(self):
        ranking = rank_candidates(self.candidates)
        semantically_complete = [
            candidate["variant_id"]
            for candidate in self.candidates
            if candidate.get("semantic_status") == "COMPLETE"
        ]
        report = {
            "schema_version": "1.0",
            "status": "COMPLETED",
            "scope": "visualization_and_metrics_only",
            "map_id": self.task.semantic_map.map_id,
            "candidate_count": len(self.candidates),
            "successful_candidate_count": sum(
                candidate.get("status") == "SUCCEEDED"
                for candidate in self.candidates
            ),
            "semantic_complete_candidate_count": len(semantically_complete),
            "ranking_basis": ["estimated_motion_time", "total_path_length"],
            "geometric_ranking": list(ranking),
            "best_geometric_candidate_id": ranking[0] if ranking else None,
            "semantically_complete_candidate_ids": semantically_complete,
            "candidates": self.candidates,
        }
        self.last_report = report
        message = String()
        message.data = json.dumps(
            report, sort_keys=True, separators=(",", ":"), allow_nan=False
        )
        self.report_publisher.publish(message)
        report_path = str(self.get_parameter("report_path").value)
        if report_path:
            _atomic_write(report_path, message.data + "\n")
        self.busy = False
        self.completed = True
        summary = (
            f"compared {len(self.candidates)} variants; "
            f"best={report['best_geometric_candidate_id']}"
        )
        self._publish_status(
            "COMPLETED",
            summary,
        )

    def _abort(self, code, detail):
        self.busy = False
        self.completed = True
        self._publish_status("FAILED", detail, code)

    def _publish_markers(self):
        output = MarkerArray()
        clear = Marker()
        clear.header.frame_id = "map"
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        output.markers.append(clear)
        for index, (variant, path) in enumerate(self.paths):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = clear.header.stamp
            marker.ns = f"coverage_candidate_{variant.variant_id}"
            marker.id = index + 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.06
            color = COLORS[index % len(COLORS)]
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 0.90
            marker.points = [
                Point(x=pose.pose.position.x, y=pose.pose.position.y, z=0.03 + index * 0.01)
                for pose in path.poses
            ]
            output.markers.append(marker)
        self.marker_publisher.publish(output)

    def _publish_status(self, state, detail, error_code="none"):
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "agt_coverage_variant_comparator"
        status.hardware_id = "opennav_coverage_humble_v2"
        status.message = state
        status.level = (
            DiagnosticStatus.OK
            if state in {"IDLE", "COMPLETED"}
            else DiagnosticStatus.WARN
            if state in {"COMPARING", "WAITING_FOR_SERVER"}
            else DiagnosticStatus.ERROR
        )
        status.values = [
            KeyValue(key="detail", value=str(detail)),
            KeyValue(key="error_code", value=str(error_code)),
            KeyValue(key="execution_enabled", value="false"),
        ]
        message.status = [status]
        self.status_publisher.publish(message)


def _build_goal(spec, variant):
    goal = ComputeCoveragePath.Goal()
    goal.generate_headland = spec.generate_headland
    goal.generate_route = True
    goal.generate_path = True
    goal.frame_id = "map"
    goal.headland_mode.mode = "CONSTANT"
    goal.headland_mode.width = spec.headland_width
    goal.swath_mode.objective = spec.swath_objective
    goal.swath_mode.mode = spec.swath_mode
    goal.swath_mode.best_angle = (
        spec.swath_angle + variant.swath_angle_offset
    ) % math.pi
    goal.route_mode.mode = variant.route_mode
    goal.route_mode.spiral_n = variant.spiral_n
    goal.path_mode.mode = variant.path_mode
    goal.path_mode.continuity_mode = spec.path_continuity_mode
    goal.path_mode.turn_point_distance = 0.10
    goal.use_gml_file = False
    for ring in spec.polygons:
        polygon = Coordinates()
        polygon.coordinates = [
            Coordinate(axis1=point[0], axis2=point[1]) for point in ring
        ]
        goal.polygons.append(polygon)
    return goal


def _path_poses(path):
    if path.header.frame_id != "map":
        raise VariantComparisonError("invalid_result_frame", "path frame must be map")
    return tuple(_pose_2d(stamped) for stamped in path.poses)


def _pose_2d(stamped):
    if stamped.header.frame_id not in {"", "map"}:
        raise VariantComparisonError("invalid_pose_frame", "pose frame must be map")
    quaternion = stamped.pose.orientation
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    norm = math.sqrt(sum(value * value for value in values))
    if not all(math.isfinite(value) for value in values) or norm <= 1e-9:
        raise VariantComparisonError(
            "invalid_path_orientation", "path orientation is invalid"
        )
    x, y, z, w = (value / norm for value in values)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return Pose2D(float(stamped.pose.position.x), float(stamped.pose.position.y), yaw)


def _swath_inputs(components):
    return tuple(
        SwathInput(
            Pose2D(float(swath.start.x), float(swath.start.y), 0.0),
            Pose2D(float(swath.end.x), float(swath.end.y), 0.0),
        )
        for swath in components.swaths
    )


def _turn_inputs(components):
    return tuple(
        TurnInput(tuple(_pose_2d(stamped) for stamped in turn.poses))
        for turn in components.turns
    )


def _area_metrics(task, components):
    enabled = [feature for feature in task.semantic_map.features if feature.enabled]
    fields = [feature for feature in enabled if feature.feature_type == "field_boundary"]
    exclusions = [
        feature for feature in enabled if feature.feature_type == "exclusion_zone"
    ]
    return coverage_area_metrics(
        fields[0].coordinates[0],
        [feature.coordinates[0] for feature in exclusions],
        [
            ((swath.start.x, swath.start.y), (swath.end.x, swath.end.y))
            for swath in components.swaths
        ],
        task.coverage.operation_width,
    )


def _load_motion_limits(path):
    document = yaml.safe_load(Path(path).expanduser().resolve().read_text(encoding="utf-8"))
    values = document["platform"]["limits"]
    return MotionLimits(
        max_forward_velocity=float(values["max_forward_velocity"]),
        max_reverse_velocity=float(values["max_reverse_velocity"]),
        max_angular_velocity=float(values["max_angular_velocity"]),
        max_linear_acceleration=float(values["max_linear_acceleration"]),
        max_linear_deceleration=float(values["max_linear_deceleration"]),
        max_angular_acceleration=float(values["max_angular_acceleration"]),
        max_angular_deceleration=float(values["max_angular_deceleration"]),
    )


def _duration_seconds(duration):
    return round(float(duration.sec) + float(duration.nanosec) * 1e-9, 9)


def _atomic_write(path, text):
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(destination)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageVariantComparator()
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
