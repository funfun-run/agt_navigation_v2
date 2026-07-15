#!/usr/bin/env python3
"""Repair only invalid coverage CONNECTION components through Nav2."""

from copy import deepcopy
import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PolygonStamped, PoseStamped, Quaternion
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from agt_coverage_planning.path_repair import (
    PathRepairError,
    RepairResult,
    apply_connection_repairs,
    failed_report,
    prepare_connection_repairs,
    repair_policy_from_profile,
    successful_report,
)
from agt_coverage_planning.path_validator import (
    GridMap,
    PathValidationError,
    Pose2D,
    ValidatorConfig,
    footprint_shape_matches,
    validate_path,
)
from agt_ui_bridge.platform_profile import load_platform_profile


LATCHED_QOS = QoSProfile(
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class CoveragePathRepair(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__(
            "coverage_path_repair", parameter_overrides=parameter_overrides
        )
        self.declare_parameter("platform_profile", "")
        self.declare_parameter("planner_action", "/compute_path_to_pose")
        self.declare_parameter(
            "path_topic", "/agt/coverage/path_reconstructed"
        )
        self.declare_parameter("semantics_topic", "/agt/coverage/path_semantics")
        self.declare_parameter(
            "validation_report_topic", "/agt/coverage/validation_report"
        )
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter(
            "footprint_topic", "/global_costmap/published_footprint"
        )
        self.declare_parameter(
            "semantic_status_topic", "/agt/map/semantic_status"
        )
        self.declare_parameter("keepout_mask_topic", "/agt/map/keepout_mask")
        self.declare_parameter("occupied_cost_threshold", 65)
        self.declare_parameter("unknown_space_policy", "collision")
        self.declare_parameter("outside_costmap_is_collision", True)
        self.declare_parameter("published_footprint_tolerance", 0.03)
        self.declare_parameter("repair_endpoint_tolerance", 0.25)
        self.declare_parameter("maximum_sample_count", 200000)

        profile_path = str(self.get_parameter("platform_profile").value)
        if not profile_path:
            raise RuntimeError("platform_profile parameter is required")
        profile_document = yaml.safe_load(
            Path(profile_path).read_text(encoding="utf-8")
        )
        try:
            self.repair_policy = repair_policy_from_profile(profile_document)
        except PathRepairError as exc:
            raise RuntimeError(f"{exc.code}: {exc}") from exc
        platform = load_platform_profile(profile_path)
        if platform["name"] != self.repair_policy.platform_name:
            raise RuntimeError("platform profile name mismatch")
        self.footprint = tuple(tuple(point) for point in platform["footprint"])
        if not math.isclose(
            float(platform["min_turning_radius"]),
            self.repair_policy.min_turning_radius,
            abs_tol=1e-9,
        ):
            raise RuntimeError("platform minimum turning radius mismatch")
        self.footprint_tolerance = float(
            self.get_parameter("published_footprint_tolerance").value
        )
        self.endpoint_tolerance = float(
            self.get_parameter("repair_endpoint_tolerance").value
        )
        if not math.isfinite(self.footprint_tolerance) or self.footprint_tolerance < 0:
            raise RuntimeError("published_footprint_tolerance must be non-negative")
        if not math.isfinite(self.endpoint_tolerance) or self.endpoint_tolerance < 0:
            raise RuntimeError("repair_endpoint_tolerance must be non-negative")
        self.validator_config = ValidatorConfig(
            occupied_cost_threshold=int(
                self.get_parameter("occupied_cost_threshold").value
            ),
            unknown_space_policy=str(
                self.get_parameter("unknown_space_policy").value
            ),
            outside_costmap_is_collision=bool(
                self.get_parameter("outside_costmap_is_collision").value
            ),
            maximum_sample_count=int(
                self.get_parameter("maximum_sample_count").value
            ),
        )

        self.path_message = None
        self.semantics_message = None
        self.validation_message = None
        self.costmap_message = None
        self.footprint_message = None
        self.semantic_status_message = None
        self.keepout_mask_message = None
        self.pending = None
        self.last_repaired_path = None
        self.last_report_json = ""

        self.repaired_path_publisher = self.create_publisher(
            NavPath, "/agt/coverage/path_repaired", LATCHED_QOS
        )
        self.repair_report_publisher = self.create_publisher(
            String, "/agt/coverage/repair_report", LATCHED_QOS
        )
        self.create_subscription(
            NavPath,
            str(self.get_parameter("path_topic").value),
            self._path_callback,
            LATCHED_QOS,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("semantics_topic").value),
            self._semantics_callback,
            LATCHED_QOS,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("validation_report_topic").value),
            self._validation_callback,
            LATCHED_QOS,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("costmap_topic").value),
            self._costmap_callback,
            LATCHED_QOS,
        )
        self.create_subscription(
            PolygonStamped,
            str(self.get_parameter("footprint_topic").value),
            self._footprint_callback,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter("semantic_status_topic").value),
            self._semantic_status_callback,
            LATCHED_QOS,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("keepout_mask_topic").value),
            self._keepout_mask_callback,
            LATCHED_QOS,
        )
        self.repair_service = self.create_service(
            Trigger, "/agt/coverage/repair", self._repair_callback
        )
        self.planner_action = ActionClient(
            self,
            ComputePathToPose,
            str(self.get_parameter("planner_action").value),
        )

    def _path_callback(self, message):
        self.path_message = deepcopy(message)

    def _semantics_callback(self, message):
        self.semantics_message = deepcopy(message)

    def _validation_callback(self, message):
        self.validation_message = deepcopy(message)

    def _costmap_callback(self, message):
        self.costmap_message = deepcopy(message)

    def _footprint_callback(self, message):
        self.footprint_message = deepcopy(message)

    def _semantic_status_callback(self, message):
        self.semantic_status_message = deepcopy(message)

    def _keepout_mask_callback(self, message):
        self.keepout_mask_message = deepcopy(message)

    def _repair_callback(self, _request, response):
        response.success, response.message = self._start_repair()
        return response

    def _start_repair(self):
        if self.pending is not None:
            return False, "coverage repair is already in progress"
        try:
            self._require_inputs()
            path = _path_poses(self.path_message)
            semantics = json.loads(self.semantics_message.data)
            validation = json.loads(self.validation_message.data)
            preparation = prepare_connection_repairs(path, semantics, validation)
            self._validate_runtime_footprint()
            current_validation = self._validate_poses(path)
        except (
            json.JSONDecodeError,
            KeyError,
            OSError,
            PathRepairError,
            PathValidationError,
            TypeError,
            ValueError,
        ) as exc:
            code = getattr(exc, "code", "repair_input_error")
            self._publish_failure(code, str(exc))
            return False, f"{code}: {exc}"

        if not preparation.targets:
            if not current_validation.report.valid:
                self._publish_failure(
                    "validation_report_stale",
                    "current costmap validation disagrees with the valid report",
                )
                return False, "current costmap validation disagrees with the report"
            result = _unchanged_result(path, preparation.preserved_swath_ids)
            report = successful_report(
                result,
                self.repair_policy.planner_id,
                time.monotonic(),
                current_validation.report.to_dict(),
            )
            self._publish_success(result.poses, report)
            return True, "path is already valid; no repair was required"
        if current_validation.report.valid:
            self._publish_failure(
                "validation_report_stale",
                "current validation no longer contains the reported invalid connection",
            )
            return False, "current validation no longer matches the invalid report"
        if not self.planner_action.server_is_ready():
            self._publish_failure(
                "planner_action_unavailable", "Nav2 ComputePathToPose is not ready"
            )
            return False, "Nav2 ComputePathToPose is not ready"

        self.pending = {
            "path": path,
            "semantics": semantics,
            "preparation": preparation,
            "replacements": {},
            "next_index": 0,
            "started_at": time.monotonic(),
        }
        self._send_next_goal()
        return True, f"repairing {len(preparation.targets)} connection components"

    def _send_next_goal(self):
        target = self.pending["preparation"].targets[self.pending["next_index"]]
        goal = ComputePathToPose.Goal()
        goal.start = _pose_stamped(
            self.pending["path"][target.start_pose_index], self.get_clock().now().to_msg()
        )
        goal.goal = _pose_stamped(
            self.pending["path"][target.end_pose_index], self.get_clock().now().to_msg()
        )
        goal.planner_id = self.repair_policy.planner_id
        goal.use_start = True
        future = self.planner_action.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        if self.pending is None:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._abort("repair_goal_send_failed", str(exc))
            return
        if not goal_handle.accepted:
            self._abort("repair_goal_rejected", "Nav2 rejected the repair goal")
            return
        future = goal_handle.get_result_async()
        future.add_done_callback(self._goal_result)

    def _goal_result(self, future):
        if self.pending is None:
            return
        try:
            wrapped = future.result()
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                raise PathRepairError(
                    "repair_planner_failed",
                    f"Nav2 planner returned action status {wrapped.status}",
                )
            candidate = _path_poses(wrapped.result.path)
            validation = self._validate_poses(candidate)
            if not validation.report.valid:
                raise PathRepairError(
                    "repair_candidate_invalid",
                    ",".join(validation.report.error_codes),
                )
        except (PathRepairError, PathValidationError, TypeError, ValueError) as exc:
            self._abort(getattr(exc, "code", "repair_result_error"), str(exc))
            return
        except Exception as exc:
            self._abort("repair_result_error", str(exc))
            return

        target = self.pending["preparation"].targets[self.pending["next_index"]]
        self.pending["replacements"][target.component_id] = candidate
        self.pending["next_index"] += 1
        if self.pending["next_index"] < len(self.pending["preparation"].targets):
            self._send_next_goal()
            return
        self._finish_repair()

    def _finish_repair(self):
        try:
            if not self._semantic_loaded():
                raise PathRepairError(
                    "semantic_map_not_loaded", "semantic status changed before completion"
                )
            if not self._source_products_unchanged():
                raise PathRepairError(
                    "repair_source_changed",
                    "coverage path or semantics changed while repair was running",
                )
            self._validate_runtime_footprint()
            result = apply_connection_repairs(
                self.pending["path"],
                self.pending["semantics"],
                self.pending["preparation"],
                self.pending["replacements"],
                endpoint_tolerance=self.endpoint_tolerance,
            )
            final_validation = self._validate_poses(result.poses)
            if not final_validation.report.valid:
                raise PathRepairError(
                    "repaired_path_invalid",
                    ",".join(final_validation.report.error_codes),
                )
            report = successful_report(
                result,
                self.repair_policy.planner_id,
                self.pending["started_at"],
                final_validation.report.to_dict(),
            )
        except (PathRepairError, PathValidationError, TypeError, ValueError) as exc:
            self._abort(getattr(exc, "code", "repair_finalize_error"), str(exc))
            return
        self.pending = None
        self._publish_success(result.poses, report)

    def _abort(self, code, detail):
        started_at = self.pending["started_at"] if self.pending else None
        self.pending = None
        self._publish_failure(code, detail, started_at=started_at)

    def _require_inputs(self):
        required = {
            "path_reconstructed": self.path_message,
            "path_semantics": self.semantics_message,
            "validation_report": self.validation_message,
            "global_costmap": self.costmap_message,
            "published_footprint": self.footprint_message,
            "semantic_status": self.semantic_status_message,
            "keepout_mask": self.keepout_mask_message,
        }
        missing = [name for name, value in required.items() if value is None]
        if missing:
            raise PathRepairError(
                "repair_inputs_missing", "missing inputs: " + ",".join(missing)
            )
        if not self._semantic_loaded():
            raise PathRepairError(
                "semantic_map_not_loaded", "semantic status must be LOADED"
            )

    def _semantic_loaded(self):
        return self.semantic_status_message is not None and any(
            status.message == "LOADED"
            for status in self.semantic_status_message.status
        )

    def _validate_runtime_footprint(self):
        if self.footprint_message.header.frame_id != "map":
            raise PathRepairError(
                "invalid_published_footprint_frame",
                "published footprint frame must be map",
            )
        runtime = tuple(
            (float(point.x), float(point.y))
            for point in self.footprint_message.polygon.points
        )
        if not footprint_shape_matches(
            self.footprint, runtime, self.footprint_tolerance
        ):
            raise PathRepairError(
                "published_footprint_profile_mismatch",
                "published footprint differs from platform profile",
            )

    def _validate_poses(self, poses):
        costmap_result = validate_path(
            poses,
            "map",
            _grid_map(self.costmap_message),
            self.footprint,
            self.repair_policy.min_turning_radius,
            self.validator_config,
        )
        if not costmap_result.report.valid:
            return costmap_result
        mask_config = ValidatorConfig(
            occupied_cost_threshold=65,
            unknown_space_policy="collision",
            outside_costmap_is_collision=True,
            maximum_sample_count=self.validator_config.maximum_sample_count,
        )
        mask_result = validate_path(
            poses,
            "map",
            _grid_map(self.keepout_mask_message),
            self.footprint,
            self.repair_policy.min_turning_radius,
            mask_config,
        )
        if not mask_result.report.valid:
            mask_result.report.error_codes = sorted(
                set(mask_result.report.error_codes) | {"semantic_keepout_collision"}
            )
            return mask_result
        costmap_result.report.minimum_clearance = min(
            costmap_result.report.minimum_clearance,
            mask_result.report.minimum_clearance,
        )
        costmap_result.report.maximum_cost = max(
            costmap_result.report.maximum_cost,
            mask_result.report.maximum_cost,
        )
        return costmap_result

    def _source_products_unchanged(self):
        try:
            same_path = _path_poses(self.path_message) == self.pending["path"]
            current_semantics = json.loads(self.semantics_message.data)
            same_semantics = current_semantics == self.pending["semantics"]
            return same_path and same_semantics
        except (json.JSONDecodeError, PathRepairError, TypeError, ValueError):
            return False

    def _publish_success(self, poses, report):
        message = _nav_path(poses, self.get_clock().now().to_msg())
        self.last_repaired_path = message
        self.repaired_path_publisher.publish(message)
        self._publish_report(report)

    def _publish_failure(self, code, detail, started_at=None):
        empty = _nav_path([], self.get_clock().now().to_msg())
        self.last_repaired_path = empty
        self.repaired_path_publisher.publish(empty)
        self._publish_report(
            failed_report(
                code,
                detail,
                planner_id=self.repair_policy.planner_id,
                started_at=started_at,
            )
        )

    def _publish_report(self, report):
        message = String()
        message.data = report.to_json()
        self.last_report_json = message.data
        self.repair_report_publisher.publish(message)


def _unchanged_result(poses, swath_ids):
    length = sum(
        math.hypot(second.x - first.x, second.y - first.y)
        for first, second in zip(poses, poses[1:])
    )
    return RepairResult(
        poses=list(poses),
        repaired_component_ids=[],
        preserved_swath_ids=list(swath_ids),
        original_length=length,
        repaired_length=length,
        swath_coordinates_unchanged=True,
    )


def _path_poses(message):
    if message.header.frame_id != "map":
        raise PathRepairError("invalid_repair_path_frame", "path frame must be map")
    output = []
    for stamped in message.poses:
        if stamped.header.frame_id not in {"", "map"}:
            raise PathRepairError(
                "invalid_repair_pose_frame", "path pose frame must be map"
            )
        output.append(
            Pose2D(
                float(stamped.pose.position.x),
                float(stamped.pose.position.y),
                _quaternion_yaw(stamped.pose.orientation),
            )
        )
    if len(output) < 2:
        raise PathRepairError("path_too_short", "path needs at least two poses")
    return output


def _nav_path(poses, stamp):
    message = NavPath()
    message.header.frame_id = "map"
    message.header.stamp = stamp
    message.poses = [_pose_stamped(pose, stamp) for pose in poses]
    return message


def _pose_stamped(pose, stamp):
    message = PoseStamped()
    message.header.frame_id = "map"
    message.header.stamp = stamp
    message.pose.position.x = float(pose.x)
    message.pose.position.y = float(pose.y)
    message.pose.orientation = _yaw_quaternion(pose.yaw)
    return message


def _yaw_quaternion(yaw):
    output = Quaternion()
    output.z = math.sin(yaw * 0.5)
    output.w = math.cos(yaw * 0.5)
    return output


def _quaternion_yaw(quaternion):
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(value) for value in values):
        raise PathRepairError(
            "invalid_repair_orientation", "orientation must be finite"
        )
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-9:
        raise PathRepairError(
            "invalid_repair_orientation", "orientation norm is zero"
        )
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _grid_map(message):
    origin = message.info.origin
    return GridMap(
        width=int(message.info.width),
        height=int(message.info.height),
        resolution=float(message.info.resolution),
        origin_x=float(origin.position.x),
        origin_y=float(origin.position.y),
        origin_yaw=_quaternion_yaw(origin.orientation),
        data=tuple(int(value) for value in message.data),
        frame_id=message.header.frame_id,
    )


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePathRepair()
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
