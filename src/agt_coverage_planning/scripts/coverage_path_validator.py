#!/usr/bin/env python3
"""ROS 2 node validating raw coverage paths against the global costmap."""

from copy import deepcopy
import json
import math

from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from agt_coverage_planning.path_validator import (
    GridMap,
    PathValidationError,
    Pose2D,
    ValidationReport,
    ValidatorConfig,
    footprint_shape_matches,
    validate_path,
)
from agt_coverage_planning.path_semantics import (
    PathSemanticsError,
    parse_path_semantics,
)
from agt_ui_bridge.platform_profile import load_platform_profile


LATCHED_QOS = QoSProfile(
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class CoveragePathValidator(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__(
            "coverage_path_validator", parameter_overrides=parameter_overrides
        )
        self.declare_parameter("platform_profile", "")
        self.declare_parameter("path_topic", "/agt/coverage/path_raw")
        self.declare_parameter("semantics_topic", "/agt/coverage/path_semantics")
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter(
            "footprint_topic", "/global_costmap/published_footprint"
        )
        self.declare_parameter("validation_frequency", 2.0)
        self.declare_parameter("occupied_cost_threshold", 65)
        self.declare_parameter("unknown_space_policy", "collision")
        self.declare_parameter("outside_costmap_is_collision", True)
        self.declare_parameter("published_footprint_tolerance", 0.03)
        self.declare_parameter("maximum_sample_count", 200000)
        self.declare_parameter("maximum_visualized_footprints", 500)

        profile_path = str(self.get_parameter("platform_profile").value)
        if not profile_path:
            raise RuntimeError("platform_profile parameter is required")
        platform = load_platform_profile(profile_path)
        self.footprint = tuple(tuple(point) for point in platform["footprint"])
        self.min_turning_radius = float(platform["min_turning_radius"])
        self.footprint_tolerance = float(
            self.get_parameter("published_footprint_tolerance").value
        )
        self.maximum_visualized_footprints = int(
            self.get_parameter("maximum_visualized_footprints").value
        )
        if not math.isfinite(self.footprint_tolerance) or self.footprint_tolerance < 0.0:
            raise RuntimeError("published_footprint_tolerance must be non-negative")
        if self.maximum_visualized_footprints < 0:
            raise RuntimeError("maximum_visualized_footprints must be non-negative")
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

        self.raw_path = None
        self.costmap = None
        self.published_footprint = None
        self.path_semantics = None
        self.dirty = False
        self.last_report_json = ""
        self.last_validated_path = None
        self.last_collision_poses = None
        self.last_footprint_markers = None

        self.validated_path_publisher = self.create_publisher(
            Path, "/agt/coverage/path_validated", LATCHED_QOS
        )
        self.collision_poses_publisher = self.create_publisher(
            PoseArray, "/agt/coverage/collision_poses", LATCHED_QOS
        )
        self.footprint_markers_publisher = self.create_publisher(
            MarkerArray, "/agt/coverage/footprint_markers", LATCHED_QOS
        )
        self.validation_report_publisher = self.create_publisher(
            String, "/agt/coverage/validation_report", LATCHED_QOS
        )

        self.create_subscription(
            Path,
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
            OccupancyGrid,
            str(self.get_parameter("costmap_topic").value),
            self._costmap_callback,
            LATCHED_QOS,
        )
        self.create_subscription(
            type(self)._footprint_message_type(),
            str(self.get_parameter("footprint_topic").value),
            self._footprint_callback,
            10,
        )
        frequency = float(self.get_parameter("validation_frequency").value)
        if not math.isfinite(frequency) or frequency <= 0.0:
            raise RuntimeError("validation_frequency must be positive")
        self.create_timer(1.0 / frequency, self._validate_if_ready)

    @staticmethod
    def _footprint_message_type():
        from geometry_msgs.msg import PolygonStamped

        return PolygonStamped

    def _path_callback(self, message):
        self.raw_path = deepcopy(message)
        self.dirty = True

    def _costmap_callback(self, message):
        self.costmap = deepcopy(message)
        self.dirty = True

    def _semantics_callback(self, message):
        self.path_semantics = deepcopy(message)
        self.dirty = True

    def _footprint_callback(self, message):
        self.published_footprint = deepcopy(message)
        self.dirty = True

    def _validate_if_ready(self):
        if not self.dirty:
            return
        if (
            self.raw_path is None
            or self.path_semantics is None
            or self.costmap is None
            or self.published_footprint is None
        ):
            return
        self.dirty = False
        try:
            self._validate_current_inputs()
        except (
            json.JSONDecodeError,
            KeyError,
            TypeError,
            ValueError,
            PathValidationError,
            PathSemanticsError,
        ) as exc:
            code = getattr(exc, "code", "validator_input_error")
            self._publish_failure(code)

    def _validate_current_inputs(self):
        if self.published_footprint.header.frame_id != "map":
            raise PathValidationError(
                "invalid_published_footprint_frame",
                "published footprint frame must be map",
            )
        runtime_footprint = tuple(
            (float(point.x), float(point.y))
            for point in self.published_footprint.polygon.points
        )
        if not footprint_shape_matches(
            self.footprint, runtime_footprint, self.footprint_tolerance
        ):
            raise PathValidationError(
                "published_footprint_profile_mismatch",
                "published footprint shape differs from the platform profile",
            )

        poses = []
        for stamped in self.raw_path.poses:
            if stamped.header.frame_id not in {"", "map"}:
                raise PathValidationError(
                    "invalid_path_pose_frame", "all path poses must use map frame"
                )
            poses.append(
                Pose2D(
                    x=float(stamped.pose.position.x),
                    y=float(stamped.pose.position.y),
                    yaw=_quaternion_yaw(stamped.pose.orientation),
                )
            )
        semantic_summary = parse_path_semantics(
            json.loads(self.path_semantics.data), poses, "map"
        )
        origin = self.costmap.info.origin
        grid = GridMap(
            width=int(self.costmap.info.width),
            height=int(self.costmap.info.height),
            resolution=float(self.costmap.info.resolution),
            origin_x=float(origin.position.x),
            origin_y=float(origin.position.y),
            origin_yaw=_quaternion_yaw(origin.orientation),
            data=tuple(int(value) for value in self.costmap.data),
            frame_id=self.costmap.header.frame_id,
        )
        result = validate_path(
            poses,
            self.raw_path.header.frame_id,
            grid,
            self.footprint,
            self.min_turning_radius,
            self.validator_config,
        )
        _annotate_report_with_semantics(result.report, semantic_summary)
        self._publish_result(result)

    def _publish_result(self, result):
        stamp = self.get_clock().now().to_msg()
        validated = Path()
        validated.header.frame_id = "map"
        validated.header.stamp = stamp
        if result.report.valid:
            validated = deepcopy(self.raw_path)
            validated.header.frame_id = "map"
            validated.header.stamp = stamp
        self.last_validated_path = validated
        self.validated_path_publisher.publish(validated)

        collisions = PoseArray()
        collisions.header.frame_id = "map"
        collisions.header.stamp = stamp
        collisions.poses = [_pose(sample.pose) for sample in result.collision_samples]
        self.last_collision_poses = collisions
        self.collision_poses_publisher.publish(collisions)
        markers = _footprint_markers(
            result.invalid_samples,
            self.footprint,
            stamp,
            self.maximum_visualized_footprints,
        )
        self.last_footprint_markers = markers
        self.footprint_markers_publisher.publish(markers)
        self._publish_report(result.report)

    def _publish_failure(self, code):
        stamp = self.get_clock().now().to_msg()
        empty_path = Path()
        empty_path.header.frame_id = "map"
        empty_path.header.stamp = stamp
        self.last_validated_path = empty_path
        self.validated_path_publisher.publish(empty_path)
        collision_poses = PoseArray()
        collision_poses.header.frame_id = "map"
        collision_poses.header.stamp = stamp
        self.last_collision_poses = collision_poses
        self.collision_poses_publisher.publish(collision_poses)
        markers = _clear_markers(stamp)
        self.last_footprint_markers = markers
        self.footprint_markers_publisher.publish(markers)
        self._publish_report(
            ValidationReport(
                valid=False,
                required_min_turning_radius=self.min_turning_radius,
                unknown_space_policy=self.validator_config.unknown_space_policy,
                error_codes=[str(code)],
            )
        )

    def _publish_report(self, report):
        message = String()
        message.data = report.to_json()
        self.last_report_json = message.data
        self.validation_report_publisher.publish(message)


def _quaternion_yaw(quaternion):
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(value) for value in values):
        raise PathValidationError(
            "invalid_orientation", "orientation must contain finite values"
        )
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-9:
        raise PathValidationError(
            "invalid_orientation", "orientation quaternion norm is zero"
        )
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _pose(pose):
    output = Pose()
    output.position.x = pose.x
    output.position.y = pose.y
    output.orientation = _yaw_quaternion(pose.yaw)
    return output


def _yaw_quaternion(yaw):
    output = Quaternion()
    output.z = math.sin(yaw * 0.5)
    output.w = math.cos(yaw * 0.5)
    return output


def _footprint_markers(samples, footprint, stamp, maximum_count):
    output = _clear_markers(stamp)
    for marker_id, sample in enumerate(samples[:maximum_count], start=1):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.ns = "invalid_coverage_footprint"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.04
        marker.color.r = 0.92
        marker.color.g = 0.18
        marker.color.b = 0.12
        marker.color.a = 0.85
        marker.points = [
            _point(world)
            for world in _transform_points(footprint, sample.pose)
        ]
        marker.points.append(deepcopy(marker.points[0]))
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


def _transform_points(points, pose):
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    return [
        (
            pose.x + cosine * x - sine * y,
            pose.y + sine * x + cosine * y,
        )
        for x, y in points
    ]


def _point(values):
    point = Point()
    point.x = values[0]
    point.y = values[1]
    return point


def _annotate_report_with_semantics(report, summary):
    report.path_fingerprint = summary.path_fingerprint
    report.swath_ids = list(summary.swath_ids)
    invalid_components = set()
    invalid_swaths = set()
    for segment_index in report.invalid_segment_indices:
        if segment_index < 0 or segment_index >= len(summary.segment_labels):
            raise PathSemanticsError(
                "invalid_segment_semantics_index",
                "validator segment index is outside semantic labels",
            )
        label = summary.segment_labels[segment_index]
        invalid_components.add(label.component_id)
        if label.swath_id:
            invalid_swaths.add(label.swath_id)
    report.invalid_component_ids = sorted(invalid_components)
    report.invalid_swath_ids = sorted(invalid_swaths)


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePathValidator()
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
