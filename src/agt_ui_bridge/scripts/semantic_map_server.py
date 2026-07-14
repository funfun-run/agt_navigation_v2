#!/usr/bin/env python3
"""Transactional ROS 2 server for validated agricultural semantic maps."""

from copy import deepcopy
from dataclasses import dataclass
import math
from pathlib import Path
from urllib.parse import unquote, urlparse

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from agt_ui_bridge.map_transform import MapGeometry
from agt_ui_bridge.platform_profile import (
    load_platform_profile,
    resolve_platform_profile,
)
from agt_ui_bridge.semantic_io import SemanticFileError, load_semantic_task
from agt_ui_bridge.semantic_rasterizer import rasterize_keepout_mask
from agt_ui_bridge.semantic_validation import ValidationContext, validate_task


LATCHED_QOS = QoSProfile(
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

FEATURE_COLORS = {
    "field_boundary": (0.18, 0.62, 0.45, 0.95),
    "exclusion_zone": (0.85, 0.36, 0.25, 0.95),
    "row_centerline": (0.90, 0.74, 0.33, 0.95),
    "entry_pose": (0.25, 0.51, 0.84, 0.95),
    "work_direction": (0.21, 0.66, 0.71, 0.95),
    "headland_zone": (0.82, 0.54, 0.25, 0.95),
    "keepout_zone": (0.73, 0.29, 0.38, 0.95),
}


@dataclass
class SemanticCandidate:
    task: object
    markers: MarkerArray
    mask: OccupancyGrid


class SemanticMapServer(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__(
            "agt_semantic_map_server", parameter_overrides=parameter_overrides
        )
        self.declare_parameter("semantic_map", "")
        self.declare_parameter("platform_profile", "")
        self.declare_parameter("base_map_topic", "/agt/map/global_occupancy")
        self.declare_parameter("auto_load", True)
        self.declare_parameter("minimum_boundary_clearance", 0.0)
        self.declare_parameter("outside_field_is_keepout", True)
        self.declare_parameter("mask_free_value", 0)
        self.declare_parameter("mask_occupied_value", 100)

        self.semantic_path = str(self.get_parameter("semantic_map").value)
        self.platform_profile_path = str(
            self.get_parameter("platform_profile").value
        )
        self.minimum_boundary_clearance = float(
            self.get_parameter("minimum_boundary_clearance").value
        )
        self.outside_field_is_keepout = bool(
            self.get_parameter("outside_field_is_keepout").value
        )
        self.mask_free_value = int(self.get_parameter("mask_free_value").value)
        self.mask_occupied_value = int(
            self.get_parameter("mask_occupied_value").value
        )
        self.base_map = None
        self.active_candidate = None
        self.last_status = None
        self._auto_load_pending = bool(
            self.get_parameter("auto_load").value and self.semantic_path
        )

        self.marker_publisher = self.create_publisher(
            MarkerArray, "/agt/map/semantic_markers", LATCHED_QOS
        )
        self.mask_publisher = self.create_publisher(
            OccupancyGrid, "/agt/map/keepout_mask", LATCHED_QOS
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray, "/agt/map/semantic_status", LATCHED_QOS
        )
        self.base_map_subscription = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("base_map_topic").value),
            self._base_map_callback,
            LATCHED_QOS,
        )
        self.load_service = self.create_service(
            LoadMap, "/agt/map/semantic/load", self._load_callback
        )
        self.reload_service = self.create_service(
            Trigger, "/agt/map/semantic/reload", self._reload_callback
        )
        self.validate_service = self.create_service(
            Trigger, "/agt/map/semantic/validate", self._validate_callback
        )
        self._publish_status("UNLOADED", "no semantic task is active")

    def _base_map_callback(self, message):
        self.base_map = deepcopy(message)
        if self._auto_load_pending:
            self._auto_load_pending = False
            self._load_and_activate(self.semantic_path)

    def _load_callback(self, request, response):
        try:
            path = _path_from_url(request.map_url)
        except ValueError as exc:
            self._publish_status("LOAD_FAILED", str(exc))
            response.result = LoadMap.Response.RESULT_INVALID_MAP_METADATA
            return response
        success, state, message = self._load_and_activate(path)
        response.result = (
            LoadMap.Response.RESULT_SUCCESS
            if success
            else _load_map_result(state)
        )
        if success:
            response.map = deepcopy(self.base_map)
        return response

    def _reload_callback(self, _request, response):
        if not self.semantic_path:
            response.success = False
            response.message = "semantic_map parameter is empty"
            self._publish_status("UNLOADED", response.message)
            return response
        response.success, _state, response.message = self._load_and_activate(
            self.semantic_path
        )
        return response

    def _validate_callback(self, _request, response):
        if not self.semantic_path:
            response.success = False
            response.message = "semantic_map parameter is empty"
            return response
        success, state, message, _candidate = self._prepare_candidate(
            self.semantic_path
        )
        response.success = success
        response.message = message
        if not success:
            self._publish_status(state, message)
        return response

    def _load_and_activate(self, semantic_path):
        success, state, message, candidate = self._prepare_candidate(semantic_path)
        if not success:
            self._publish_status(state, message)
            return False, state, message

        self.active_candidate = candidate
        self.semantic_path = str(candidate.task.semantic_path)
        self.marker_publisher.publish(candidate.markers)
        self.mask_publisher.publish(candidate.mask)
        self._publish_status(
            "LOADED",
            "semantic task loaded and keepout mask rasterized",
        )
        return True, "LOADED", "semantic task loaded"

    def _prepare_candidate(self, semantic_path):
        if self.base_map is None:
            return False, "UNLOADED", "waiting for base OccupancyGrid", None
        try:
            task = load_semantic_task(semantic_path)
        except (KeyError, OSError, TypeError, ValueError, SemanticFileError) as exc:
            return False, "LOAD_FAILED", str(exc), None

        if task.read_only:
            state = (
                "HASH_MISMATCH"
                if "base_map_hash_mismatch" in task.warnings
                else "GEOMETRY_INVALID"
            )
            return False, state, ", ".join(task.warnings), None

        try:
            profile_path = resolve_platform_profile(
                task.coverage.robot_profile,
                explicit_path=self.platform_profile_path or None,
            )
            if profile_path is None:
                raise FileNotFoundError(
                    f"platform profile not found: {task.coverage.robot_profile}"
                )
            platform = load_platform_profile(profile_path)
        except (KeyError, OSError, TypeError, ValueError) as exc:
            return False, "GEOMETRY_INVALID", str(exc), None

        if platform["name"] != task.coverage.robot_profile:
            return False, "GEOMETRY_INVALID", "robot profile name mismatch", None
        if not math.isclose(
            platform["robot_width"], task.coverage.robot_width, abs_tol=1e-6
        ):
            return False, "GEOMETRY_INVALID", "robot width differs from profile", None

        expected_geometry = MapGeometry.from_nav2_yaml(task.base_map_path)
        context = ValidationContext(
            map_geometry=expected_geometry,
            navigation_footprint=tuple(
                tuple(point) for point in platform["footprint"]
            ),
            minimum_boundary_clearance=self.minimum_boundary_clearance,
            base_map_path=task.base_map_path,
        )
        report = validate_task(task.semantic_map, task.coverage, context=context)
        if not report.valid:
            codes = ", ".join(
                f"{issue.code}[{issue.object_id}]" for issue in report.issues
            )
            state = (
                "HASH_MISMATCH"
                if any(
                    issue.code == "base_map_hash_mismatch"
                    for issue in report.issues
                )
                else "GEOMETRY_INVALID"
            )
            return False, state, codes, None
        if not _occupancy_matches_geometry(self.base_map, context.map_geometry):
            return False, "GEOMETRY_INVALID", "base map metadata mismatch", None

        try:
            markers = self._build_markers(task.semantic_map)
            mask = self._build_keepout_mask(task.semantic_map, expected_geometry)
        except Exception as exc:
            return False, "RASTERIZATION_FAILED", str(exc), None
        return True, "LOADED", "semantic task is valid", SemanticCandidate(
            task=task,
            markers=markers,
            mask=mask,
        )

    def _build_markers(self, semantic_map):
        now = self.get_clock().now().to_msg()
        output = MarkerArray()
        clear = Marker()
        clear.header.frame_id = "map"
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        output.markers.append(clear)

        for marker_id, feature in enumerate(semantic_map.features, start=1):
            if not feature.enabled:
                continue
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = now
            marker.ns = feature.feature_type
            marker.id = marker_id
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                FEATURE_COLORS[feature.feature_type]
            )
            if feature.geometry_type == "Point":
                marker.type = Marker.ARROW
                marker.scale.x = 0.08
                marker.scale.y = 0.16
                marker.scale.z = 0.18
                yaw = float(feature.properties["yaw"])
                start = feature.coordinates
                end = [
                    start[0] + math.cos(yaw),
                    start[1] + math.sin(yaw),
                ]
                marker.points = [_point(start), _point(end)]
            else:
                marker.type = Marker.LINE_STRIP
                marker.scale.x = 0.08
                coordinates = (
                    feature.coordinates[0]
                    if feature.geometry_type == "Polygon"
                    else feature.coordinates
                )
                marker.points = [_point(coordinate) for coordinate in coordinates]
            output.markers.append(marker)
        return output

    def _build_keepout_mask(self, semantic_map, map_geometry):
        rasterized = rasterize_keepout_mask(
            semantic_map,
            map_geometry,
            outside_field_is_keepout=self.outside_field_is_keepout,
            free_value=self.mask_free_value,
            occupied_value=self.mask_occupied_value,
        )
        mask = OccupancyGrid()
        mask.header = deepcopy(self.base_map.header)
        mask.header.frame_id = "map"
        mask.header.stamp = self.get_clock().now().to_msg()
        mask.info = deepcopy(self.base_map.info)
        mask.info.map_load_time = mask.header.stamp
        mask.data = list(rasterized.data)
        return mask

    def _publish_status(self, state, detail):
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "agt_semantic_map_server"
        status.hardware_id = "semantic_map_files"
        status.level = (
            DiagnosticStatus.OK
            if state == "LOADED"
            else DiagnosticStatus.STALE
            if state == "UNLOADED"
            else DiagnosticStatus.ERROR
        )
        status.message = state
        active_map_id = (
            self.active_candidate.task.semantic_map.map_id
            if self.active_candidate is not None
            else ""
        )
        status.values = [
            KeyValue(key="detail", value=detail),
            KeyValue(key="semantic_path", value=self.semantic_path),
            KeyValue(key="active_map_id", value=active_map_id),
            KeyValue(key="mask_mode", value="semantic_keepout_task06"),
            KeyValue(
                key="outside_field_is_keepout",
                value=str(self.outside_field_is_keepout).lower(),
            ),
        ]
        message.status = [status]
        self.last_status = message
        self.status_publisher.publish(message)


def _point(coordinate):
    point = Point()
    point.x = float(coordinate[0])
    point.y = float(coordinate[1])
    return point


def _path_from_url(resource):
    parsed = urlparse(resource)
    if parsed.scheme not in {"", "file"}:
        raise ValueError("semantic load supports plain paths and file:// URLs")
    path = unquote(parsed.path) if parsed.scheme == "file" else resource
    if not path:
        raise ValueError("semantic map path is empty")
    return str(Path(path).expanduser().resolve())


def _geometry_from_occupancy(message):
    orientation = message.info.origin.orientation
    yaw = math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
    )
    return MapGeometry(
        resolution=float(message.info.resolution),
        width=int(message.info.width),
        height=int(message.info.height),
        origin_x=float(message.info.origin.position.x),
        origin_y=float(message.info.origin.position.y),
        origin_yaw=yaw,
        frame_id=message.header.frame_id or "map",
    )


def _occupancy_matches_geometry(message, geometry):
    observed = _geometry_from_occupancy(message)
    return (
        observed.frame_id == geometry.frame_id == "map"
        and observed.width == geometry.width
        and observed.height == geometry.height
        and math.isclose(observed.resolution, geometry.resolution, abs_tol=1e-9)
        and math.isclose(observed.origin_x, geometry.origin_x, abs_tol=1e-9)
        and math.isclose(observed.origin_y, geometry.origin_y, abs_tol=1e-9)
        and math.isclose(observed.origin_yaw, geometry.origin_yaw, abs_tol=1e-9)
    )


def _load_map_result(state):
    if state == "LOAD_FAILED":
        return LoadMap.Response.RESULT_MAP_DOES_NOT_EXIST
    if state in {"HASH_MISMATCH", "GEOMETRY_INVALID"}:
        return LoadMap.Response.RESULT_INVALID_MAP_DATA
    return LoadMap.Response.RESULT_UNDEFINED_FAILURE


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
