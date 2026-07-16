#!/usr/bin/env python3

"""Publish deterministic kinematic time estimates for an offline coverage path."""

import json
import math
import os
from pathlib import Path
import tempfile

from nav_msgs.msg import Path as NavPath
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import yaml

from agt_coverage_planning.path_semantics import (
    PathSemanticsError,
    parse_path_semantics,
    path_fingerprint,
)
from agt_coverage_planning.path_validator import Pose2D
from agt_coverage_planning.time_simulation import (
    MotionLimits,
    SimulationPose,
    TimeSimulationError,
    simulate_path_time,
)


LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class CoverageTimeSimulator(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__(
            "coverage_time_simulator", parameter_overrides=parameter_overrides
        )
        self.declare_parameter("path_topic", "/agt/coverage/path_preview")
        self.declare_parameter("semantics_topic", "/agt/coverage/path_semantics")
        self.declare_parameter("platform_profile", "")
        self.declare_parameter("report_topic", "/agt/coverage/simulation_report")
        self.declare_parameter("report_path", "")

        profile_path = str(self.get_parameter("platform_profile").value)
        self.motion_limits = _load_motion_limits(profile_path)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.report_path = str(self.get_parameter("report_path").value)
        self.semantic_document = None
        self.last_path = None
        self.last_report = None

        self.report_publisher = self.create_publisher(
            String, str(self.get_parameter("report_topic").value), LATCHED_QOS
        )
        self.create_subscription(
            NavPath, self.path_topic, self._path_callback, LATCHED_QOS
        )
        self.create_subscription(
            String,
            str(self.get_parameter("semantics_topic").value),
            self._semantics_callback,
            LATCHED_QOS,
        )

    def _path_callback(self, message):
        self.last_path = message
        self._simulate()

    def _semantics_callback(self, message):
        try:
            document = json.loads(message.data)
        except (TypeError, ValueError):
            self.semantic_document = None
        else:
            self.semantic_document = document
        if self.last_path is not None:
            self._simulate()

    def _simulate(self):
        try:
            poses = _poses_from_path(self.last_path)
            semantic_poses = tuple(Pose2D(pose.x, pose.y, pose.yaw) for pose in poses)
            fingerprint = path_fingerprint(semantic_poses, "map")
            segment_types = None
            component_ids = None
            semantic_error = "path_semantics_unavailable"
            if self.semantic_document is not None:
                try:
                    summary = parse_path_semantics(
                        self.semantic_document, semantic_poses, frame_id="map"
                    )
                except PathSemanticsError as exc:
                    semantic_error = exc.code
                else:
                    segment_types = [
                        label.component_type for label in summary.segment_labels
                    ]
                    component_ids = [
                        label.component_id for label in summary.segment_labels
                    ]
                    semantic_error = "none"
            report = simulate_path_time(
                poses,
                self.motion_limits,
                path_fingerprint=fingerprint,
                segment_types=segment_types,
                component_ids=component_ids,
            ).to_dict()
            report["source_topic"] = self.path_topic
            report["semantic_classification_error"] = semantic_error
            report["status"] = "ESTIMATED"
        except (KeyError, OSError, TimeSimulationError, ValueError) as exc:
            report = {
                "schema_version": "1.0",
                "status": "FAILED",
                "error_code": getattr(exc, "code", "simulation_failed"),
                "detail": str(exc),
                "source_topic": self.path_topic,
            }

        message = String()
        message.data = json.dumps(
            report, sort_keys=True, separators=(",", ":"), allow_nan=False
        )
        self.last_report = report
        self.report_publisher.publish(message)
        if self.report_path:
            _atomic_write(self.report_path, message.data + "\n")


def _load_motion_limits(path):
    if not path:
        raise ValueError("platform_profile parameter is required")
    profile_path = Path(path).expanduser().resolve()
    document = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
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


def _poses_from_path(message):
    if message is None or message.header.frame_id != "map":
        raise TimeSimulationError("invalid_path_frame", "path frame_id must be map")
    output = []
    for index, stamped in enumerate(message.poses):
        if stamped.header.frame_id not in {"", "map"}:
            raise TimeSimulationError(
                "invalid_pose_frame", f"path pose {index} is not in map frame"
            )
        orientation = stamped.pose.orientation
        values = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        norm = math.sqrt(sum(value * value for value in values))
        if not all(math.isfinite(value) for value in values) or norm <= 1e-9:
            raise TimeSimulationError(
                "invalid_path_orientation", f"path pose {index} orientation is invalid"
            )
        x, y, z, w = (value / norm for value in values)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        output.append(
            SimulationPose(
                float(stamped.pose.position.x),
                float(stamped.pose.position.y),
                yaw,
            )
        )
    return tuple(output)


def _atomic_write(path, text):
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=destination.parent,
            prefix=f".{destination.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary_path = Path(stream.name)
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, destination)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def main(args=None):
    rclpy.init(args=args)
    node = CoverageTimeSimulator()
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
