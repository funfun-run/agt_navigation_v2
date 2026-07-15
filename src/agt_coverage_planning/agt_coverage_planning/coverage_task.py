"""ROS-independent contracts for coverage task goals and progress feedback."""

from dataclasses import dataclass
import math

from .path_semantics import SWATH, path_fingerprint
from .path_validator import Pose2D


ALLOWED_PLANNING_MODES = {"polygon", "annotated_rows"}
ALLOWED_STAGES = {
    "LOADING",
    "VALIDATING_MAP",
    "PLANNING",
    "VALIDATING_PATH",
    "REPAIRING",
    "READY",
    "EXECUTING",
    "PAUSED",
    "COMPLETED",
    "FAILED",
    "CANCELED",
}

ERROR_NONE = 0
ERROR_INVALID_GOAL = 100
ERROR_MAP_LOAD = 110
ERROR_PLANNING = 120
ERROR_PATH_INVALID = 130
ERROR_REPAIR_DISALLOWED = 140
ERROR_REPAIR_FAILED = 141
ERROR_SAFETY_NOT_READY = 150
ERROR_EXECUTION = 160
ERROR_CANCELED = 170
ERROR_INTERNAL = 199


class CoverageTaskError(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = int(code)


@dataclass(frozen=True)
class TaskGoal:
    semantic_map_uri: str
    field_id: str
    planning_mode: str
    controller_id: str
    allow_repair: bool


@dataclass(frozen=True)
class ProgressModel:
    total_length: float
    swath_intervals: tuple

    @property
    def total_swaths(self):
        return len(self.swath_intervals)

    def swath_index(self, distance_remaining):
        if not self.swath_intervals:
            return 0
        remaining = min(max(float(distance_remaining), 0.0), self.total_length)
        travelled = self.total_length - remaining
        for index, (start, end) in enumerate(self.swath_intervals):
            if travelled <= end:
                return index
        return len(self.swath_intervals) - 1


def validate_task_goal(goal):
    values = TaskGoal(
        semantic_map_uri=str(goal.semantic_map_uri).strip(),
        field_id=str(goal.field_id).strip(),
        planning_mode=str(goal.planning_mode).strip(),
        controller_id=str(goal.controller_id).strip(),
        allow_repair=bool(goal.allow_repair),
    )
    if not values.semantic_map_uri:
        raise CoverageTaskError(ERROR_INVALID_GOAL, "semantic_map_uri is required")
    if not values.field_id:
        raise CoverageTaskError(ERROR_INVALID_GOAL, "field_id is required")
    if values.planning_mode not in ALLOWED_PLANNING_MODES:
        raise CoverageTaskError(
            ERROR_INVALID_GOAL,
            f"unsupported planning_mode: {values.planning_mode}",
        )
    if not values.controller_id:
        raise CoverageTaskError(ERROR_INVALID_GOAL, "controller_id is required")
    return values


def build_progress_model(final_path, semantic_document, reconstructed_path=None):
    poses = _validate_poses(final_path)
    cumulative = _cumulative_lengths(poses)
    intervals = []
    if path_fingerprint(poses, "map") == semantic_document.get("path_fingerprint"):
        labels = [
            segment
            for segment in semantic_document.get("raw_segments", [])
            if segment.get("component_type") == SWATH
        ]
        for segment in labels:
            start = int(segment["start_index"])
            end = int(segment["end_index"]) + 1
            _append_interval(intervals, cumulative, start, end, len(poses))
    else:
        reconstructed = _validate_poses(reconstructed_path or [])
        components = sorted(
            (
                component
                for component in semantic_document.get("components", [])
                if component.get("component_type") == SWATH
            ),
            key=lambda component: int(component["order_index"]),
        )
        search_from = 0
        for component in components:
            source = reconstructed[
                int(component["start_pose_index"]):
                int(component["end_pose_index"]) + 1
            ]
            start = _find_subsequence(poses, source, search_from)
            if start is None:
                raise CoverageTaskError(
                    ERROR_PATH_INVALID,
                    f"SWATH missing from final path: {component.get('component_id', '')}",
                )
            end = start + len(source) - 1
            _append_interval(intervals, cumulative, start, end, len(poses))
            search_from = end
    if not intervals:
        raise CoverageTaskError(ERROR_PATH_INVALID, "final path has no SWATH intervals")
    return ProgressModel(cumulative[-1], tuple(intervals))


def _append_interval(output, cumulative, start, end, pose_count):
    if start < 0 or end <= start or end >= pose_count:
        raise CoverageTaskError(ERROR_PATH_INVALID, "invalid SWATH interval")
    output.append((cumulative[start], cumulative[end]))


def _find_subsequence(path, expected, start):
    if not expected:
        return None
    for index in range(start, len(path) - len(expected) + 1):
        if path[index:index + len(expected)] == expected:
            return index
    return None


def _validate_poses(poses):
    output = list(poses)
    if len(output) < 2:
        raise CoverageTaskError(ERROR_PATH_INVALID, "path requires at least two poses")
    if not all(
        all(
            (
                isinstance(pose, Pose2D),
                all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)),
            )
        )
        for pose in output
    ):
        raise CoverageTaskError(ERROR_PATH_INVALID, "path contains invalid poses")
    return output


def _cumulative_lengths(poses):
    output = [0.0]
    for first, second in zip(poses, poses[1:]):
        output.append(
            output[-1] + math.hypot(second.x - first.x, second.y - first.y)
        )
    if output[-1] <= 1e-9:
        raise CoverageTaskError(ERROR_PATH_INVALID, "path length must be positive")
    return output
