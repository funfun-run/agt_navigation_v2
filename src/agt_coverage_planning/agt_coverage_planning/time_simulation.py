"""Deterministic kinematic travel-time estimates for coverage paths."""

from dataclasses import asdict, dataclass
import math


EPSILON = 1e-9
MODEL_NAME = "curvature_limited_trapezoidal_v1"


class TimeSimulationError(ValueError):
    """Stable failure raised for invalid simulation inputs."""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = str(code)


@dataclass(frozen=True)
class SimulationPose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class MotionLimits:
    max_forward_velocity: float
    max_reverse_velocity: float
    max_angular_velocity: float
    max_linear_acceleration: float
    max_linear_deceleration: float
    max_angular_acceleration: float
    max_angular_deceleration: float


@dataclass(frozen=True)
class TimeSimulationReport:
    path_fingerprint: str
    pose_count: int
    segment_count: int
    total_path_length: float
    forward_path_length: float
    reverse_path_length: float
    work_path_length: float | None
    non_work_path_length: float | None
    pure_rotation_angle: float
    direction_change_count: int
    estimated_turn_count: int
    estimated_motion_time: float
    estimated_work_time: float | None
    estimated_non_work_time: float | None
    classification_source: str
    model: str = MODEL_NAME
    frame_id: str = "map"

    def to_dict(self):
        return _stable_values(asdict(self))


@dataclass(frozen=True)
class _Segment:
    distance: float
    yaw_change: float
    reverse: bool
    speed_limit: float
    pure_rotation: bool


def simulate_path_time(
    poses,
    limits,
    path_fingerprint="",
    segment_types=None,
    component_ids=None,
):
    """Estimate motion time while respecting velocity and acceleration limits."""
    poses = tuple(_validate_pose(pose) for pose in poses)
    limits = _validate_limits(limits)
    if len(poses) < 2:
        raise TimeSimulationError("path_too_short", "path requires at least two poses")

    segments = tuple(
        _segment(first, second, limits)
        for first, second in zip(poses, poses[1:])
    )
    if not any(segment.distance > EPSILON or segment.pure_rotation for segment in segments):
        raise TimeSimulationError("zero_motion_path", "path contains no motion")

    types, ids, classification_source = _validate_classification(
        len(segments), segment_types, component_ids
    )
    vertex_speeds = _vertex_speeds(segments, limits)
    segment_times = tuple(
        _segment_time(segment, vertex_speeds[index], vertex_speeds[index + 1], limits)
        for index, segment in enumerate(segments)
    )

    total_length = sum(segment.distance for segment in segments)
    reverse_length = sum(
        segment.distance for segment in segments if segment.reverse
    )
    direction_changes = sum(
        first.reverse != second.reverse
        for first, second in zip(segments, segments[1:])
        if first.distance > EPSILON and second.distance > EPSILON
    )
    turn_count = _turn_count(segments, types, ids)

    if types is None:
        work_length = None
        non_work_length = None
        work_time = None
        non_work_time = None
    else:
        work_length = sum(
            segment.distance
            for segment, segment_type in zip(segments, types)
            if segment_type == "SWATH"
        )
        work_time = sum(
            duration
            for duration, segment_type in zip(segment_times, types)
            if segment_type == "SWATH"
        )
        non_work_length = total_length - work_length
        non_work_time = sum(segment_times) - work_time

    return TimeSimulationReport(
        path_fingerprint=str(path_fingerprint),
        pose_count=len(poses),
        segment_count=len(segments),
        total_path_length=total_length,
        forward_path_length=total_length - reverse_length,
        reverse_path_length=reverse_length,
        work_path_length=work_length,
        non_work_path_length=non_work_length,
        pure_rotation_angle=sum(
            abs(segment.yaw_change) for segment in segments if segment.pure_rotation
        ),
        direction_change_count=direction_changes,
        estimated_turn_count=turn_count,
        estimated_motion_time=sum(segment_times),
        estimated_work_time=work_time,
        estimated_non_work_time=non_work_time,
        classification_source=classification_source,
    )


def _segment(first, second, limits):
    dx = second.x - first.x
    dy = second.y - first.y
    distance = math.hypot(dx, dy)
    yaw_change = _angle_difference(second.yaw, first.yaw)
    if distance <= EPSILON:
        return _Segment(0.0, yaw_change, False, 0.0, abs(yaw_change) > EPSILON)

    projection = dx * math.cos(first.yaw) + dy * math.sin(first.yaw)
    reverse = projection < 0.0
    speed_limit = (
        limits.max_reverse_velocity if reverse else limits.max_forward_velocity
    )
    curvature = abs(yaw_change) / distance
    if curvature > EPSILON:
        speed_limit = min(speed_limit, limits.max_angular_velocity / curvature)
    return _Segment(distance, yaw_change, reverse, speed_limit, False)


def _vertex_speeds(segments, limits):
    count = len(segments) + 1
    caps = [math.inf] * count
    for index, segment in enumerate(segments):
        if segment.distance <= EPSILON:
            caps[index] = 0.0
            caps[index + 1] = 0.0
            continue
        caps[index] = min(caps[index], segment.speed_limit)
        caps[index + 1] = min(caps[index + 1], segment.speed_limit)
    caps[0] = 0.0
    caps[-1] = 0.0
    for index in range(1, count - 1):
        previous = segments[index - 1]
        following = segments[index]
        if (
            previous.distance > EPSILON
            and following.distance > EPSILON
            and previous.reverse != following.reverse
        ):
            caps[index] = 0.0

    speeds = [0.0 if not math.isfinite(cap) else cap for cap in caps]
    speeds[0] = 0.0
    for index, segment in enumerate(segments):
        if segment.distance <= EPSILON:
            speeds[index + 1] = 0.0
            continue
        reachable = math.sqrt(
            speeds[index] ** 2
            + 2.0 * limits.max_linear_acceleration * segment.distance
        )
        speeds[index + 1] = min(speeds[index + 1], reachable)
    speeds[-1] = 0.0
    for index in range(len(segments) - 1, -1, -1):
        segment = segments[index]
        if segment.distance <= EPSILON:
            speeds[index] = 0.0
            continue
        reachable = math.sqrt(
            speeds[index + 1] ** 2
            + 2.0 * limits.max_linear_deceleration * segment.distance
        )
        speeds[index] = min(speeds[index], reachable)
    return tuple(speeds)


def _segment_time(segment, start_speed, end_speed, limits):
    if segment.pure_rotation:
        return _bounded_motion_time(
            abs(segment.yaw_change),
            limits.max_angular_velocity,
            limits.max_angular_acceleration,
            limits.max_angular_deceleration,
        )
    if segment.distance <= EPSILON:
        return 0.0
    return _bounded_motion_time(
        segment.distance,
        segment.speed_limit,
        limits.max_linear_acceleration,
        limits.max_linear_deceleration,
        start_speed,
        end_speed,
    )


def _bounded_motion_time(distance, maximum_speed, acceleration, deceleration, start=0.0, end=0.0):
    peak_squared = (
        2.0 * acceleration * deceleration * distance
        + deceleration * start * start
        + acceleration * end * end
    ) / (acceleration + deceleration)
    peak = min(maximum_speed, math.sqrt(max(0.0, peak_squared)))
    acceleration_time = max(0.0, peak - start) / acceleration
    deceleration_time = max(0.0, peak - end) / deceleration
    acceleration_distance = max(0.0, peak * peak - start * start) / (2.0 * acceleration)
    deceleration_distance = max(0.0, peak * peak - end * end) / (2.0 * deceleration)
    cruise_distance = max(0.0, distance - acceleration_distance - deceleration_distance)
    return acceleration_time + deceleration_time + cruise_distance / maximum_speed


def _turn_count(segments, segment_types, component_ids):
    if segment_types is not None:
        connections = []
        for index, segment_type in enumerate(segment_types):
            if segment_type != "CONNECTION":
                continue
            component = component_ids[index] if component_ids else f"segment_{index}"
            if not connections or connections[-1] != component:
                connections.append(component)
        return len(connections)

    turning = [
        segment.pure_rotation
        or (
            segment.distance > EPSILON
            and abs(segment.yaw_change) / segment.distance >= 0.5
        )
        for segment in segments
    ]
    return sum(
        value and (index == 0 or not turning[index - 1])
        for index, value in enumerate(turning)
    )


def _validate_classification(segment_count, segment_types, component_ids):
    if segment_types is None:
        if component_ids is not None:
            raise TimeSimulationError(
                "classification_mismatch", "component IDs require segment types"
            )
        return None, None, "geometric_fallback"
    types = tuple(str(value) for value in segment_types)
    if len(types) != segment_count or any(
        value not in {"SWATH", "CONNECTION"} for value in types
    ):
        raise TimeSimulationError(
            "invalid_segment_types", "every path segment requires SWATH or CONNECTION"
        )
    ids = tuple(str(value) for value in component_ids) if component_ids is not None else None
    if ids is not None and (len(ids) != segment_count or any(not value for value in ids)):
        raise TimeSimulationError(
            "invalid_component_ids", "every path segment requires a component ID"
        )
    return types, ids, "path_semantics"


def _validate_pose(pose):
    if not isinstance(pose, SimulationPose) or not all(
        math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)
    ):
        raise TimeSimulationError("invalid_pose", "poses must be finite SimulationPose values")
    return pose


def _validate_limits(limits):
    if not isinstance(limits, MotionLimits) or not all(
        math.isfinite(value) and value > 0.0 for value in asdict(limits).values()
    ):
        raise TimeSimulationError(
            "invalid_motion_limits", "all motion limits must be finite and positive"
        )
    return limits


def _angle_difference(first, second):
    return math.atan2(math.sin(first - second), math.cos(first - second))


def _stable_values(value):
    if isinstance(value, dict):
        return {key: _stable_values(item) for key, item in value.items()}
    if isinstance(value, float):
        return round(value, 9)
    return value
