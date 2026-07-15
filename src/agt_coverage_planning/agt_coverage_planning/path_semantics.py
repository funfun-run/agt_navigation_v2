"""Deterministic SWATH/CONNECTION semantics for OpenNav path components."""

from bisect import bisect_left
from dataclasses import dataclass
import hashlib
import json
import math

from .path_validator import Pose2D


EPSILON = 1e-9
SWATH = "SWATH"
CONNECTION = "CONNECTION"


class PathSemanticsError(ValueError):
    """Stable failure raised when PathComponents cannot describe the raw path."""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = str(code)


@dataclass(frozen=True)
class SwathInput:
    start: Pose2D
    end: Pose2D


@dataclass(frozen=True)
class TurnInput:
    poses: tuple


@dataclass
class SemanticComponent:
    component_type: str
    component_id: str
    poses: tuple
    swath_id: str = ""
    order_index: int = 0
    start_pose_index: int = 0
    end_pose_index: int = 0

    @property
    def length(self):
        return _path_length(self.poses)


@dataclass(frozen=True)
class SegmentLabel:
    start_index: int
    end_index: int
    component_type: str
    component_id: str
    swath_id: str = ""


@dataclass(frozen=True)
class SemanticSummary:
    path_fingerprint: str
    swath_ids: tuple
    segment_labels: tuple


@dataclass
class PathSemanticsResult:
    components: list
    reconstructed_poses: list
    reconstructed_labels: list
    raw_labels: list
    path_fingerprint: str
    raw_path_length: float
    reconstructed_path_length: float
    length_error: float
    length_tolerance: float
    frame_id: str = "map"

    @property
    def swath_ids(self):
        return tuple(
            component.swath_id
            for component in self.components
            if component.component_type == SWATH
        )

    def to_dict(self):
        return {
            "schema_version": "1.0",
            "frame_id": self.frame_id,
            "path_fingerprint": self.path_fingerprint,
            "reconstructed_path_fingerprint": path_fingerprint(
                self.reconstructed_poses, self.frame_id
            ),
            "raw_pose_count": len(self.raw_labels) + 1,
            "raw_path_length": _stable_float(self.raw_path_length),
            "reconstructed_pose_count": len(self.reconstructed_poses),
            "reconstructed_path_length": _stable_float(
                self.reconstructed_path_length
            ),
            "length_error": _stable_float(self.length_error),
            "length_tolerance": _stable_float(self.length_tolerance),
            "swath_ids": list(self.swath_ids),
            "components": [
                {
                    "order_index": component.order_index,
                    "component_type": component.component_type,
                    "component_id": component.component_id,
                    "swath_id": component.swath_id,
                    "start_pose_index": component.start_pose_index,
                    "end_pose_index": component.end_pose_index,
                    "length": _stable_float(component.length),
                }
                for component in self.components
            ],
            "raw_segments": [
                {
                    "start_index": label.start_index,
                    "end_index": label.end_index,
                    "component_type": label.component_type,
                    "component_id": label.component_id,
                    "swath_id": label.swath_id,
                }
                for label in _coalesce_labels(self.raw_labels)
            ],
        }

    def to_json(self):
        return json.dumps(
            self.to_dict(), sort_keys=True, separators=(",", ":"), allow_nan=False
        )


def build_path_semantics(
    raw_poses,
    swaths,
    turns,
    frame_id="map",
    contains_turns=True,
    swaths_ordered=True,
    swath_sample_step=0.1,
    absolute_length_tolerance=0.05,
    relative_length_tolerance=0.005,
):
    """Rebuild a flat path and label every raw interval from PathComponents."""
    raw = _validate_poses(raw_poses, "raw path")
    swath_inputs = [_validate_swath(swath) for swath in swaths]
    turn_inputs = [_validate_turn(turn) for turn in turns]
    if frame_id != "map":
        raise PathSemanticsError("invalid_semantics_frame", "frame_id must be map")
    if not contains_turns:
        raise PathSemanticsError(
            "path_components_missing_turns",
            "PathComponents must be produced from the generated full path",
        )
    if not swaths_ordered:
        raise PathSemanticsError(
            "path_components_unordered_swaths", "PathComponents swaths must be ordered"
        )
    if not swath_inputs:
        raise PathSemanticsError("empty_swaths", "PathComponents contains no swaths")
    if not math.isfinite(swath_sample_step) or swath_sample_step <= 0.0:
        raise PathSemanticsError(
            "invalid_swath_sample_step", "swath_sample_step must be positive"
        )
    if not all(
        math.isfinite(value)
        for value in (absolute_length_tolerance, relative_length_tolerance)
    ) or absolute_length_tolerance < 0.0 or relative_length_tolerance < 0.0:
        raise PathSemanticsError(
            "invalid_length_tolerance", "length tolerances must be non-negative"
        )

    swath_ids = _stable_swath_ids(swath_inputs)
    schedule = _select_component_schedule(raw, swath_inputs, turn_inputs)
    components = []
    connection_index = 0
    for order_index, (kind, source_index) in enumerate(schedule):
        if kind == SWATH:
            source = swath_inputs[source_index]
            poses = tuple(_interpolate_swath(source, swath_sample_step))
            component_id = swath_ids[source_index]
            components.append(
                SemanticComponent(
                    SWATH,
                    component_id,
                    poses,
                    swath_id=component_id,
                    order_index=order_index,
                )
            )
        else:
            connection_index += 1
            components.append(
                SemanticComponent(
                    CONNECTION,
                    f"connection_{connection_index:04d}",
                    turn_inputs[source_index].poses,
                    order_index=order_index,
                )
            )

    reconstructed, reconstructed_labels = _flatten_components(components)
    raw_length = _path_length(raw)
    reconstructed_length = _path_length(reconstructed)
    tolerance = max(
        float(absolute_length_tolerance),
        float(relative_length_tolerance) * raw_length,
    )
    length_error = abs(raw_length - reconstructed_length)
    if length_error > tolerance + EPSILON:
        raise PathSemanticsError(
            "path_reconstruction_length_mismatch",
            f"path length error {length_error:.6f} exceeds {tolerance:.6f}",
        )
    raw_labels = _map_raw_intervals(raw, reconstructed, reconstructed_labels)
    if len(raw_labels) != len(raw) - 1:
        raise PathSemanticsError(
            "unclassified_path_interval", "every raw path interval must have a type"
        )
    return PathSemanticsResult(
        components=components,
        reconstructed_poses=reconstructed,
        reconstructed_labels=reconstructed_labels,
        raw_labels=raw_labels,
        path_fingerprint=path_fingerprint(raw, frame_id),
        raw_path_length=raw_length,
        reconstructed_path_length=reconstructed_length,
        length_error=length_error,
        length_tolerance=tolerance,
        frame_id=frame_id,
    )


def path_fingerprint(poses, frame_id="map"):
    path = _validate_poses(poses, "path fingerprint")
    payload = {
        "frame_id": frame_id,
        "poses": [
            [_stable_float(pose.x), _stable_float(pose.y), _stable_float(pose.yaw)]
            for pose in path
        ],
    }
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("ascii")
    return hashlib.sha256(encoded).hexdigest()


def parse_path_semantics(document, raw_poses, frame_id="map"):
    """Validate a published semantic document against its exact raw path."""
    if not isinstance(document, dict) or document.get("schema_version") != "1.0":
        raise PathSemanticsError(
            "invalid_path_semantics_schema", "path semantics schema must be 1.0"
        )
    if document.get("frame_id") != frame_id:
        raise PathSemanticsError(
            "path_semantics_frame_mismatch", "path semantics frame differs from path"
        )
    raw = _validate_poses(raw_poses, "semantic path")
    if document.get("path_fingerprint") != path_fingerprint(raw, frame_id):
        raise PathSemanticsError(
            "path_semantics_fingerprint_mismatch",
            "path semantics does not describe the current raw path",
        )
    swath_ids = document.get("swath_ids")
    segments = document.get("raw_segments")
    if not isinstance(swath_ids, list) or not isinstance(segments, list):
        raise PathSemanticsError(
            "invalid_path_semantics_document", "semantic arrays are missing"
        )
    if len(set(swath_ids)) != len(swath_ids) or not all(
        _valid_identifier(value, "swath_") for value in swath_ids
    ):
        raise PathSemanticsError(
            "invalid_swath_ids", "swath IDs must be unique stable identifiers"
        )

    labels = [None] * (len(raw) - 1)
    for item in segments:
        try:
            start = int(item["start_index"])
            end = int(item["end_index"])
            component_type = str(item["component_type"])
            component_id = str(item["component_id"])
            swath_id = str(item.get("swath_id", ""))
        except (KeyError, TypeError, ValueError) as exc:
            raise PathSemanticsError(
                "invalid_path_semantics_document", "invalid raw segment entry"
            ) from exc
        if component_type not in {SWATH, CONNECTION} or not component_id:
            raise PathSemanticsError(
                "invalid_component_type", "component must be SWATH or CONNECTION"
            )
        if component_type == SWATH and swath_id not in swath_ids:
            raise PathSemanticsError(
                "invalid_swath_reference", "SWATH interval must reference a swath ID"
            )
        if component_type == CONNECTION and swath_id:
            raise PathSemanticsError(
                "invalid_connection_reference", "CONNECTION cannot reference a swath ID"
            )
        if start < 0 or end < start or end >= len(labels):
            raise PathSemanticsError(
                "invalid_segment_range", "semantic segment range is outside the path"
            )
        label = SegmentLabel(start, end, component_type, component_id, swath_id)
        for index in range(start, end + 1):
            if labels[index] is not None:
                raise PathSemanticsError(
                    "overlapping_segment_ranges", "semantic segment ranges overlap"
                )
            labels[index] = label
    if any(label is None for label in labels):
        raise PathSemanticsError(
            "unclassified_path_interval", "every raw path interval must have a type"
        )
    return SemanticSummary(
        str(document["path_fingerprint"]), tuple(swath_ids), tuple(labels)
    )


def _select_component_schedule(raw, swaths, turns):
    swath_count = len(swaths)
    turn_count = len(turns)
    if abs(swath_count - turn_count) > 1:
        raise PathSemanticsError(
            "invalid_path_component_count",
            "swath and turn counts may differ by at most one",
        )
    candidates = []
    if swath_count == turn_count + 1:
        candidates.append(_alternating_schedule(SWATH, swath_count, turn_count))
    elif turn_count == swath_count + 1:
        candidates.append(_alternating_schedule(CONNECTION, swath_count, turn_count))
    else:
        candidates.extend(
            (
                _alternating_schedule(SWATH, swath_count, turn_count),
                _alternating_schedule(CONNECTION, swath_count, turn_count),
            )
        )
    scored = [
        (_schedule_cost(candidate, raw, swaths, turns), index, candidate)
        for index, candidate in enumerate(candidates)
    ]
    return min(scored)[2]


def _alternating_schedule(first, swath_count, turn_count):
    output = []
    swath_index = 0
    turn_index = 0
    kind = first
    while swath_index < swath_count or turn_index < turn_count:
        if kind == SWATH and swath_index < swath_count:
            output.append((SWATH, swath_index))
            swath_index += 1
        elif kind == CONNECTION and turn_index < turn_count:
            output.append((CONNECTION, turn_index))
            turn_index += 1
        else:
            return []
        kind = CONNECTION if kind == SWATH else SWATH
    return output


def _schedule_cost(schedule, raw, swaths, turns):
    if not schedule:
        return math.inf
    endpoints = []
    for kind, index in schedule:
        if kind == SWATH:
            endpoints.append((swaths[index].start, swaths[index].end))
        else:
            endpoints.append((turns[index].poses[0], turns[index].poses[-1]))
    cost = _distance(raw[0], endpoints[0][0]) + _distance(raw[-1], endpoints[-1][1])
    cost += sum(
        _distance(previous[1], current[0])
        for previous, current in zip(endpoints, endpoints[1:])
    )
    return _stable_float(cost)


def _flatten_components(components):
    poses = []
    labels = []
    previous_component = None
    for component in components:
        if not poses:
            component.start_pose_index = 0
            poses.extend(component.poses)
            labels.extend(_internal_labels(component, 0))
        else:
            same_point = _distance(poses[-1], component.poses[0]) <= EPSILON
            if same_point:
                start = len(poses) - 1
                component.start_pose_index = start
                poses.extend(component.poses[1:])
                labels.extend(_internal_labels(component, start))
            else:
                bridge = _connection_for_bridge(previous_component, component)
                labels.append(
                    SegmentLabel(
                        len(poses) - 1,
                        len(poses),
                        bridge.component_type,
                        bridge.component_id,
                        bridge.swath_id,
                    )
                )
                start = len(poses)
                component.start_pose_index = start
                poses.extend(component.poses)
                labels.extend(_internal_labels(component, start))
        component.end_pose_index = len(poses) - 1
        previous_component = component
    if len(poses) < 2 or len(labels) != len(poses) - 1:
        raise PathSemanticsError(
            "invalid_reconstructed_path", "reconstructed path requires typed intervals"
        )
    return poses, labels


def _internal_labels(component, start_index):
    return [
        SegmentLabel(
            start_index + index,
            start_index + index + 1,
            component.component_type,
            component.component_id,
            component.swath_id,
        )
        for index in range(len(component.poses) - 1)
    ]


def _connection_for_bridge(previous, current):
    if current.component_type == CONNECTION:
        return current
    if previous is not None and previous.component_type == CONNECTION:
        return previous
    raise PathSemanticsError(
        "invalid_component_schedule", "adjacent swaths require a connection"
    )


def _map_raw_intervals(raw, reconstructed, labels):
    raw_lengths = [_distance(first, second) for first, second in zip(raw, raw[1:])]
    reconstructed_lengths = [
        _distance(first, second)
        for first, second in zip(reconstructed, reconstructed[1:])
    ]
    raw_total = sum(raw_lengths)
    reconstructed_total = sum(reconstructed_lengths)
    if raw_total <= EPSILON or reconstructed_total <= EPSILON:
        raise PathSemanticsError(
            "zero_length_path", "path must contain positive translation length"
        )
    cumulative = []
    value = 0.0
    for length in reconstructed_lengths:
        value += length
        cumulative.append(value)
    output = []
    raw_value = 0.0
    for index, length in enumerate(raw_lengths):
        midpoint = raw_value + length * 0.5
        target = midpoint * reconstructed_total / raw_total
        target_index = min(bisect_left(cumulative, target), len(labels) - 1)
        source = labels[target_index]
        output.append(
            SegmentLabel(
                index,
                index,
                source.component_type,
                source.component_id,
                source.swath_id,
            )
        )
        raw_value += length
    return output


def _stable_swath_ids(swaths):
    indexed = sorted(
        enumerate(swaths),
        key=lambda item: (_canonical_swath_key(item[1]), item[0]),
    )
    output = [""] * len(swaths)
    for stable_index, (source_index, _swath) in enumerate(indexed, start=1):
        output[source_index] = f"swath_{stable_index:04d}"
    return output


def _canonical_swath_key(swath):
    endpoints = sorted(
        (
            (_stable_float(swath.start.x), _stable_float(swath.start.y)),
            (_stable_float(swath.end.x), _stable_float(swath.end.y)),
        )
    )
    return tuple(endpoints)


def _interpolate_swath(swath, sample_step):
    distance = _distance(swath.start, swath.end)
    if distance <= EPSILON:
        raise PathSemanticsError("zero_length_swath", "swath endpoints must differ")
    count = max(1, int(math.ceil(distance / sample_step)))
    yaw = math.atan2(swath.end.y - swath.start.y, swath.end.x - swath.start.x)
    return [
        Pose2D(
            swath.start.x + (swath.end.x - swath.start.x) * index / count,
            swath.start.y + (swath.end.y - swath.start.y) * index / count,
            yaw,
        )
        for index in range(count + 1)
    ]


def _validate_swath(swath):
    if not isinstance(swath, SwathInput):
        raise PathSemanticsError("invalid_swath", "swath input type is invalid")
    _validate_poses((swath.start, swath.end), "swath")
    return swath


def _validate_turn(turn):
    if not isinstance(turn, TurnInput):
        raise PathSemanticsError("invalid_connection", "turn input type is invalid")
    poses = tuple(_validate_poses(turn.poses, "connection", minimum_count=1))
    return TurnInput(poses)


def _validate_poses(poses, name, minimum_count=2):
    output = list(poses)
    if len(output) < minimum_count:
        raise PathSemanticsError(
            "path_too_short", f"{name} requires at least {minimum_count} poses"
        )
    for pose in output:
        if not isinstance(pose, Pose2D) or not all(
            math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)
        ):
            raise PathSemanticsError(
                "invalid_path_pose", f"{name} poses must be finite Pose2D values"
            )
    return output


def _coalesce_labels(labels):
    output = []
    for label in labels:
        if (
            output
            and output[-1].component_type == label.component_type
            and output[-1].component_id == label.component_id
            and output[-1].end_index + 1 == label.start_index
        ):
            previous = output[-1]
            output[-1] = SegmentLabel(
                previous.start_index,
                label.end_index,
                previous.component_type,
                previous.component_id,
                previous.swath_id,
            )
        else:
            output.append(label)
    return output


def _path_length(poses):
    return sum(_distance(first, second) for first, second in zip(poses, poses[1:]))


def _distance(first, second):
    return math.hypot(second.x - first.x, second.y - first.y)


def _valid_identifier(value, prefix):
    return isinstance(value, str) and value.startswith(prefix) and value[len(prefix):].isdigit()


def _stable_float(value):
    return float(f"{float(value):.12g}")
