"""Transactional replacement of invalid coverage CONNECTION components."""

from dataclasses import dataclass
import json
import math
import time

from .path_semantics import CONNECTION, SWATH, path_fingerprint
from .path_validator import Pose2D


EPSILON = 1e-9


class PathRepairError(ValueError):
    """Stable repair failure that leaves the source path unchanged."""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = str(code)


@dataclass(frozen=True)
class RepairPolicy:
    platform_name: str
    kinematics: str
    planner_id: str
    allow_in_place_rotation: bool
    min_turning_radius: float


@dataclass(frozen=True)
class RepairTarget:
    component_id: str
    start_pose_index: int
    end_pose_index: int
    order_index: int


@dataclass(frozen=True)
class RepairPreparation:
    targets: tuple
    preserved_swath_ids: tuple


@dataclass
class RepairResult:
    poses: list
    repaired_component_ids: list
    preserved_swath_ids: list
    original_length: float
    repaired_length: float
    swath_coordinates_unchanged: bool


@dataclass
class RepairReport:
    success: bool
    state: str
    error_code: str = "none"
    detail: str = ""
    planner_id: str = ""
    repaired_segment_count: int = 0
    repaired_component_ids: tuple = ()
    preserved_swath_ids: tuple = ()
    duration: float = 0.0
    original_length: float = 0.0
    repaired_length: float = 0.0
    swath_coordinates_unchanged: bool = False
    final_validation: dict = None

    def to_dict(self):
        return {
            "success": self.success,
            "state": self.state,
            "error_code": self.error_code,
            "detail": self.detail,
            "planner_id": self.planner_id,
            "repaired_segment_count": self.repaired_segment_count,
            "repaired_component_ids": list(self.repaired_component_ids),
            "preserved_swath_ids": list(self.preserved_swath_ids),
            "duration": _stable_float(self.duration),
            "original_length": _stable_float(self.original_length),
            "repaired_length": _stable_float(self.repaired_length),
            "swath_coordinates_unchanged": self.swath_coordinates_unchanged,
            "final_validation": self.final_validation or {},
        }

    def to_json(self):
        return json.dumps(
            self.to_dict(), sort_keys=True, separators=(",", ":"), allow_nan=False
        )


def repair_policy_from_profile(profile_document):
    """Load the explicit repair planner contract from a platform profile mapping."""
    try:
        platform = profile_document["platform"]
        repair = platform["coverage_repair"]
        platform_name = str(platform["name"])
        kinematics = str(platform["kinematics"])
    except (KeyError, TypeError) as exc:
        raise PathRepairError(
            "repair_policy_missing", "platform profile has no coverage_repair contract"
        ) from exc
    if not bool(repair.get("enabled", False)):
        reason = str(repair.get("disabled_reason", "coverage repair is disabled"))
        raise PathRepairError("coverage_repair_disabled", reason)
    planner_id = str(repair.get("planner_id", ""))
    if not planner_id:
        raise PathRepairError("repair_planner_missing", "repair planner_id is empty")
    allow_rotation = bool(repair.get("allow_in_place_rotation", False))
    radius = float(platform.get("geometry", {}).get("min_turning_radius", 0.0))
    if not math.isfinite(radius) or radius < 0.0:
        raise PathRepairError(
            "invalid_repair_turning_radius", "minimum turning radius is invalid"
        )
    if kinematics in {"tracked_differential", "differential"}:
        if not allow_rotation:
            raise PathRepairError(
                "differential_rotation_contract_invalid",
                "differential repair must explicitly allow in-place rotation",
            )
    elif kinematics == "ackermann":
        planner_family = str(repair.get("planner_family", ""))
        if planner_family not in {"hybrid_a_star", "state_lattice"}:
            raise PathRepairError(
                "ackermann_planner_contract_invalid",
                "Ackermann repair requires Hybrid-A* or State Lattice",
            )
        if allow_rotation or radius <= EPSILON:
            raise PathRepairError(
                "ackermann_turning_contract_invalid",
                "Ackermann repair requires positive radius and forbids in-place rotation",
            )
    else:
        raise PathRepairError(
            "unsupported_repair_kinematics", f"unsupported kinematics: {kinematics}"
        )
    return RepairPolicy(
        platform_name=platform_name,
        kinematics=kinematics,
        planner_id=planner_id,
        allow_in_place_rotation=allow_rotation,
        min_turning_radius=radius,
    )


def prepare_connection_repairs(path, semantic_document, validation_report):
    """Select invalid connection components without modifying any source product."""
    poses = _validate_path(path)
    if not isinstance(semantic_document, dict) or not isinstance(
        validation_report, dict
    ):
        raise PathRepairError("invalid_repair_input", "repair inputs must be mappings")
    if semantic_document.get("schema_version") != "1.0":
        raise PathRepairError(
            "invalid_path_semantics_schema", "path semantics schema must be 1.0"
        )
    expected = semantic_document.get("reconstructed_path_fingerprint")
    if expected != path_fingerprint(poses, "map"):
        raise PathRepairError(
            "reconstructed_path_fingerprint_mismatch",
            "path semantics does not describe the reconstructed path",
        )
    if validation_report.get("path_fingerprint") != semantic_document.get(
        "path_fingerprint"
    ):
        raise PathRepairError(
            "validation_report_fingerprint_mismatch",
            "validation report does not describe the semantic source path",
        )
    if validation_report.get("valid") is True:
        return RepairPreparation((), tuple(semantic_document.get("swath_ids", [])))
    invalid_swaths = validation_report.get("invalid_swath_ids", [])
    if invalid_swaths:
        raise PathRepairError(
            "swath_repair_forbidden",
            "invalid SWATH components cannot be modified by TASK-12",
        )
    invalid_ids = validation_report.get("invalid_component_ids")
    if not isinstance(invalid_ids, list) or not invalid_ids:
        raise PathRepairError(
            "no_invalid_connections", "validation report contains no repair target"
        )
    if len(set(invalid_ids)) != len(invalid_ids):
        raise PathRepairError(
            "duplicate_invalid_component", "invalid component IDs must be unique"
        )

    components = semantic_document.get("components")
    if not isinstance(components, list):
        raise PathRepairError(
            "invalid_path_semantics_document", "semantic components are missing"
        )
    by_id = {}
    swath_ids = []
    for component in components:
        try:
            component_id = str(component["component_id"])
            component_type = str(component["component_type"])
            start = int(component["start_pose_index"])
            end = int(component["end_pose_index"])
            order = int(component["order_index"])
        except (KeyError, TypeError, ValueError) as exc:
            raise PathRepairError(
                "invalid_path_semantics_document", "invalid semantic component"
            ) from exc
        if component_id in by_id:
            raise PathRepairError(
                "duplicate_component_id", "semantic component IDs must be unique"
            )
        if start < 0 or end <= start or end >= len(poses):
            raise PathRepairError(
                "invalid_component_pose_range", "component pose range is invalid"
            )
        by_id[component_id] = (component_type, start, end, order)
        if component_type == SWATH:
            swath_ids.append(component_id)

    targets = []
    for component_id in invalid_ids:
        if component_id not in by_id:
            raise PathRepairError(
                "unknown_invalid_component", f"unknown component: {component_id}"
            )
        component_type, start, end, order = by_id[component_id]
        if component_type != CONNECTION:
            raise PathRepairError(
                "non_connection_repair_forbidden",
                f"component {component_id} is not a CONNECTION",
            )
        targets.append(RepairTarget(component_id, start, end, order))
    targets.sort(key=lambda target: target.start_pose_index)
    for previous, current in zip(targets, targets[1:]):
        if previous.end_pose_index >= current.start_pose_index:
            raise PathRepairError(
                "overlapping_repair_targets", "repair target ranges overlap"
            )
    return RepairPreparation(tuple(targets), tuple(swath_ids))


def apply_connection_repairs(
    path,
    semantic_document,
    preparation,
    replacements,
    endpoint_tolerance=0.25,
):
    """Apply all replacements atomically and prove every SWATH remains unchanged."""
    poses = _validate_path(path)
    if not math.isfinite(endpoint_tolerance) or endpoint_tolerance < 0.0:
        raise PathRepairError(
            "invalid_endpoint_tolerance", "endpoint tolerance must be non-negative"
        )
    target_ids = {target.component_id for target in preparation.targets}
    if set(replacements) != target_ids:
        raise PathRepairError(
            "incomplete_repair_set", "every selected connection needs one replacement"
        )
    normalized = {}
    for target in preparation.targets:
        candidate = _validate_path(replacements[target.component_id])
        source_start = poses[target.start_pose_index]
        source_end = poses[target.end_pose_index]
        start_mismatch = _distance(candidate[0], source_start) > endpoint_tolerance
        end_mismatch = _distance(candidate[-1], source_end) > endpoint_tolerance
        if start_mismatch or end_mismatch:
            raise PathRepairError(
                "repair_endpoint_mismatch",
                f"replacement endpoints differ for {target.component_id}",
            )
        candidate[0] = source_start
        candidate[-1] = source_end
        normalized[target.component_id] = candidate

    repaired = []
    cursor = 0
    for target in preparation.targets:
        repaired.extend(poses[cursor:target.start_pose_index])
        repaired.extend(normalized[target.component_id])
        cursor = target.end_pose_index + 1
    repaired.extend(poses[cursor:])
    if len(repaired) < 2:
        raise PathRepairError("empty_repaired_path", "repaired path is empty")

    unchanged = _swaths_are_unchanged(poses, repaired, semantic_document)
    if not unchanged:
        raise PathRepairError(
            "swath_coordinates_changed", "repair modified or deleted a SWATH coordinate"
        )
    return RepairResult(
        poses=repaired,
        repaired_component_ids=[target.component_id for target in preparation.targets],
        preserved_swath_ids=list(preparation.preserved_swath_ids),
        original_length=_path_length(poses),
        repaired_length=_path_length(repaired),
        swath_coordinates_unchanged=True,
    )


def successful_report(result, planner_id, started_at, final_validation):
    return RepairReport(
        success=True,
        state="SUCCEEDED",
        planner_id=planner_id,
        repaired_segment_count=len(result.repaired_component_ids),
        repaired_component_ids=tuple(result.repaired_component_ids),
        preserved_swath_ids=tuple(result.preserved_swath_ids),
        duration=max(0.0, time.monotonic() - started_at),
        original_length=result.original_length,
        repaired_length=result.repaired_length,
        swath_coordinates_unchanged=result.swath_coordinates_unchanged,
        final_validation=final_validation,
    )


def failed_report(code, detail, planner_id="", started_at=None):
    duration = 0.0 if started_at is None else max(0.0, time.monotonic() - started_at)
    return RepairReport(
        success=False,
        state="FAILED",
        error_code=str(code),
        detail=str(detail),
        planner_id=str(planner_id),
        duration=duration,
    )


def _swaths_are_unchanged(original, repaired, semantic_document):
    components = semantic_document.get("components", [])
    for component in components:
        if component.get("component_type") != SWATH:
            continue
        start = int(component["start_pose_index"])
        end = int(component["end_pose_index"])
        source = original[start:end + 1]
        if not _contains_exact_subsequence(repaired, source):
            return False
    return True


def _contains_exact_subsequence(path, expected):
    if not expected or len(expected) > len(path):
        return False
    return any(
        path[index:index + len(expected)] == expected
        for index in range(len(path) - len(expected) + 1)
    )


def _validate_path(path):
    output = list(path)
    if len(output) < 2:
        raise PathRepairError("path_too_short", "repair path needs at least two poses")
    for pose in output:
        if not isinstance(pose, Pose2D) or not all(
            math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)
        ):
            raise PathRepairError("invalid_repair_pose", "repair poses must be finite")
    return output


def _path_length(path):
    return sum(_distance(first, second) for first, second in zip(path, path[1:]))


def _distance(first, second):
    return math.hypot(second.x - first.x, second.y - first.y)


def _stable_float(value):
    return float(f"{float(value):.12g}")
