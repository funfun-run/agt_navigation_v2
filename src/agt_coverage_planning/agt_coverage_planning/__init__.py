"""Semantic-map adapters for agricultural coverage planning."""

from .coverage_adapter import (
    CoverageAdapterError,
    CoverageRequestSpec,
    prepare_coverage_request,
)
from .path_validator import (
    GridMap,
    PathValidationError,
    Pose2D,
    ValidationReport,
    ValidatorConfig,
    footprint_shape_matches,
    validate_path,
)
from .path_semantics import (
    CONNECTION,
    SWATH,
    PathSemanticsError,
    SwathInput,
    TurnInput,
    build_path_semantics,
    parse_path_semantics,
    path_fingerprint,
)
from .path_repair import (
    PathRepairError,
    RepairPolicy,
    apply_connection_repairs,
    prepare_connection_repairs,
    repair_policy_from_profile,
)
from .coverage_task import (
    ALLOWED_STAGES,
    CoverageTaskError,
    ProgressModel,
    TaskGoal,
    build_progress_model,
    validate_task_goal,
)

__all__ = [
    "CoverageAdapterError",
    "CoverageRequestSpec",
    "prepare_coverage_request",
    "GridMap",
    "PathValidationError",
    "Pose2D",
    "ValidationReport",
    "ValidatorConfig",
    "footprint_shape_matches",
    "validate_path",
    "CONNECTION",
    "SWATH",
    "PathSemanticsError",
    "SwathInput",
    "TurnInput",
    "build_path_semantics",
    "parse_path_semantics",
    "path_fingerprint",
    "PathRepairError",
    "RepairPolicy",
    "apply_connection_repairs",
    "prepare_connection_repairs",
    "repair_policy_from_profile",
    "ALLOWED_STAGES",
    "CoverageTaskError",
    "ProgressModel",
    "TaskGoal",
    "build_progress_model",
    "validate_task_goal",
]
