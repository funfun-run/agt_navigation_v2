from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from agt_coverage_planning.coverage_task import (  # noqa: E402
    ERROR_INVALID_GOAL,
    CoverageTaskError,
    build_progress_model,
    validate_task_goal,
)
from agt_coverage_planning.path_semantics import (  # noqa: E402
    Pose2D,
    SwathInput,
    TurnInput,
    build_path_semantics,
)


class Goal:
    semantic_map_uri = "map.geojson"
    field_id = "field_001"
    planning_mode = "polygon"
    controller_id = "FollowPath"
    allow_repair = True


def _products():
    raw = [
        Pose2D(0, 0, 0),
        Pose2D(2, 0, 0),
        Pose2D(2.5, 0.5, 1.57),
        Pose2D(2, 1, 3.14),
        Pose2D(0, 1, 3.14),
    ]
    semantics = build_path_semantics(
        raw,
        [
            SwathInput(raw[0], raw[1]),
            SwathInput(raw[3], raw[4]),
        ],
        [TurnInput(tuple(raw[1:4]))],
        swath_sample_step=1.0,
    )
    return raw, semantics


def test_goal_contract_rejects_missing_or_unknown_values():
    assert validate_task_goal(Goal()).field_id == "field_001"
    goal = Goal()
    goal.semantic_map_uri = ""
    with pytest.raises(CoverageTaskError) as missing:
        validate_task_goal(goal)
    assert missing.value.code == ERROR_INVALID_GOAL

    goal.semantic_map_uri = "map.geojson"
    goal.planning_mode = "unknown"
    with pytest.raises(CoverageTaskError) as mode:
        validate_task_goal(goal)
    assert mode.value.code == ERROR_INVALID_GOAL


def test_raw_path_progress_uses_swath_semantics_not_equal_partitioning():
    raw, semantics = _products()
    model = build_progress_model(raw, semantics.to_dict())
    assert model.total_swaths == 2
    assert model.swath_index(model.total_length) == 0
    assert model.swath_index(0.0) == 1


def test_repaired_connection_progress_preserves_swath_order():
    _raw, semantics = _products()
    reconstructed = semantics.reconstructed_poses
    connection = next(
        component
        for component in semantics.components
        if component.component_type == "CONNECTION"
    )
    repaired = list(reconstructed[:connection.start_pose_index])
    repaired.extend(
        [
            reconstructed[connection.start_pose_index],
            Pose2D(3.0, 0.5, 1.57),
            reconstructed[connection.end_pose_index],
        ]
    )
    repaired.extend(reconstructed[connection.end_pose_index + 1:])
    model = build_progress_model(repaired, semantics.to_dict(), reconstructed)
    assert model.total_swaths == 2
    assert model.swath_index(model.total_length) == 0
    assert model.swath_index(0.0) == 1


def test_progress_rejects_repaired_path_that_deleted_a_swath():
    _raw, semantics = _products()
    reconstructed = semantics.reconstructed_poses
    with pytest.raises(CoverageTaskError):
        build_progress_model(reconstructed[:-2], semantics.to_dict(), reconstructed)
