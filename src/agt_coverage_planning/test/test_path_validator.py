import json
import math
from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPOSITORY_ROOT / "src/agt_ui_bridge"))

from agt_coverage_planning.path_validator import (  # noqa: E402
    GridMap,
    PathValidationError,
    Pose2D,
    ValidatorConfig,
    footprint_shape_matches,
    validate_path,
)


SMALL_FOOTPRINT = ((-0.4, -0.2), (0.4, -0.2), (0.4, 0.2), (-0.4, 0.2))


def _grid(width=120, height=60, resolution=0.1, fill=0, cells=None, yaw=0.0):
    data = [fill] * (width * height)
    for column, row, cost in cells or []:
        data[row * width + column] = cost
    return GridMap(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=yaw,
        data=tuple(data),
    )


def test_free_path_is_valid_and_report_has_required_fields():
    result = validate_path(
        [Pose2D(1.0, 2.0, 0.0), Pose2D(4.0, 2.0, 0.0)],
        "map",
        _grid(),
        SMALL_FOOTPRINT,
        min_turning_radius=0.0,
    )

    assert result.report.valid
    assert result.report.collision_pose_count == 0
    assert result.report.invalid_segment_indices == []
    assert result.report.maximum_cost == 0
    assert result.report.minimum_clearance > 0.0
    assert result.report.maximum_curvature == 0.0
    assert result.report.required_min_turning_radius == 0.0
    assert result.report.sample_count > 2
    parsed = json.loads(result.report.to_json())
    assert set(
        (
            "valid",
            "collision_pose_count",
            "invalid_segment_indices",
            "maximum_cost",
            "minimum_clearance",
            "maximum_curvature",
            "required_min_turning_radius",
        )
    ) <= set(parsed)


def test_sparse_path_detects_collision_between_original_poses():
    result = validate_path(
        [Pose2D(1.0, 2.0, 0.0), Pose2D(10.0, 2.0, 0.0)],
        "map",
        _grid(cells=[(50, 20, 100)]),
        SMALL_FOOTPRINT,
        min_turning_radius=0.0,
    )

    assert not result.report.valid
    assert result.report.collision_pose_count > 0
    assert result.report.invalid_segment_indices == [0]
    assert result.report.maximum_cost == 100
    assert "footprint_collision" in result.report.error_codes
    assert all(sample.pose.x not in {1.0, 10.0} for sample in result.collision_samples)


def test_full_footprint_edge_collision_is_not_reduced_to_center_or_corners():
    result = validate_path(
        [Pose2D(2.0, 2.0, 0.0), Pose2D(2.1, 2.0, 0.0)],
        "map",
        _grid(cells=[(23, 19, 100)]),
        SMALL_FOOTPRINT,
        min_turning_radius=0.0,
    )

    assert not result.report.valid
    assert result.report.collision_pose_count > 0


def test_in_place_rotation_interpolation_detects_corner_sweep_collision():
    long_footprint = ((-1.0, -0.2), (1.0, -0.2), (1.0, 0.2), (-1.0, 0.2))
    result = validate_path(
        [Pose2D(3.0, 3.0, 0.0), Pose2D(3.0, 3.0, math.pi / 2.0)],
        "map",
        _grid(cells=[(36, 36, 100)]),
        long_footprint,
        min_turning_radius=0.0,
    )

    assert not result.report.valid
    assert result.report.in_place_rotation_count > 0
    assert result.report.collision_pose_count > 0
    assert all(
        0.0 < sample.pose.yaw < math.pi / 2.0
        for sample in result.collision_samples
    )


def test_unknown_space_policy_is_configurable():
    grid = _grid(cells=[(20, 20, -1)])
    poses = [Pose2D(2.0, 2.0, 0.0), Pose2D(2.1, 2.0, 0.0)]

    conservative = validate_path(
        poses,
        "map",
        grid,
        SMALL_FOOTPRINT,
        0.0,
        ValidatorConfig(unknown_space_policy="collision"),
    )
    permissive = validate_path(
        poses,
        "map",
        grid,
        SMALL_FOOTPRINT,
        0.0,
        ValidatorConfig(unknown_space_policy="free"),
    )

    assert not conservative.report.valid
    assert conservative.report.unknown_collision_pose_count > 0
    assert permissive.report.valid
    assert permissive.report.unknown_collision_pose_count == 0


def test_positive_turning_radius_rejects_tight_curve_and_in_place_rotation():
    curved = validate_path(
        [Pose2D(2.0, 2.0, 0.0), Pose2D(2.1, 2.0, math.pi / 2.0)],
        "map",
        _grid(),
        SMALL_FOOTPRINT,
        min_turning_radius=1.0,
    )
    rotation = validate_path(
        [Pose2D(3.0, 3.0, 0.0), Pose2D(3.0, 3.0, 0.5)],
        "map",
        _grid(),
        SMALL_FOOTPRINT,
        min_turning_radius=1.0,
    )

    assert not curved.report.valid
    assert curved.report.maximum_curvature > 1.0
    assert "minimum_turning_radius_violation" in curved.report.error_codes
    assert not rotation.report.valid
    assert rotation.report.in_place_rotation_count > 0


def test_runtime_footprint_shape_matches_profile_independent_of_pose():
    published = ((4.4, 2.8), (4.4, 3.2), (3.6, 3.2), (3.6, 2.8))
    wrong = ((4.8, 2.8), (4.8, 3.2), (3.2, 3.2), (3.2, 2.8))
    equal_edges_wrong_angles = ((0.0, 0.0), (0.8, 0.0), (1.0, 0.3464), (0.2, 0.3464))

    assert footprint_shape_matches(SMALL_FOOTPRINT, published)
    assert not footprint_shape_matches(SMALL_FOOTPRINT, wrong)
    assert not footprint_shape_matches(SMALL_FOOTPRINT, equal_edges_wrong_angles)


def test_rotated_costmap_origin_is_respected():
    grid = GridMap(
        width=60,
        height=60,
        resolution=0.1,
        origin_x=5.0,
        origin_y=-2.0,
        origin_yaw=math.pi / 2.0,
        data=tuple([0] * 3600),
    )
    result = validate_path(
        [Pose2D(3.0, -1.0, math.pi / 2.0), Pose2D(3.0, 2.0, math.pi / 2.0)],
        "map",
        grid,
        SMALL_FOOTPRINT,
        min_turning_radius=0.0,
    )
    assert result.report.valid


def test_reports_are_byte_stable_for_identical_inputs():
    arguments = (
        [Pose2D(1.0, 2.0, 0.0), Pose2D(4.0, 2.0, 0.0)],
        "map",
        _grid(cells=[(25, 20, 100)]),
        SMALL_FOOTPRINT,
        0.0,
    )
    first = validate_path(*arguments).report.to_json()
    second = validate_path(*arguments).report.to_json()
    assert first == second


def test_invalid_frame_and_data_are_rejected_with_stable_codes():
    with pytest.raises(PathValidationError) as frame_error:
        validate_path(
            [Pose2D(1.0, 1.0, 0.0), Pose2D(2.0, 1.0, 0.0)],
            "odom",
            _grid(),
            SMALL_FOOTPRINT,
            0.0,
        )
    assert frame_error.value.code == "invalid_path_frame"

    with pytest.raises(PathValidationError) as grid_error:
        GridMap(2, 2, 0.1, 0.0, 0.0, 0.0, (0, 0), "map")
    assert grid_error.value.code == "invalid_costmap_data"
