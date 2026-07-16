import math
from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPOSITORY_ROOT / "src/agt_ui_bridge"))

from agt_coverage_planning.time_simulation import (  # noqa: E402
    MotionLimits,
    SimulationPose,
    TimeSimulationError,
    simulate_path_time,
)


LIMITS = MotionLimits(0.5, 0.25, 0.6, 0.35, 0.7, 0.8, 1.2)


def _pose(x, y, yaw=0.0):
    return SimulationPose(float(x), float(y), float(yaw))


def test_straight_path_uses_acceleration_cruise_and_deceleration():
    report = simulate_path_time([_pose(0, 0), _pose(10, 0)], LIMITS)

    assert report.total_path_length == pytest.approx(10.0)
    assert report.forward_path_length == pytest.approx(10.0)
    assert report.reverse_path_length == 0.0
    assert report.estimated_motion_time > 20.0
    assert report.classification_source == "geometric_fallback"


def test_reverse_and_direction_change_are_slower_and_force_a_stop():
    forward = simulate_path_time([_pose(0, 0), _pose(2, 0)], LIMITS)
    reverse = simulate_path_time(
        [_pose(0, 0, math.pi), _pose(2, 0, math.pi)], LIMITS
    )
    fishtail = simulate_path_time(
        [_pose(0, 0), _pose(2, 0), _pose(1, 0)], LIMITS
    )

    assert reverse.reverse_path_length == pytest.approx(2.0)
    assert reverse.estimated_motion_time > forward.estimated_motion_time
    assert fishtail.direction_change_count == 1
    assert fishtail.estimated_motion_time > forward.estimated_motion_time


def test_curvature_and_pure_rotation_respect_angular_limits():
    straight = simulate_path_time([_pose(0, 0), _pose(1, 0)], LIMITS)
    curved = simulate_path_time(
        [_pose(0, 0), _pose(1, 0, math.pi / 2.0)], LIMITS
    )
    rotation = simulate_path_time(
        [_pose(0, 0), _pose(0, 0, math.pi)], LIMITS
    )

    assert curved.estimated_motion_time > straight.estimated_motion_time
    assert rotation.pure_rotation_angle == pytest.approx(math.pi)
    assert rotation.estimated_motion_time > math.pi / LIMITS.max_angular_velocity


def test_semantics_split_work_and_connection_metrics_deterministically():
    report = simulate_path_time(
        [_pose(0, 0), _pose(2, 0), _pose(2, 1, math.pi / 2.0), _pose(2, 3, math.pi / 2.0)],
        LIMITS,
        path_fingerprint="abc",
        segment_types=["SWATH", "CONNECTION", "SWATH"],
        component_ids=["swath_1", "connection_1", "swath_2"],
    )

    assert report.work_path_length == pytest.approx(4.0)
    assert report.non_work_path_length == pytest.approx(1.0)
    assert report.estimated_turn_count == 1
    assert report.estimated_work_time + report.estimated_non_work_time == pytest.approx(
        report.estimated_motion_time
    )
    assert report.to_dict() == report.to_dict()


def test_invalid_classification_and_limits_fail_closed():
    with pytest.raises(TimeSimulationError, match="SWATH or CONNECTION"):
        simulate_path_time(
            [_pose(0, 0), _pose(1, 0)], LIMITS, segment_types=["UNKNOWN"]
        )
    with pytest.raises(TimeSimulationError, match="finite and positive"):
        simulate_path_time(
            [_pose(0, 0), _pose(1, 0)],
            MotionLimits(0.0, 0.25, 0.6, 0.35, 0.7, 0.8, 1.2),
        )
