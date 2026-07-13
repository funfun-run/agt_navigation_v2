import importlib.util
import math
from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "fast_livo2_adapter.py"
SPEC = importlib.util.spec_from_file_location("fast_livo2_adapter", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_translation_extrinsic_moves_base_behind_sensor():
    position, orientation = MODULE.sensor_pose_to_base_pose(
        (10.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
        ((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    )
    assert position == pytest.approx((9.0, 0.0, 0.0))
    assert orientation == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_sensor_yaw_is_removed_from_base_pose():
    half_yaw = math.pi / 4.0
    yaw_quaternion = (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
    _, orientation = MODULE.sensor_pose_to_base_pose(
        (0.0, 0.0, 0.0), yaw_quaternion, ((0.0, 0.0, 0.0), yaw_quaternion)
    )
    assert orientation == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_zero_quaternion_is_rejected():
    with pytest.raises(ValueError):
        MODULE.sensor_pose_to_base_pose(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0),
            ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        )


def test_twist_accounts_for_sensor_lever_arm():
    linear, angular = MODULE.sensor_twist_to_base_twist(
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        ((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    )
    assert linear == pytest.approx((0.0, 0.0, 0.0))
    assert angular == pytest.approx((0.0, 0.0, 1.0))
