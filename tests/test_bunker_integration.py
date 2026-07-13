import importlib.util
import math
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_module(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_bunker_dimensions_and_conservative_limits():
    profile = yaml.safe_load((ROOT / "profiles/platforms/bunker.yaml").read_text())[
        "platform"
    ]
    geometry = profile["geometry"]
    assert geometry["length"] == 1.023
    assert geometry["width"] == 0.778
    assert geometry["height"] == 0.400
    assert geometry["outer_dimensions_verified"] is True
    assert geometry["effective_track_width_verified"] is False
    assert profile["limits"]["max_forward_velocity"] <= 0.5


def test_track_projection_preserves_curvature_and_caps_both_tracks():
    safety = load_module(
        ROOT / "src/agt_safety/scripts/tracked_safety_controller.py"
    )
    linear, angular = safety.project_track_speeds(0.5, 0.6, 0.62, 0.65)
    left = linear - angular * 0.62 * 0.5
    right = linear + angular * 0.62 * 0.5
    assert max(abs(left), abs(right)) <= 0.65 + 1e-9
    assert math.isclose(angular / linear, 0.6 / 0.5)


def test_slew_uses_acceleration_and_deceleration_limits():
    safety = load_module(
        ROOT / "src/agt_safety/scripts/tracked_safety_controller.py"
    )
    assert math.isclose(safety.slew(0.0, 1.0, 0.35, 0.7, 0.1), 0.035)
    assert math.isclose(safety.slew(0.5, 0.0, 0.35, 0.7, 0.1), 0.43)


def test_driver_has_tf_switch_and_independent_watchdog():
    messenger = (
        ROOT
        / "third_party/bunker_ros2/bunker_base/include/bunker_base/bunker_messenger.hpp"
    ).read_text()
    launch = (ROOT / "src/agt_chassis/launch/bunker.launch.py").read_text()
    assert "command_timeout_" in messenger
    assert "publish_odom_tf_" in messenger
    assert '"publish_driver_odom_tf", default_value="false"' in launch
    assert '("/cmd_vel", "/agt/chassis/cmd_vel")' in launch


def test_bag_recorder_keeps_remote_controller_state():
    recorder = (ROOT / "src/agt_bringup/launch/bag_record.launch.py").read_text()
    assert '"/agt/chassis/rc_state"' in recorder


def test_estop_also_revokes_motion_enable():
    source = (
        ROOT / "src/agt_safety/scripts/tracked_safety_controller.py"
    ).read_text()
    callback = source.split("def _estop_callback", 1)[1].split(
        "def _set_motion_enabled", 1
    )[0]
    assert "self._estop_latched = True" in callback
    assert "self._motion_enabled = False" in callback


def test_vendor_actuator_copy_does_not_overflow_two_element_state():
    source = (
        ROOT
        / "third_party/ugv_sdk/include/ugv_sdk/details/robot_base/bunker_base.hpp"
    ).read_text()
    assert "for (int i = 0; i < 2; ++i)" in source
