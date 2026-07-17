import json
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_profile(name):
    path = PACKAGE_ROOT / "config" / f"ros_qt5_gui_{name}.json"
    return json.loads(path.read_text(encoding="utf-8"))


def topics(config):
    return {item["display_name"]: item["topic"] for item in config["display_config"]}


def test_mapping_profile_contract():
    config = load_profile("mapping")
    assert topics(config)["kOccupancyMap"] == "/agt/map/mapping_occupancy"
    assert topics(config)["kRobotPose"] == "/agt/mapping/odometry"
    assert topics(config)["kSetRobotSpeed"] == "/agt/cmd_vel_manual"
    assert topics(config)["kGlobalPath"] == "/plan"
    assert topics(config)["kLocalPath"] == "/local_plan"
    assert config["key_value"] == {
        "BaseFrameId": "base_footprint",
        "FixedFrameId": "odom",
    }


def test_navigation_profile_contract():
    config = load_profile("navigation")
    profile_topics = topics(config)
    assert profile_topics["kOccupancyMap"] == "/agt/map/global_occupancy"
    assert profile_topics["GoalPose"] == "/goal_pose"
    assert profile_topics["kSetRelocPose"] == "/initialpose"
    assert profile_topics["kSetRobotSpeed"] == "/agt/cmd_vel_manual"
    assert config["key_value"] == {
        "BaseFrameId": "base_footprint",
        "FixedFrameId": "map",
    }


def test_profiles_have_isolated_runtime_configs():
    script = (PACKAGE_ROOT / "scripts/start_ros_qt5_gui_app.sh").read_text(
        encoding="utf-8"
    )
    assert 'RUNTIME_DIR="${RUNTIME_ROOT}/${PROFILE}"' in script
    assert "ros_qt5_gui_${PROFILE}.json" in script
    forbidden_home = "/".join(["", "home", "yangxuan"])
    assert forbidden_home not in script
    assert "LEGACY_BUILD_DIR" not in script


def test_gui_never_targets_chassis_command_topic():
    for profile in (load_profile("mapping"), load_profile("navigation")):
        assert "/agt/chassis/cmd_vel" not in topics(profile).values()
