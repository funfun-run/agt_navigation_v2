import ast
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_NAME = re.compile(r"^agt_[a-z0-9_]+$")
FRAME_NAME = re.compile(r"^[a-z][a-z0-9_]*$")
TOPIC_NAME = re.compile(r"^/agt/[a-z0-9_/]+$")


def test_all_packages_follow_the_naming_contract():
    manifests = sorted((ROOT / "src").glob("agt_*/package.xml"))
    assert len(manifests) == 16
    for manifest in manifests:
        name = ET.parse(manifest).getroot().findtext("name")
        assert PACKAGE_NAME.fullmatch(name), name
        assert manifest.parent.name == name


def test_python_launch_files_parse():
    for launch_file in (ROOT / "src").glob("agt_*/launch/*.launch.py"):
        ast.parse(launch_file.read_text(encoding="utf-8"), filename=str(launch_file))


def test_mid360_profile_uses_only_the_canonical_calibration_file():
    profile = yaml.safe_load(
        (ROOT / "profiles/sensors/mid360.yaml").read_text(encoding="utf-8")
    )
    lidar = profile["sensors"]["lidar"]
    assert lidar["extrinsics"] == {
        "parent_frame": "base_link",
        "calibration_file": "agt_description/config/mk_mini_mid360.yaml",
    }
    assert FRAME_NAME.fullmatch(lidar["frame_id"])
    assert FRAME_NAME.fullmatch(lidar["driver_frame_id"])
    assert TOPIC_NAME.fullmatch(lidar["points_topic"])
    assert TOPIC_NAME.fullmatch(lidar["native_topic"])
    assert lidar["native_message_type"] == "livox_ros_driver2/msg/CustomMsg"


def test_calibration_has_all_required_fields_and_radian_values():
    calibration = yaml.safe_load(
        (ROOT / "src/agt_description/config/mk_mini_mid360.yaml").read_text(
            encoding="utf-8"
        )
    )["/**"]["ros__parameters"]
    required = {
        "calibration_verified",
        "base_length",
        "base_width",
        "base_height",
        "base_link_z",
        "lidar_x",
        "lidar_y",
        "lidar_z",
        "lidar_roll",
        "lidar_pitch",
        "lidar_yaw",
    }
    assert set(calibration) == required
    for angle in ("lidar_roll", "lidar_pitch", "lidar_yaw"):
        assert -3.141592653589793 <= calibration[angle] <= 3.141592653589793


def test_fast_livo_patch_disables_the_native_tf_by_parameter():
    patch = (
        ROOT / "src/agt_mapping/patches/fast_livo2_publish_tf.patch"
    ).read_text(encoding="utf-8")
    assert '"common.publish_tf"' in patch
    assert "if (publish_tf)" in patch
    cmake_patch = (
        ROOT / "src/agt_mapping/patches/fast_livo2_cmake_portability.patch"
    ).read_text(encoding="utf-8")
    assert "vikit_common::vikit_common" in cmake_patch
    assert "vikit_ros::vikit_ros" in cmake_patch
    added_lines = [
        line for line in cmake_patch.splitlines()
        if line.startswith("+") and not line.startswith("+++")
    ]
    assert all("../../install" not in line for line in added_lines)
    lidar_cloud_patch = (
        ROOT / "src/agt_mapping/patches/fast_livo2_lidar_frame_cloud.patch"
    ).read_text(encoding="utf-8")
    assert '"/cloud_registered_lidar"' in lidar_cloud_patch
    assert 'frame_id = "lidar_link"' in lidar_cloud_patch


def test_fast_livo_is_vendored_at_the_selected_pinned_commit():
    vendor = ROOT / "third_party/fast_livo2_ros2"
    assert (vendor / "package.xml").exists()
    assert (vendor / "LICENSE").exists()
    provenance = (ROOT / "third_party/README.md").read_text(encoding="utf-8")
    assert "https://github.com/Aldoubt/FASTLIVO2_ROS2.git" in provenance
    assert "a713004f0ba0624c8fb80d85c7047fe62523c6fb" in provenance
    manifest = ET.parse(vendor / "package.xml").getroot()
    assert manifest.findtext("license") == "GPL-2.0-only"


def test_vendored_fast_livo_publishes_lidar_frame_cloud():
    source = (
        ROOT / "third_party/fast_livo2_ros2/src/LIVMapper.cpp"
    ).read_text(encoding="utf-8")
    assert '"/cloud_registered_lidar"' in source
    assert 'message.header.frame_id = "lidar_link"' in source
    assert "message.header.stamp = odomAftMapped.header.stamp" in source


def test_fast_livo_adapter_is_installed_as_an_executable():
    adapter = ROOT / "src/agt_mapping/scripts/fast_livo2_adapter.py"
    assert adapter.stat().st_mode & 0o111


def test_lidar_only_fast_livo_still_has_required_camera_model():
    camera = yaml.safe_load(
        (
            ROOT / "src/agt_mapping/config/camera_disabled_placeholder.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]["camera"]
    assert camera["model"] == "Pinhole"
    assert camera["width"] > 0
    assert camera["height"] > 0


def test_mid360_fast_livo_uses_native_message_path():
    config = yaml.safe_load(
        (ROOT / "src/agt_mapping/config/mid360_lio_only.yaml").read_text()
    )["/**"]["ros__parameters"]
    assert config["common"]["lid_topic"] == "/agt/sensors/lidar/custom"
    assert config["preprocess"]["lidar_type"] == 1


def test_mid360_network_config_keeps_extrinsics_in_description_only():
    config = json.loads(
        (ROOT / "src/agt_sensor_adapters/config/mid360_network.json").read_text(
            encoding="utf-8"
        )
    )
    device = config["lidar_configs"][0]
    assert device["ip"]
    assert set(device["extrinsic_parameter"].values()) == {0, 0.0}


def test_vendored_livox_driver_provenance_and_license_exist():
    driver = ROOT / "third_party/livox_ros_driver2"
    assert (driver / "package.xml").exists()
    assert (driver / "LICENSE.txt").exists()
    provenance = (ROOT / "third_party/README.md").read_text(encoding="utf-8")
    assert "115c7beeaea02593957af46ccbecc263bc5cf12f" in provenance


def test_relocalization_uses_base_to_lidar_extrinsics_before_publishing_map_odom():
    source = (ROOT / "src/agt_localization/src/relocalization_node.cpp").read_text(
        encoding="utf-8"
    )
    assert 'declare_parameter<std::string>("base_frame", "base_link")' in source
    assert "poseMsgToEigen(msg->pose.pose) * base_from_tracking" in source
    assert "result.estimated_pose * tracking_in_odom" in source
    assert '"/agt/localization/status"' in source
    assert (ROOT / "third_party/relocalization_core/LICENSE").exists()
    assert (ROOT / "third_party/ndt_omp_ros2/LICENSE").exists()


def test_qt5_map_editor_uses_v2_map_interfaces():
    gui = ROOT / "third_party/ros_qt5_gui_app"
    assert (gui / "CMakeLists.txt").exists()
    assert (gui / "LICENSE").read_text(encoding="utf-8").startswith(
        "                    GNU GENERAL PUBLIC LICENSE"
    )
    provenance = (ROOT / "third_party/README.md").read_text(encoding="utf-8")
    assert "b0825e3cba3e7186cba8a6b83ff230be37c8b1fb" in provenance
    config = json.loads(
        (ROOT / "src/agt_ui_bridge/config/ros_qt5_gui_app.json").read_text(
            encoding="utf-8"
        )
    )
    topics = {item["display_name"]: item["topic"] for item in config["display_config"]}
    assert topics["kOccupancyMap"] == "/agt/map/global_occupancy"
    assert topics["kRobotPose"] == "/agt/mapping/odometry"
    assert topics["kSetRobotSpeed"] == "/agt/cmd_vel_manual"
    assert config["key_value"]["BaseFrameId"] == "base_link"
    bridge = (ROOT / "src/agt_ui_bridge/scripts/map_io_bridge.py").read_text(
        encoding="utf-8"
    )
    editor = (ROOT / "src/agt_ui_bridge/scripts/map_editor_qt5.py").read_text(
        encoding="utf-8"
    )
    assert '"/agt/map/load"' in bridge
    assert '"/agt/map/save"' in bridge
    assert '"/agt/map/edited"' in editor
    assert '"/initialpose"' in editor
    assert '"/goal_pose"' in editor
    assert "jie_map_msgs" not in bridge + editor
