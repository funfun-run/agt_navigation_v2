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
