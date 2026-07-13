from pathlib import Path
import xml.etree.ElementTree as ET


XACRO_NS = "http://www.ros.org/wiki/xacro"
EXPECTED_PARENTS = {
    "base_link": "base_footprint",
    "lidar_link": "base_link",
    "livox_frame": "lidar_link",
    "imu_link": "lidar_link",
}


def _model_root():
    model = Path(__file__).parents[1] / "urdf" / "agt_base.urdf.xacro"
    return ET.parse(model).getroot()


def test_required_frames_have_one_parent():
    root = _model_root()
    parents_by_child = {}
    for joint in root.findall("joint"):
        child = joint.find("child")
        parent = joint.find("parent")
        if child is not None and parent is not None:
            parents_by_child.setdefault(child.attrib["link"], []).append(
                parent.attrib["link"]
            )

    assert parents_by_child == {
        child: [parent] for child, parent in EXPECTED_PARENTS.items()
    }


def test_base_footprint_is_a_root_frame():
    root = _model_root()
    child_frames = {
        child.attrib["link"]
        for child in root.findall("joint/child")
    }
    assert "base_footprint" not in child_frames


def test_extrinsics_are_launch_overridable():
    root = _model_root()
    argument_names = {
        argument.attrib["name"]
        for argument in root.findall(f"{{{XACRO_NS}}}arg")
    }
    assert {
        "lidar_x",
        "lidar_y",
        "lidar_z",
        "lidar_roll",
        "lidar_pitch",
        "lidar_yaw",
    }.issubset(argument_names)
