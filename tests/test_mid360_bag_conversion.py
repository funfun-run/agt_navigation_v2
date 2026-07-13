import importlib.util
from pathlib import Path

import pytest
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools/bag_tools/convert_mid360_pointcloud2_to_custom.py"
SPEC = importlib.util.spec_from_file_location("mid360_bag_conversion", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="tag", offset=16, datatype=PointField.UINT8, count=1),
    PointField(name="line", offset=17, datatype=PointField.UINT8, count=1),
    PointField(name="timestamp", offset=18, datatype=PointField.FLOAT64, count=1),
]


def test_conversion_preserves_livox_fields_and_relative_time():
    cloud = point_cloud2.create_cloud(Header(frame_id="livox_frame"), FIELDS, [
        (1.0, 2.0, 3.0, 42.0, 16, 2, 1_000_000_000.0),
        (4.0, 5.0, 6.0, 300.0, 32, 3, 1_000_012_345.0),
    ])
    result = MODULE.pointcloud_to_custom(cloud)
    assert result.timebase == 1_000_000_000
    assert result.point_num == 2
    assert result.points[1].offset_time == 12_345
    assert result.points[0].line == 2
    assert result.points[0].tag == 16
    assert result.points[1].reflectivity == 255


def test_conversion_rejects_cloud_without_timestamps():
    cloud = point_cloud2.create_cloud_xyz32(Header(), [(1.0, 2.0, 3.0)])
    with pytest.raises(ValueError, match="missing fields"):
        MODULE.pointcloud_to_custom(cloud)
