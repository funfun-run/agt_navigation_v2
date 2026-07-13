#!/usr/bin/env python3
"""Convert Livox PointCloud2 recordings to livox_ros_driver2 CustomMsg."""

import argparse
from pathlib import Path

import rosbag2_py
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2


INPUT_LIDAR_TOPIC = "/livox/lidar"
INPUT_IMU_TOPIC = "/livox/imu"
OUTPUT_LIDAR_TOPIC = "/agt/sensors/lidar/custom"
OUTPUT_IMU_TOPIC = "/agt/sensors/imu/data"
REQUIRED_FIELDS = {"x", "y", "z", "intensity", "tag", "line", "timestamp"}


def pointcloud_to_custom(message):
    missing = REQUIRED_FIELDS - {field.name for field in message.fields}
    if missing:
        raise ValueError(f"PointCloud2 is missing fields: {sorted(missing)}")
    rows = list(point_cloud2.read_points(
        message,
        field_names=("x", "y", "z", "intensity", "tag", "line", "timestamp"),
        skip_nans=True,
    ))
    output = CustomMsg()
    output.header = message.header
    if not rows:
        return output
    timebase = min(int(row[6]) for row in rows)
    output.timebase = timebase
    output.lidar_id = 0
    output.rsvd = [0, 0, 0]
    output.points = [CustomPoint(
        offset_time=int(row[6]) - timebase,
        x=float(row[0]), y=float(row[1]), z=float(row[2]),
        reflectivity=max(0, min(255, round(float(row[3])))),
        tag=int(row[4]), line=int(row[5]),
    ) for row in rows]
    output.point_num = len(output.points)
    return output


def convert_bag(input_path, output_path, max_lidar_messages=None):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(input_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    writer.create_topic(rosbag2_py.TopicMetadata(
        name=OUTPUT_LIDAR_TOPIC,
        type="livox_ros_driver2/msg/CustomMsg",
        serialization_format="cdr",
    ))
    writer.create_topic(rosbag2_py.TopicMetadata(
        name=OUTPUT_IMU_TOPIC,
        type="sensor_msgs/msg/Imu",
        serialization_format="cdr",
    ))
    lidar_count = 0
    imu_count = 0
    stop_timestamp = None
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if stop_timestamp is not None and timestamp > stop_timestamp:
            break
        if topic == INPUT_LIDAR_TOPIC:
            if max_lidar_messages is not None and lidar_count >= max_lidar_messages:
                stop_timestamp = timestamp
                continue
            cloud = deserialize_message(data, PointCloud2)
            writer.write(
                OUTPUT_LIDAR_TOPIC,
                serialize_message(pointcloud_to_custom(cloud)),
                timestamp,
            )
            lidar_count += 1
        elif topic == INPUT_IMU_TOPIC:
            writer.write(
                OUTPUT_IMU_TOPIC,
                serialize_message(deserialize_message(data, Imu)),
                timestamp,
            )
            imu_count += 1
    return lidar_count, imu_count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--max-lidar-messages", type=int)
    args = parser.parse_args()
    if args.output.exists():
        parser.error(f"output already exists: {args.output}")
    lidar_count, imu_count = convert_bag(
        args.input, args.output, args.max_lidar_messages
    )
    print(f"converted lidar={lidar_count}, imu={imu_count}, output={args.output}")


if __name__ == "__main__":
    main()
