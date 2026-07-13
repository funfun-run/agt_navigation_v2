from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("agt_sensor_adapters"))
    default_config = package_share / "config" / "mid360_network.json"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "user_config_path",
                default_value=str(default_config),
                description="Livox MID360 network configuration JSON",
            ),
            DeclareLaunchArgument("publish_freq", default_value="10.0"),
            DeclareLaunchArgument("frame_id", default_value="livox_frame"),
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="agt_sensor_mid360_driver",
                output="screen",
                parameters=[
                    {
                        # FASTLIVO2_ROS2 consumes per-point timing from CustomMsg.
                        "xfer_format": 1,
                        "multi_topic": 0,
                        "data_src": 0,
                        "publish_freq": LaunchConfiguration("publish_freq"),
                        "output_data_type": 0,
                        "frame_id": LaunchConfiguration("frame_id"),
                        "user_config_path": LaunchConfiguration("user_config_path"),
                    }
                ],
                remappings=[
                    ("/livox/lidar", "/agt/sensors/lidar/custom"),
                    ("/livox/imu", "/agt/sensors/imu/data"),
                ],
            ),
        ]
    )
