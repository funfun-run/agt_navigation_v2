from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mapping_share = Path(get_package_share_directory("agt_mapping"))
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=str(mapping_share / "config" / "mid360_lio_only.yaml")),
        DeclareLaunchArgument(
            "camera_params_file",
            default_value=str(mapping_share / "config" / "camera_disabled_placeholder.yaml"),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("save_pcd", default_value="false"),
        DeclareLaunchArgument("pcd_save_interval", default_value="-1"),
        DeclareLaunchArgument("pcd_output_dir", default_value="runtime/maps/fast_livo2"),
        Node(
            package="fast_livo", executable="fastlivo_mapping", name="fast_livo2_backend",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                LaunchConfiguration("camera_params_file"), {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "common.publish_tf": False,
                "pcd_save.pcd_save_en": ParameterValue(
                    LaunchConfiguration("save_pcd"), value_type=bool
                ),
                "pcd_save.interval": ParameterValue(
                    LaunchConfiguration("pcd_save_interval"), value_type=int
                ),
                "pcd_save.output_directory": LaunchConfiguration("pcd_output_dir"),
            }],
            remappings=[
                ("/cloud_registered", "/agt/mapping/backend/registered_points"),
                (
                    "/cloud_registered_lidar",
                    "/agt/mapping/registered_points_lidar",
                ),
            ],
        ),
        Node(
            package="agt_mapping", executable="fast_livo2_adapter.py",
            name="agt_mapping_fast_livo2_adapter", output="screen",
            parameters=[str(mapping_share / "config" / "fast_livo2_adapter.yaml"), {
                "use_sim_time": LaunchConfiguration("use_sim_time")
            }],
        ),
    ])
