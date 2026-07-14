from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("agt_ui_bridge"))
    default_config = package_share / "config" / "semantic_map_server.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("semantic_map", default_value=""),
            DeclareLaunchArgument("platform_profile", default_value=""),
            DeclareLaunchArgument(
                "base_map_topic", default_value="/agt/map/global_occupancy"
            ),
            DeclareLaunchArgument("outside_field_is_keepout", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("config", default_value=str(default_config)),
            Node(
                package="agt_ui_bridge",
                executable="semantic_map_server.py",
                name="agt_semantic_map_server",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "semantic_map": LaunchConfiguration("semantic_map"),
                        "platform_profile": LaunchConfiguration("platform_profile"),
                        "base_map_topic": LaunchConfiguration("base_map_topic"),
                        "outside_field_is_keepout": LaunchConfiguration(
                            "outside_field_is_keepout"
                        ),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
            ),
        ]
    )
