from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("agt_ui_bridge"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("semantic_map", default_value=""),
            DeclareLaunchArgument("platform_profile", default_value=""),
            DeclareLaunchArgument(
                "config",
                default_value=str(
                    package_share / "config" / "semantic_editor.yaml"
                ),
            ),
            Node(
                package="agt_ui_bridge",
                executable="semantic_editor_qt5.py",
                name="agt_semantic_editor",
                output="screen",
                arguments=[
                    "--map",
                    LaunchConfiguration("map"),
                    "--semantic-map",
                    LaunchConfiguration("semantic_map"),
                    "--platform-profile",
                    LaunchConfiguration("platform_profile"),
                    "--config",
                    LaunchConfiguration("config"),
                ],
            ),
        ]
    )
