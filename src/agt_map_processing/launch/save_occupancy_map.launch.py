from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_topic", default_value="/agt/map/mapping_occupancy"
            ),
            DeclareLaunchArgument(
                "map_prefix", default_value="runtime/maps/mid360_map"
            ),
            DeclareLaunchArgument("image_format", default_value="pgm"),
            DeclareLaunchArgument("save_map_timeout", default_value="20.0"),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "nav2_map_server",
                    "map_saver_cli",
                    "-t",
                    LaunchConfiguration("map_topic"),
                    "-f",
                    LaunchConfiguration("map_prefix"),
                    "--fmt",
                    LaunchConfiguration("image_format"),
                    "--ros-args",
                    "-p",
                    "map_subscribe_transient_local:=true",
                    "-p",
                    ["save_map_timeout:=", LaunchConfiguration("save_map_timeout")],
                ],
                output="screen",
            ),
        ]
    )
