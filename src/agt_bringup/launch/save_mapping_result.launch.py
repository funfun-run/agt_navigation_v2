from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def default_runtime_dir():
    share = Path(get_package_share_directory("agt_bringup"))
    return str(share.parents[3] / "runtime")


def prepare_map_directory(context):
    runtime_dir = Path(LaunchConfiguration("runtime_dir").perform(context))
    map_name = LaunchConfiguration("map_name").perform(context)
    runtime_dir.joinpath("maps", map_name).mkdir(parents=True, exist_ok=True)
    return []


def generate_launch_description():
    prefix = PathJoinSubstitution(
        [
            LaunchConfiguration("runtime_dir"),
            "maps",
            LaunchConfiguration("map_name"),
            LaunchConfiguration("map_name"),
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("runtime_dir", default_value=default_runtime_dir()),
            DeclareLaunchArgument("map_name", default_value="mid360_map"),
            DeclareLaunchArgument("map_topic", default_value="/agt/map/global_occupancy"),
            OpaqueFunction(function=prepare_map_directory),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "nav2_map_server",
                    "map_saver_cli",
                    "-t",
                    LaunchConfiguration("map_topic"),
                    "-f",
                    prefix,
                    "--fmt",
                    "pgm",
                    "--ros-args",
                    "-p",
                    "map_subscribe_transient_local:=true",
                    "-p",
                    "save_map_timeout:=20.0",
                ],
                output="screen",
            ),
        ]
    )
