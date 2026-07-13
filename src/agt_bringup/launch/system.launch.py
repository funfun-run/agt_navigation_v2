from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def bringup_launch(name):
    share = Path(get_package_share_directory("agt_bringup"))
    return PythonLaunchDescriptionSource(str(share / "launch" / name))


def validate_mode_arguments(context):
    mode = LaunchConfiguration("mode").perform(context)
    if mode == "mapping":
        map_name = LaunchConfiguration("map_name").perform(context)
        if not re.fullmatch(r"[A-Za-z0-9_-]+", map_name):
            raise RuntimeError("map_name may contain only letters, numbers, '_' and '-'")
        return []

    for argument in ("map", "global_map_pcd"):
        value = LaunchConfiguration(argument).perform(context)
        if not value:
            raise RuntimeError(f"navigation mode requires {argument}:=/absolute/path")
        if not Path(value).is_file():
            raise RuntimeError(f"navigation mode {argument} file does not exist: {value}")
    return []


def generate_launch_description():
    common = {
        "runtime_dir": LaunchConfiguration("runtime_dir"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "start_sensor": LaunchConfiguration("start_sensor"),
        "start_chassis": LaunchConfiguration("start_chassis"),
        "start_gui": LaunchConfiguration("start_gui"),
        "record_bag": LaunchConfiguration("record_bag"),
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="mapping", choices=["mapping", "navigation"]),
            DeclareLaunchArgument(
                "runtime_dir",
                default_value=str(Path(get_package_share_directory("agt_bringup")).parents[3] / "runtime"),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_sensor", default_value="true"),
            DeclareLaunchArgument("start_chassis", default_value="true"),
            DeclareLaunchArgument("start_gui", default_value="true"),
            DeclareLaunchArgument("record_bag", default_value="false"),
            DeclareLaunchArgument("map_name", default_value="mid360_map"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("global_map_pcd", default_value=""),
            DeclareLaunchArgument("backend", default_value="ndt"),
            OpaqueFunction(function=validate_mode_arguments),
            LogInfo(msg=["AGT system mode: ", LaunchConfiguration("mode")]),
            IncludeLaunchDescription(
                bringup_launch("mapping_mode.launch.py"),
                launch_arguments={**common, "map_name": LaunchConfiguration("map_name")}.items(),
                condition=LaunchConfigurationEquals("mode", "mapping"),
            ),
            IncludeLaunchDescription(
                bringup_launch("navigation_system.launch.py"),
                launch_arguments={
                    **common,
                    "map": LaunchConfiguration("map"),
                    "global_map_pcd": LaunchConfiguration("global_map_pcd"),
                    "backend": LaunchConfiguration("backend"),
                }.items(),
                condition=LaunchConfigurationEquals("mode", "navigation"),
            ),
        ]
    )
