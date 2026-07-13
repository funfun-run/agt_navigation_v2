from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def share(package):
    return Path(get_package_share_directory(package))


def include(package, launch_file, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share(package) / "launch" / launch_file)),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def default_runtime_dir():
    return str(share("agt_bringup").parents[3] / "runtime")


def prepare_runtime(context):
    runtime_dir = Path(LaunchConfiguration("runtime_dir").perform(context))
    map_name = LaunchConfiguration("map_name").perform(context)
    runtime_dir.joinpath("maps", map_name, "pcd").mkdir(parents=True, exist_ok=True)
    runtime_dir.joinpath("rosbag").mkdir(parents=True, exist_ok=True)
    return []


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    runtime_dir = LaunchConfiguration("runtime_dir")
    map_name = LaunchConfiguration("map_name")
    pcd_dir = PathJoinSubstitution([runtime_dir, "maps", map_name, "pcd"])
    bag_name = f"mapping_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    return LaunchDescription(
        [
            DeclareLaunchArgument("runtime_dir", default_value=default_runtime_dir()),
            DeclareLaunchArgument("map_name", default_value="mid360_map"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_sensor", default_value="true"),
            DeclareLaunchArgument("start_chassis", default_value="true"),
            DeclareLaunchArgument("start_gui", default_value="true"),
            DeclareLaunchArgument("record_bag", default_value="false"),
            OpaqueFunction(function=prepare_runtime),
            include(
                "agt_description",
                "bunker_description.launch.py",
                {"use_sim_time": use_sim_time},
            ),
            include(
                "agt_sensor_adapters",
                "mid360.launch.py",
                condition=IfCondition(LaunchConfiguration("start_sensor")),
            ),
            include(
                "agt_mapping",
                "fast_livo2_mapping.launch.py",
                {
                    "params_file": str(share("agt_mapping") / "config" / "mid360_lio_only.yaml"),
                    "camera_params_file": str(
                        share("agt_mapping") / "config" / "camera_disabled_placeholder.yaml"
                    ),
                    "use_sim_time": use_sim_time,
                    "save_pcd": "true",
                    "pcd_save_interval": "-1",
                    "pcd_output_dir": pcd_dir,
                },
            ),
            include(
                "agt_map_processing",
                "octomap_projection.launch.py",
                {
                    "params_file": str(
                        share("agt_map_processing") / "config" / "octomap_projection.yaml"
                    ),
                    "use_sim_time": use_sim_time,
                },
            ),
            include(
                "agt_chassis",
                "bunker.launch.py",
                {"use_sim_time": use_sim_time},
                IfCondition(LaunchConfiguration("start_chassis")),
            ),
            include(
                "agt_ui_bridge",
                "ros_qt5_gui.launch.py",
                {"use_sim_time": use_sim_time},
                IfCondition(LaunchConfiguration("start_gui")),
            ),
            include(
                "agt_bringup",
                "bag_record.launch.py",
                {"runtime_dir": runtime_dir, "bag_name": bag_name},
                IfCondition(LaunchConfiguration("record_bag")),
            ),
        ]
    )
