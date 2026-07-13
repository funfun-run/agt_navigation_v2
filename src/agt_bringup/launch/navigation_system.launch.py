from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from datetime import datetime

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(package, launch_file, arguments=None, condition=None):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / launch_file)),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    runtime_dir = LaunchConfiguration("runtime_dir")
    bag_name = f"navigation_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value="", description="Occupancy-grid map YAML used by Nav2"),
            DeclareLaunchArgument("global_map_pcd", default_value="", description="PCD map used by ICP/NDT relocalization"),
            DeclareLaunchArgument(
                "runtime_dir",
                default_value=str(Path(get_package_share_directory("agt_bringup")).parents[3] / "runtime"),
            ),
            DeclareLaunchArgument("backend", default_value="ndt"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_sensor", default_value="true"),
            DeclareLaunchArgument("start_chassis", default_value="true"),
            DeclareLaunchArgument("start_gui", default_value="true"),
            DeclareLaunchArgument("record_bag", default_value="false"),
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
                    "params_file": str(
                        Path(get_package_share_directory("agt_mapping"))
                        / "config"
                        / "mid360_lio_only.yaml"
                    ),
                    "camera_params_file": str(
                        Path(get_package_share_directory("agt_mapping"))
                        / "config"
                        / "camera_disabled_placeholder.yaml"
                    ),
                    "use_sim_time": use_sim_time,
                    "save_pcd": "false",
                },
            ),
            include(
                "agt_perception",
                "local_obstacles.launch.py",
                {
                    "params_file": str(
                        Path(get_package_share_directory("agt_perception"))
                        / "config"
                        / "local_obstacle_filter.yaml"
                    ),
                    "use_sim_time": use_sim_time,
                },
            ),
            include(
                "agt_localization",
                "relocalization.launch.py",
                {
                    "params_file": str(
                        Path(get_package_share_directory("agt_localization"))
                        / "config"
                        / "relocalization.yaml"
                    ),
                    "global_map_pcd": LaunchConfiguration("global_map_pcd"),
                    "backend": LaunchConfiguration("backend"),
                    "use_sim_time": use_sim_time,
                },
            ),
            include(
                "agt_navigation",
                "navigation.launch.py",
                {
                    "params_file": str(
                        Path(get_package_share_directory("agt_navigation"))
                        / "config"
                        / "nav2_bunker.yaml"
                    ),
                    "map": LaunchConfiguration("map"),
                    "use_sim_time": use_sim_time,
                },
            ),
            include(
                "agt_safety",
                "bunker_safety.launch.py",
                {"use_sim_time": use_sim_time},
                UnlessCondition(LaunchConfiguration("start_chassis")),
            ),
            include(
                "agt_chassis",
                "bunker.launch.py",
                {"use_sim_time": use_sim_time, "start_safety": "true"},
                condition=IfCondition(LaunchConfiguration("start_chassis")),
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
