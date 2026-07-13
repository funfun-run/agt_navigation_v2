from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav_share = Path(get_package_share_directory("agt_navigation"))
    description_share = Path(get_package_share_directory("agt_description"))
    safety_share = Path(get_package_share_directory("agt_safety"))

    return LaunchDescription(
        [
            DeclareLaunchArgument("synthetic_obstacle_enabled", default_value="false"),
            DeclareLaunchArgument("synthetic_obstacle_x", default_value="0.7"),
            DeclareLaunchArgument("synthetic_obstacle_y", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(description_share / "launch" / "bunker_description.launch.py"))
            ),
            Node(
                package="agt_navigation",
                executable="differential_drive_simulator.py",
                name="agt_offline_base_simulator",
                output="screen",
                parameters=[
                    {
                        "synthetic_obstacle_enabled": ParameterValue(
                            LaunchConfiguration("synthetic_obstacle_enabled"), value_type=bool
                        ),
                        "synthetic_obstacle_x": ParameterValue(
                            LaunchConfiguration("synthetic_obstacle_x"), value_type=float
                        ),
                        "synthetic_obstacle_y": ParameterValue(
                            LaunchConfiguration("synthetic_obstacle_y"), value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="agt_safety",
                executable="tracked_safety_controller.py",
                name="agt_tracked_safety_controller",
                output="screen",
                parameters=[
                    str(safety_share / "config" / "bunker_safety.yaml"),
                    {"startup_motion_enabled": True},
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(nav_share / "launch" / "navigation.launch.py")),
                launch_arguments={
                    "map": str(nav_share / "maps" / "offline_test.yaml"),
                    "params_file": str(nav_share / "config" / "nav2_bunker.yaml"),
                    "use_sim_time": "false",
                    "autostart": "true",
                }.items(),
            ),
        ]
    )
