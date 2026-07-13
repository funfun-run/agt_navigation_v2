from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("agt_localization"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(package_share / "config" / "relocalization.yaml"),
            ),
            DeclareLaunchArgument("global_map_pcd", default_value=""),
            DeclareLaunchArgument("backend", default_value="ndt"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="agt_localization",
                executable="relocalization_node",
                name="agt_relocalization",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "global_map_pcd": LaunchConfiguration("global_map_pcd"),
                        "backend": LaunchConfiguration("backend"),
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
