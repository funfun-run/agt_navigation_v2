from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("agt_perception"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(share / "config" / "local_obstacle_filter.yaml"),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="agt_perception",
                executable="local_obstacle_filter",
                name="agt_local_obstacle_filter",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
