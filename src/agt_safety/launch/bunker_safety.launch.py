from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = Path(get_package_share_directory("agt_safety")) / "config" / "bunker_safety.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "safety_config",
                default_value=str(default_config),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="agt_safety",
                executable="tracked_safety_controller.py",
                name="agt_tracked_safety_controller",
                output="screen",
                parameters=[
                    LaunchConfiguration("safety_config"),
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
