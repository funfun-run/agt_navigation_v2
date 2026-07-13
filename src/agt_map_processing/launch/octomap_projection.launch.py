from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("agt_map_processing"))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(package_share / "config" / "octomap_projection.yaml"),
            ),
            DeclareLaunchArgument(
                "cloud_topic", default_value="/agt/mapping/registered_points_lidar"
            ),
            DeclareLaunchArgument(
                "map_topic", default_value="/agt/map/global_occupancy"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                name="agt_map_processing_octomap",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        )
                    },
                ],
                remappings=[
                    ("cloud_in", LaunchConfiguration("cloud_topic")),
                    ("projected_map", LaunchConfiguration("map_topic")),
                ],
            ),
        ]
    )
