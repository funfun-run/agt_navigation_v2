from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("agt_ui_bridge"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file", default_value=str(package_share / "config" / "map_io.yaml")
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="agt_ui_bridge",
                executable="map_io_bridge.py",
                name="agt_map_io_bridge",
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
            Node(
                package="agt_ui_bridge",
                executable="map_editor_qt5.py",
                name="agt_qt5_map_editor",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        )
                    }
                ],
            ),
        ]
    )
