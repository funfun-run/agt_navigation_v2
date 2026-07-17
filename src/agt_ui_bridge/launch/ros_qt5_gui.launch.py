from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile", default_value="navigation", choices=["mapping", "navigation"]
            ),
            DeclareLaunchArgument("start_map_io_bridge", default_value="true"),
            DeclareLaunchArgument(
                "source_map_topic", default_value="/agt/map/global_occupancy"
            ),
            DeclareLaunchArgument("map_frame_id", default_value="map"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="agt_ui_bridge",
                executable="map_io_bridge.py",
                name="agt_map_io_bridge",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_map_io_bridge")),
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "source_map_topic": LaunchConfiguration("source_map_topic"),
                        "frame_id": LaunchConfiguration("map_frame_id"),
                    }
                ],
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "agt_ui_bridge",
                    "start_ros_qt5_gui_app.sh",
                    "--profile",
                    LaunchConfiguration("profile"),
                ],
                output="screen",
            ),
        ]
    )
