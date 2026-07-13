from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_map_io_bridge", default_value="true"),
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
                        )
                    }
                ],
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "agt_ui_bridge", "start_ros_qt5_gui_app.sh"],
                output="screen",
            ),
        ]
    )
