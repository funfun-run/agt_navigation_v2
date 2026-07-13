from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("agt_navigation"))
    params = LaunchConfiguration("params_file")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    common = [params, {"use_sim_time": use_sim_time}]
    managed_nodes = [
        "map_server",
        "planner_server",
        "smoother_server",
        "controller_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "collision_monitor",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file", default_value=str(share / "config" / "nav2_bunker.yaml")
            ),
            DeclareLaunchArgument("map", default_value=str(share / "maps" / "offline_test.yaml")),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[params, {"yaml_filename": LaunchConfiguration("map"), "use_sim_time": use_sim_time}],
            ),
            Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen", parameters=common),
            Node(package="nav2_smoother", executable="smoother_server", name="smoother_server", output="screen", parameters=common),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=common,
                remappings=[("cmd_vel", "/agt/navigation/cmd_vel_raw")],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=common,
                remappings=[("cmd_vel", "/agt/navigation/cmd_vel_raw")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[
                    params,
                    {
                        "use_sim_time": use_sim_time,
                        "default_nav_to_pose_bt_xml": str(
                            share / "behavior_trees" / "navigate_to_pose.xml"
                        ),
                    },
                ],
            ),
            Node(package="nav2_waypoint_follower", executable="waypoint_follower", name="waypoint_follower", output="screen", parameters=common),
            Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor", output="screen", parameters=common),
            Node(
                package="agt_navigation",
                executable="goal_pose_bridge.py",
                name="agt_goal_pose_bridge",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": ParameterValue(LaunchConfiguration("autostart"), value_type=bool)},
                    {"node_names": managed_nodes},
                    {"bond_timeout": 4.0},
                ],
            ),
        ]
    )
