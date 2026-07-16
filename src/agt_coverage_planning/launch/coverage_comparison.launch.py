"""Visualization-only multi-variant coverage comparison."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    coverage_share = Path(get_package_share_directory("agt_coverage_planning"))
    ui_share = Path(get_package_share_directory("agt_ui_bridge"))
    parameters = str(coverage_share / "config/coverage_planning.yaml")
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": LaunchConfiguration("map"),
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[("map", "/agt/map/global_occupancy")],
    )
    map_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_coverage_comparison_map",
        output="screen",
        parameters=[
            {
                "autostart": True,
                "node_names": ["map_server"],
                "use_sim_time": use_sim_time,
            }
        ],
    )
    semantic_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ui_share / "launch/semantic_map_server.launch.py")
        ),
        launch_arguments={
            "semantic_map": LaunchConfiguration("semantic_map"),
            "platform_profile": LaunchConfiguration("platform_profile"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )
    coverage_server = Node(
        package="opennav_coverage",
        executable="opennav_coverage",
        namespace="agt/coverage/polygon",
        name="coverage_server",
        output="screen",
        parameters=[parameters, {"use_sim_time": use_sim_time}],
    )
    coverage_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        namespace="agt/coverage/polygon",
        name="lifecycle_manager_coverage_comparison",
        output="screen",
        parameters=[
            {
                "autostart": True,
                "node_names": ["coverage_server"],
                "use_sim_time": use_sim_time,
            }
        ],
    )
    comparator = Node(
        package="agt_coverage_planning",
        executable="coverage_variant_comparator.py",
        name="coverage_variant_comparator",
        output="screen",
        parameters=[
            parameters,
            {
                "semantic_map": LaunchConfiguration("semantic_map"),
                "platform_profile": LaunchConfiguration("platform_profile"),
                "variants_file": LaunchConfiguration("variants_file"),
                "report_path": LaunchConfiguration("report_path"),
                "auto_compare": True,
                "use_sim_time": use_sim_time,
            },
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="coverage_comparison_rviz",
        output="screen",
        arguments=["-d", str(coverage_share / "rviz/coverage_preview.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("semantic_map"),
            DeclareLaunchArgument("platform_profile"),
            DeclareLaunchArgument(
                "variants_file",
                default_value=str(coverage_share / "config/coverage_variants.yaml"),
            ),
            DeclareLaunchArgument("report_path", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            map_server,
            map_manager,
            semantic_server,
            coverage_server,
            coverage_manager,
            comparator,
            rviz,
        ]
    )
