from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


XACRO_ARGUMENTS = (
    ("base_length", "Provisional chassis length in metres"),
    ("base_width", "Provisional chassis width in metres"),
    ("base_height", "Provisional chassis height in metres"),
    ("base_link_z", "base_link height above the ground plane"),
    ("lidar_x", "MID360 x offset from base_link"),
    ("lidar_y", "MID360 y offset from base_link"),
    ("lidar_z", "MID360 z offset from base_link"),
    ("lidar_roll", "MID360 roll in radians"),
    ("lidar_pitch", "MID360 pitch in radians"),
    ("lidar_yaw", "MID360 yaw in radians"),
)


def generate_launch_description():
    package_share = Path(get_package_share_directory("agt_description"))
    model = package_share / "urdf" / "agt_base.urdf.xacro"
    calibration_file = package_share / "config" / "mk_mini_mid360.yaml"
    with calibration_file.open(encoding="utf-8") as stream:
        defaults = yaml.safe_load(stream)["/**"]["ros__parameters"]

    declarations = [
        DeclareLaunchArgument(
            name, default_value=str(defaults[name]).lower(), description=description
        )
        for name, description in XACRO_ARGUMENTS
    ]
    xacro_arguments = []
    for name, _ in XACRO_ARGUMENTS:
        xacro_arguments.extend([f" {name}:=", LaunchConfiguration(name)])

    robot_description = ParameterValue(
        Command(["xacro ", str(model), *xacro_arguments]),
        value_type=str,
    )

    return LaunchDescription(
        [
            *declarations,
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="agt_robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": robot_description},
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
