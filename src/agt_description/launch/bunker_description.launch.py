from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("agt_description"))
    with (share / "config" / "bunker_mid360.yaml").open(encoding="utf-8") as stream:
        parameters = yaml.safe_load(stream)["/**"]["ros__parameters"]
    arguments = {
        key: str(value).lower()
        for key, value in parameters.items()
        if key != "calibration_verified"
    }
    arguments["use_sim_time"] = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / "launch" / "description.launch.py")),
                launch_arguments=arguments.items(),
            )
        ]
    )
