from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


RECORDED_TOPICS = [
    "/clock",
    "/tf",
    "/tf_static",
    "/agt/sensors/lidar/custom",
    "/agt/sensors/imu/data",
    "/agt/mapping/odometry",
    "/agt/mapping/registered_points",
    "/agt/mapping/registered_points_lidar",
    "/agt/map/mapping_occupancy",
    "/agt/map/global_occupancy",
    "/agt/map/semantic_markers",
    "/agt/map/keepout_mask",
    "/agt/map/keepout_filter_info",
    "/agt/map/semantic_status",
    "/agt/coverage/path_raw",
    "/agt/coverage/path_components",
    "/agt/coverage/path_reconstructed",
    "/agt/coverage/path_semantics",
    "/agt/coverage/swaths",
    "/agt/coverage/headland",
    "/agt/coverage/path_validated",
    "/agt/coverage/path_repaired",
    "/agt/coverage/collision_poses",
    "/agt/coverage/footprint_markers",
    "/agt/coverage/status",
    "/agt/coverage/validation_report",
    "/agt/coverage/repair_report",
    "/agt/coverage/task_status",
    "/global_costmap/costmap",
    "/global_costmap/published_footprint",
    "/local_costmap/costmap",
    "/agt/perception/obstacle_cloud",
    "/agt/localization/status",
    "/agt/navigation/status",
    "/agt/navigation/cmd_vel_raw",
    "/agt/navigation/cmd_vel",
    "/agt/cmd_vel_manual",
    "/agt/safety/cmd_vel",
    "/agt/safety/emergency_stop",
    "/agt/safety/status",
    "/agt/chassis/cmd_vel",
    "/agt/chassis/odometry",
    "/agt/chassis/status",
    "/agt/chassis/connected",
    "/agt/chassis/rc_state",
    "/goal_pose",
    "/initialpose",
]


def default_runtime_dir():
    share = Path(get_package_share_directory("agt_bringup"))
    return str(share.parents[3] / "runtime")


def prepare_output(context):
    runtime_dir = Path(LaunchConfiguration("runtime_dir").perform(context))
    runtime_dir.joinpath("rosbag").mkdir(parents=True, exist_ok=True)
    return []


def generate_launch_description():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return LaunchDescription(
        [
            DeclareLaunchArgument("runtime_dir", default_value=default_runtime_dir()),
            DeclareLaunchArgument("bag_name", default_value=f"agt_system_{timestamp}"),
            OpaqueFunction(function=prepare_output),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "--storage",
                    "sqlite3",
                    "--output",
                    PathJoinSubstitution(
                        [
                            LaunchConfiguration("runtime_dir"),
                            "rosbag",
                            LaunchConfiguration("bag_name"),
                        ]
                    ),
                    *RECORDED_TOPICS,
                ],
                output="screen",
                sigterm_timeout="10",
                sigkill_timeout="5",
            ),
        ]
    )
