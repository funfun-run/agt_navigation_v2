import ast
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
NAV_CONFIG = ROOT / "src/agt_navigation/config/nav2_bunker.yaml"


def load_navigation_config():
    return yaml.safe_load(NAV_CONFIG.read_text(encoding="utf-8"))


def test_tracked_nav2_plugins_and_frames_are_selected():
    config = load_navigation_config()
    planner = config["planner_server"]["ros__parameters"]["GridBased"]
    controller = config["controller_server"]["ros__parameters"]["FollowPath"]
    assert planner["plugin"] == "nav2_smac_planner/SmacPlanner2D"
    assert controller["plugin"] == "nav2_mppi_controller::MPPIController"
    assert controller["motion_model"] == "DiffDrive"
    assert config["bt_navigator"]["ros__parameters"]["robot_base_frame"] == "base_footprint"


def test_navigation_command_chain_has_collision_and_safety_boundaries():
    config = load_navigation_config()["collision_monitor"]["ros__parameters"]
    assert config["cmd_vel_in_topic"] == "/agt/navigation/cmd_vel_raw"
    assert config["cmd_vel_out_topic"] == "/agt/navigation/cmd_vel"
    safety = (ROOT / "src/agt_safety/scripts/tracked_safety_controller.py").read_text()
    assert 'Twist, "/agt/navigation/cmd_vel"' in safety
    assert 'create_publisher(Twist, "/agt/safety/cmd_vel"' in safety


def test_costmap_and_collision_monitor_share_obstacle_cloud():
    config = load_navigation_config()
    local = config["local_costmap"]["local_costmap"]["ros__parameters"]
    collision = config["collision_monitor"]["ros__parameters"]
    assert local["voxel_layer"]["obstacle_cloud"]["topic"] == "/agt/perception/obstacle_cloud"
    assert config["map_server"]["ros__parameters"]["topic_name"] == "/agt/map/global_occupancy"
    assert collision["obstacle_cloud"]["topic"] == "/agt/perception/obstacle_cloud"
    assert "velocity_smoother" not in config
    assert "amcl" not in config


def test_global_costmap_applies_keepout_before_inflation():
    config = load_navigation_config()
    global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
    assert global_costmap["plugins"] == ["static_layer"]
    assert global_costmap["filters"] == [
        "keepout_filter",
        "keepout_inflation_layer",
    ]
    assert global_costmap["keepout_filter"] == {
        "plugin": "nav2_costmap_2d::KeepoutFilter",
        "enabled": True,
        "filter_info_topic": "/agt/map/keepout_filter_info",
        "transform_tolerance": 0.25,
    }
    assert global_costmap["keepout_inflation_layer"]["plugin"] == (
        "nav2_costmap_2d::InflationLayer"
    )
    assert global_costmap["keepout_inflation_layer"]["inflation_radius"] == 0.75


def test_keepout_info_server_uses_separate_transient_semantic_mask():
    config = load_navigation_config()
    info = config["costmap_filter_info_server"]["ros__parameters"]
    assert info == {
        "use_sim_time": False,
        "type": 0,
        "filter_info_topic": "/agt/map/keepout_filter_info",
        "mask_topic": "/agt/map/keepout_mask",
        "base": 0.0,
        "multiplier": 1.0,
    }
    assert config["map_server"]["ros__parameters"]["topic_name"] == (
        "/agt/map/global_occupancy"
    )
    assert info["mask_topic"] != config["map_server"]["ros__parameters"]["topic_name"]


def test_keepout_info_server_is_explicitly_opt_in_and_lifecycle_managed():
    launch_path = ROOT / "src/agt_navigation/launch/navigation.launch.py"
    launch = launch_path.read_text(encoding="utf-8")
    ast.parse(launch)
    assert '"use_keepout_filter"' in launch
    assert 'default_value="false"' in launch
    assert 'executable="costmap_filter_info_server"' in launch
    assert 'name="lifecycle_manager_keepout_filter"' in launch
    assert '{"node_names": ["costmap_filter_info_server"]}' in launch
    assert launch.count(
        'condition=IfCondition(LaunchConfiguration("use_keepout_filter"))'
    ) == 2


def test_offline_map_and_navigation_launch_are_self_contained():
    map_yaml = yaml.safe_load(
        (ROOT / "src/agt_navigation/maps/offline_test.yaml").read_text()
    )
    assert map_yaml["resolution"] > 0.0
    assert (ROOT / "src/agt_navigation/maps" / map_yaml["image"]).exists()
    launch = (ROOT / "src/agt_navigation/launch/offline_navigation.launch.py").read_text()
    assert "differential_drive_simulator.py" in launch
    assert '"startup_motion_enabled": True' in launch
    assert '"synthetic_obstacle_enabled"' in launch
    ast.parse(launch)


def test_qt_goal_pose_is_bridged_to_nav2_action():
    bridge = (ROOT / "src/agt_navigation/scripts/goal_pose_bridge.py").read_text()
    assert 'NavigateToPose, "navigate_to_pose"' in bridge
    assert 'PoseStamped, "/goal_pose"' in bridge
    assert '"/agt/navigation/status"' in bridge


def test_local_obstacle_filter_uses_base_frame_and_robot_crop():
    config = yaml.safe_load(
        (ROOT / "src/agt_perception/config/local_obstacle_filter.yaml").read_text()
    )["agt_local_obstacle_filter"]["ros__parameters"]
    assert config["input_topic"] == "/agt/mapping/registered_points_lidar"
    assert config["output_topic"] == "/agt/perception/obstacle_cloud"
    assert config["target_frame"] == "base_footprint"
    assert config["robot_min_x"] < 0.0 < config["robot_max_x"]
    assert config["robot_min_y"] < 0.0 < config["robot_max_y"]


def test_system_bringup_separates_mapping_and_navigation_modes():
    system = (ROOT / "src/agt_bringup/launch/system.launch.py").read_text()
    mapping = (ROOT / "src/agt_bringup/launch/mapping_mode.launch.py").read_text()
    navigation = (ROOT / "src/agt_bringup/launch/navigation_system.launch.py").read_text()
    assert 'choices=["mapping", "navigation"]' in system
    assert '"save_pcd": "true"' in mapping
    assert '"save_pcd": "false"' in navigation
    assert mapping.count('"bunker_description.launch.py"') == 1
    assert navigation.count('"bunker_description.launch.py"') == 1
    assert 'package="rviz2"' in mapping
    assert '"ros_qt5_gui.launch.py"' not in mapping
    assert '"ros_qt5_gui.launch.py"' in navigation
    assert '"map_topic": "/agt/map/mapping_occupancy"' in mapping
    assert "/agt/map/global_occupancy" in (
        ROOT / "src/agt_navigation/config/nav2_bunker.yaml"
    ).read_text()


def test_bag_recorder_captures_debugging_contracts():
    recorder = (ROOT / "src/agt_bringup/launch/bag_record.launch.py").read_text()
    for topic in (
        "/agt/sensors/lidar/custom",
        "/agt/mapping/odometry",
        "/agt/map/mapping_occupancy",
        "/agt/map/global_occupancy",
        "/agt/navigation/cmd_vel_raw",
        "/agt/safety/cmd_vel",
        "/tf",
    ):
        assert f'"{topic}"' in recorder


def test_fast_livo_pcd_output_is_runtime_configurable():
    source = (ROOT / "third_party/fast_livo2_ros2/src/LIVMapper.cpp").read_text()
    mapping_launch = (
        ROOT / "src/agt_mapping/launch/fast_livo2_mapping.launch.py"
    ).read_text()
    assert '"pcd_save.output_directory"' in source
    assert "std::filesystem::create_directories(pcd_output_directory)" in source
    assert '"pcd_save.pcd_save_en"' in mapping_launch
    assert '"pcd_save.output_directory"' in mapping_launch
