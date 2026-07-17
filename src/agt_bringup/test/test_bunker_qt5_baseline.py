from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]


def read(relative_path):
    return (REPOSITORY_ROOT / relative_path).read_text(encoding="utf-8")


def test_mapping_mode_starts_mapping_gui():
    source = read("src/agt_bringup/launch/mapping_mode.launch.py")
    assert 'DeclareLaunchArgument("start_gui", default_value="true")' in source
    assert '"ros_qt5_gui.launch.py"' in source
    assert '"profile": "mapping"' in source
    assert '"source_map_topic": "/agt/map/mapping_occupancy"' in source
    assert '"map_frame_id": "odom"' in source
    assert "map_saver" not in source


def test_navigation_mode_starts_navigation_gui_and_defaults_optional_features_off():
    source = read("src/agt_bringup/launch/navigation_system.launch.py")
    assert '"profile": "navigation"' in source
    assert '"source_map_topic": "/agt/map/global_occupancy"' in source
    assert '"map_frame_id": "map"' in source
    assert 'DeclareLaunchArgument("start_semantic_map_server", default_value="false")' in source
    assert 'DeclareLaunchArgument("start_coverage_planning", default_value="false")' in source


def test_system_passes_gui_to_both_modes_and_keeps_optional_features_off():
    source = read("src/agt_bringup/launch/system.launch.py")
    assert source.count('"start_gui": LaunchConfiguration("start_gui")') == 2
    assert 'DeclareLaunchArgument("start_semantic_map_server", default_value="false")' in source
    assert 'DeclareLaunchArgument("start_coverage_planning", default_value="false")' in source


def test_nav_velocity_passes_collision_monitor_and_safety():
    nav_launch = read("src/agt_navigation/launch/navigation.launch.py")
    nav_config = read("src/agt_navigation/config/nav2_bunker.yaml")
    safety = read("src/agt_safety/scripts/tracked_safety_controller.py")
    guard_config = read("src/agt_chassis/config/bunker.yaml")
    chassis_launch = read("src/agt_chassis/launch/bunker.launch.py")

    assert '("cmd_vel", "/agt/navigation/cmd_vel_raw")' in nav_launch
    assert "cmd_vel_in_topic: /agt/navigation/cmd_vel_raw" in nav_config
    assert "cmd_vel_out_topic: /agt/navigation/cmd_vel" in nav_config
    assert 'Twist, "/agt/navigation/cmd_vel"' in safety
    assert 'Twist, "/agt/cmd_vel_manual"' in safety
    assert "input_topic: /agt/safety/cmd_vel" in guard_config
    assert "output_topic: /agt/chassis/cmd_vel" in guard_config
    assert '("/cmd_vel", "/agt/chassis/cmd_vel")' in chassis_launch


def test_bunker_driver_does_not_publish_duplicate_odom_tf_by_default():
    chassis_launch = read("src/agt_chassis/launch/bunker.launch.py")
    mapping_launch = read("src/agt_mapping/launch/fast_livo2_mapping.launch.py")
    assert 'DeclareLaunchArgument("publish_driver_odom_tf", default_value="false")' in chassis_launch
    assert '"common.publish_tf": False' in mapping_launch
