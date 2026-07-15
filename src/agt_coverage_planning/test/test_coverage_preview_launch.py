from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch/coverage_preview.launch.py"
RVIZ_FILE = PACKAGE_ROOT / "rviz/coverage_preview.rviz"


def test_preview_launch_is_offline_and_fail_closed():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    compile(source, str(LAUNCH_FILE), "exec")
    assert '"plan_on_start": "true"' in source
    assert '"execution_enabled": "false"' in source
    assert 'package="nav2_map_server"' in source
    assert 'package="rviz2"' in source
    assert "nav2_controller" not in source
    assert "agt_chassis" not in source
    assert "agt_safety" not in source


def test_preview_rviz_contains_map_and_coverage_layers():
    config = yaml.safe_load(RVIZ_FILE.read_text(encoding="utf-8"))
    displays = config["Visualization Manager"]["Displays"]
    topics = {
        display.get("Topic", {}).get("Value")
        for display in displays
        if isinstance(display.get("Topic"), dict)
    }

    assert config["Visualization Manager"]["Global Options"]["Fixed Frame"] == "map"
    assert "/agt/map/global_occupancy" in topics
    assert "/agt/map/keepout_mask" in topics
    assert "/agt/coverage/path_preview" in topics
    assert "/agt/coverage/path_reconstructed" in topics
    assert "/agt/coverage/swaths" in topics
