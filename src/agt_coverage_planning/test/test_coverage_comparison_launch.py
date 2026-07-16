from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE_ROOT / "launch/coverage_comparison.launch.py"


def test_comparison_launch_is_offline_and_execution_free():
    text = LAUNCH.read_text(encoding="utf-8")

    assert 'executable="coverage_variant_comparator.py"' in text
    assert 'executable="opennav_coverage"' in text
    assert 'executable="rviz2"' in text
    assert "agt_chassis" not in text
    assert "agt_safety" not in text
    assert "coverage_task_server.py" not in text
    assert "navigation.launch.py" not in text


def test_comparison_launch_requires_canonical_inputs_and_report_is_optional():
    text = LAUNCH.read_text(encoding="utf-8")

    for argument in ("map", "semantic_map", "platform_profile"):
        assert f'DeclareLaunchArgument("{argument}")' in text
    assert 'DeclareLaunchArgument("report_path", default_value="")' in text
    assert "coverage_variants.yaml" in text
