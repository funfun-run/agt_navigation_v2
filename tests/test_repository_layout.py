from pathlib import Path


def test_core_directories_exist():
    root = Path(__file__).resolve().parents[1]
    required = [
        "docs",
        "profiles",
        "runtime",
        "src",
        "tests",
        "tools",
    ]
    for item in required:
        assert (root / item).exists(), f"missing {item}"


def test_colcon_can_discover_nested_ros_packages():
    root = Path(__file__).resolve().parents[1]
    assert not (root / "src" / "CMakeLists.txt").exists(), (
        "a top-level src/CMakeLists.txt prevents colcon from discovering packages"
    )
    assert (root / "src" / "agt_description" / "package.xml").exists()


def test_readme_points_to_the_single_extrinsics_file():
    root = Path(__file__).resolve().parents[1]
    readme = (root / "README.md").read_text(encoding="utf-8")
    assert "src/agt_description/config/mk_mini_mid360.yaml" in readme
