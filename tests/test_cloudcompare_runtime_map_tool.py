from pathlib import Path
import subprocess
import sys

import yaml
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]


def test_cloudcompare_runtime_map_tool_creates_qt5_ready_package(tmp_path):
    source_image = tmp_path / "source.png"
    image = Image.new("L", (4, 3), color=255)
    image.putpixel((1, 1), 0)
    image.save(source_image)

    runtime_root = tmp_path / "runtime_maps"
    subprocess.run(
        [
            sys.executable,
            str(ROOT / "tools" / "map_tools" / "create_cloudcompare_runtime_map.py"),
            "--source-image",
            str(source_image),
            "--map-id",
            "demo_map",
            "--resolution",
            "0.05",
            "--origin-x",
            "1.25",
            "--origin-y",
            "-3.5",
            "--runtime-root",
            str(runtime_root),
        ],
        check=True,
    )

    map_dir = runtime_root / "demo_map"
    assert (map_dir / "demo_map.png").exists()
    assert (map_dir / "demo_map_observed.png").exists()
    assert (map_dir / "demo_map.yaml").exists()
    assert (map_dir / "processing_record.yaml").exists()
    assert (map_dir / "semantic" / "coverage.yaml").exists()
    assert (map_dir / "semantic" / "README.md").exists()

    metadata = yaml.safe_load((map_dir / "demo_map.yaml").read_text(encoding="utf-8"))
    assert metadata["image"] == "demo_map.png"
    assert metadata["resolution"] == 0.05
    assert metadata["origin"] == [1.25, -3.5, 0.0]

    record = yaml.safe_load(
        (map_dir / "processing_record.yaml").read_text(encoding="utf-8")
    )
    assert record["status"]["stage"] == "qt5_annotation_baseline"
    assert record["rasterize_metadata"]["pending_action"] is not None
    assert record["nav_map"]["width_cells"] == 4
    assert record["nav_map"]["height_cells"] == 3
