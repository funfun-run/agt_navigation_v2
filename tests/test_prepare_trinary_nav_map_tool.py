from pathlib import Path
import subprocess
import sys

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]


def test_prepare_trinary_nav_map_tool_generates_only_trinary_pixels(tmp_path):
    source = tmp_path / "observed.png"
    image = np.array(
        [
            [105, 105, 180],
            [120, 160, 200],
            [104, 106, 220],
        ],
        dtype=np.uint8,
    )
    Image.fromarray(image, mode="L").save(source)
    output = tmp_path / "nav.png"

    subprocess.run(
        [
            sys.executable,
            str(ROOT / "tools" / "map_tools" / "prepare_trinary_nav_map.py"),
            "--input",
            str(source),
            "--output",
            str(output),
            "--unknown-value",
            "105",
            "--unknown-margin",
            "2",
            "--occupied-threshold",
            "170",
        ],
        check=True,
    )

    result = np.asarray(Image.open(output).convert("L"), dtype=np.uint8)
    assert set(np.unique(result).tolist()) <= {0, 205, 254}


def test_point_topology_marks_points_occupied_and_enclosed_blank_free(tmp_path):
    source = tmp_path / "point_frame.png"
    image = np.full((15, 15), 105, dtype=np.uint8)
    image[3, 3:12] = 180
    image[11, 3:12] = 180
    image[3:12, 3] = 180
    image[3:12, 11] = 180
    image[3, 7] = 105  # A one-pixel rasterization gap must not leak the interior.
    Image.fromarray(image, mode="L").save(source)
    output = tmp_path / "nav.png"

    subprocess.run(
        [
            sys.executable,
            str(ROOT / "tools" / "map_tools" / "prepare_trinary_nav_map.py"),
            "--input",
            str(source),
            "--output",
            str(output),
            "--unknown-value",
            "105",
            "--unknown-margin",
            "0",
            "--classification",
            "point-topology",
            "--closure-size",
            "3",
        ],
        check=True,
    )

    result = np.asarray(Image.open(output).convert("L"), dtype=np.uint8)
    assert result[0, 0] == 205
    assert result[7, 7] == 254
    assert result[3, 3] == 0
