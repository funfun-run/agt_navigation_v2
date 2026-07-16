#!/usr/bin/env python3
"""
根据 CloudCompare Rasterize 的 Min corner（左下角边界栅格中心）
自动生成 Nav2 PNG/YAML 地图元数据。

依赖：
    sudo apt install python3-pil python3-yaml

示例：
    python3 generate_nav2_map_yaml.py \
      --image greenhouse_navigation.png \
      --resolution 0.05 \
      --min-center-x -2.45 \
      --min-center-y -30.45 \
      --output greenhouse_navigation.yaml
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from PIL import Image
import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a Nav2 map YAML from CloudCompare Rasterize metadata."
    )
    parser.add_argument("--image", required=True, type=Path, help="Input PNG/PGM map image")
    parser.add_argument("--resolution", required=True, type=float, help="Map resolution, m/cell")
    parser.add_argument(
        "--min-center-x",
        required=True,
        type=float,
        help="CloudCompare grid minimum X cell-center coordinate",
    )
    parser.add_argument(
        "--min-center-y",
        required=True,
        type=float,
        help="CloudCompare grid minimum Y cell-center coordinate",
    )
    parser.add_argument("--output", required=True, type=Path, help="Output YAML path")
    parser.add_argument("--mode", default="trinary", choices=("trinary", "scale", "raw"))
    parser.add_argument("--negate", default=0, type=int, choices=(0, 1))
    parser.add_argument("--occupied-thresh", default=0.65, type=float)
    parser.add_argument("--free-thresh", default=0.25, type=float)
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if not args.image.is_file():
        raise FileNotFoundError(f"Map image not found: {args.image}")
    if args.resolution <= 0:
        raise ValueError("resolution must be greater than zero")
    if not (0.0 <= args.free_thresh < args.occupied_thresh <= 1.0):
        raise ValueError(
            "Thresholds must satisfy 0 <= free_thresh < occupied_thresh <= 1"
        )

    with Image.open(args.image) as image:
        width, height = image.size
        image_mode = image.mode
        bands = image.getbands()

    origin_x = args.min_center_x - args.resolution / 2.0
    origin_y = args.min_center_y - args.resolution / 2.0

    try:
        image_ref = str(args.image.resolve().relative_to(args.output.parent.resolve()))
    except ValueError:
        image_ref = str(args.image.resolve())

    metadata: dict[str, Any] = {
        "image": image_ref,
        "mode": args.mode,
        "resolution": float(args.resolution),
        "origin": [round(origin_x, 9), round(origin_y, 9), 0.0],
        "negate": int(args.negate),
        "occupied_thresh": float(args.occupied_thresh),
        "free_thresh": float(args.free_thresh),
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as file:
        yaml.safe_dump(
            metadata,
            file,
            allow_unicode=True,
            sort_keys=False,
            default_flow_style=False,
        )

    print("Nav2 map YAML generated")
    print(f"  image             : {args.image}")
    print(f"  image mode/bands  : {image_mode} / {bands}")
    print(f"  image size        : {width} x {height} cells")
    print(f"  resolution        : {args.resolution:.9f} m/cell")
    print(
        f"  physical size     : "
        f"{width * args.resolution:.3f} x {height * args.resolution:.3f} m"
    )
    print(
        f"  min cell center   : "
        f"({args.min_center_x:.9f}, {args.min_center_y:.9f})"
    )
    print(f"  YAML origin       : ({origin_x:.9f}, {origin_y:.9f}, 0.0)")
    print(f"  output            : {args.output}")

    if "A" in bands:
        print(
            "WARNING: image contains an alpha channel. "
            "Flatten/export it as a standard grayscale or RGB PNG before final testing."
        )
    if image_mode not in ("L", "1", "RGB", "RGBA"):
        print(
            f"WARNING: unusual image mode '{image_mode}'. "
            "Verify that the map server interprets the image as expected."
        )


if __name__ == "__main__":
    main()
