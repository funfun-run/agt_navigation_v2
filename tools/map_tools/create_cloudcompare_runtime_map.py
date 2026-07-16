#!/usr/bin/env python3
"""Create a runtime Nav2 map package from a CloudCompare raster image."""

from __future__ import annotations

import argparse
import hashlib
import shutil
from pathlib import Path
from typing import Any

from PIL import Image
import yaml


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RUNTIME_ROOT = ROOT / "runtime" / "maps"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Create a runtime map directory from a CloudCompare PNG so it can be "
            "loaded by Nav2 and annotated with the Qt5 tools."
        )
    )
    parser.add_argument("--source-image", required=True, type=Path)
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--origin-x", type=float, default=0.0)
    parser.add_argument("--origin-y", type=float, default=0.0)
    parser.add_argument("--origin-yaw", type=float, default=0.0)
    parser.add_argument(
        "--runtime-root",
        type=Path,
        default=DEFAULT_RUNTIME_ROOT,
        help="Destination root for runtime map packages",
    )
    parser.add_argument(
        "--source-pcd",
        type=Path,
        default=None,
        help="Optional aligned full cloud used to derive the raster",
    )
    parser.add_argument(
        "--min-center-x",
        type=float,
        default=None,
        help="Optional CloudCompare Rasterize minimum X cell-center coordinate",
    )
    parser.add_argument(
        "--min-center-y",
        type=float,
        default=None,
        help="Optional CloudCompare Rasterize minimum Y cell-center coordinate",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace an existing runtime map directory with the same map_id",
    )
    return parser.parse_args()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def relative_to_root(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path.resolve())


def validate_args(args: argparse.Namespace) -> None:
    if not args.source_image.is_file():
        raise FileNotFoundError(f"source image not found: {args.source_image}")
    if args.source_pcd is not None and not args.source_pcd.is_file():
        raise FileNotFoundError(f"source PCD not found: {args.source_pcd}")
    if args.resolution <= 0.0:
        raise ValueError("resolution must be greater than zero")
    if not args.map_id.strip():
        raise ValueError("map_id must not be empty")


def write_yaml(path: Path, content: dict[str, Any]) -> None:
    path.write_text(
        yaml.safe_dump(content, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def main() -> None:
    args = parse_args()
    validate_args(args)

    source_image = args.source_image.resolve()
    runtime_root = args.runtime_root.resolve()
    map_dir = runtime_root / args.map_id
    semantic_dir = map_dir / "semantic"
    map_image_name = f"{args.map_id}.png"
    map_yaml_name = f"{args.map_id}.yaml"
    observed_name = f"{args.map_id}_observed.png"

    if map_dir.exists():
        if not args.overwrite:
            raise FileExistsError(
                f"destination already exists: {map_dir} (use --overwrite to replace)"
            )
        shutil.rmtree(map_dir)

    semantic_dir.mkdir(parents=True, exist_ok=True)

    with Image.open(source_image) as image:
        grayscale = image.convert("L")
        width, height = grayscale.size
        grayscale.save(map_dir / map_image_name)
        grayscale.save(map_dir / observed_name)

    origin = [float(args.origin_x), float(args.origin_y), float(args.origin_yaw)]
    nav_yaml = {
        "image": map_image_name,
        "mode": "trinary",
        "resolution": float(args.resolution),
        "origin": origin,
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    write_yaml(map_dir / map_yaml_name, nav_yaml)

    processing_record = {
        "map_id": args.map_id,
        "status": {
            "stage": "qt5_annotation_baseline",
            "description": (
                "CloudCompare observed raster packaged for Qt5 editing. "
                "This is not yet the final validated navigation map."
            ),
        },
        "sources": {
            "cloudcompare_png": relative_to_root(source_image),
            "cloudcompare_png_sha256": sha256_file(source_image),
            "aligned_full_cloud": (
                relative_to_root(args.source_pcd) if args.source_pcd is not None else None
            ),
        },
        "nav_map": {
            "yaml": map_yaml_name,
            "image": map_image_name,
            "observed_reference_image": observed_name,
            "resolution": float(args.resolution),
            "origin": origin,
            "width_cells": int(width),
            "height_cells": int(height),
        },
        "rasterize_metadata": {
            "min_center_x": args.min_center_x,
            "min_center_y": args.min_center_y,
            "origin_from_min_center": (
                [
                    float(args.min_center_x - args.resolution / 2.0),
                    float(args.min_center_y - args.resolution / 2.0),
                    0.0,
                ]
                if args.min_center_x is not None and args.min_center_y is not None
                else None
            ),
            "pending_action": (
                None
                if args.min_center_x is not None and args.min_center_y is not None
                else "Record CloudCompare Rasterize min center X/Y before using this map for closed-loop localization/navigation."
            ),
        },
        "qt5_annotation": {
            "base_map_read_only": observed_name,
            "editable_nav_map": map_image_name,
            "semantic_dir": "semantic",
        },
    }
    write_yaml(map_dir / "processing_record.yaml", processing_record)

    coverage = {
        "schema_version": "1.0.0",
        "map_id": args.map_id,
        "annotation_status": "draft",
        "notes": "Populate after Qt5 semantic annotation.",
    }
    write_yaml(semantic_dir / "coverage.yaml", coverage)

    (semantic_dir / "README.md").write_text(
        "\n".join(
            [
                f"# {args.map_id} semantic",
                "",
                "该目录预留给 Qt5 语义标注结果。",
                "",
                "- `semantic_map.geojson`：保存 field boundary、exclusion zone、rows、entry pose。",
                "- `coverage.yaml`：保存覆盖规划参数和任务级元数据。",
                "",
                "当前只完成基础导航底图封装，语义文件仍待人工标注。",
                "",
            ]
        ),
        encoding="utf-8",
    )

    print(f"Created runtime map package: {map_dir}")
    print(f"  image      : {map_dir / map_image_name}")
    print(f"  yaml       : {map_dir / map_yaml_name}")
    print(f"  semantic   : {semantic_dir}")
    if processing_record["rasterize_metadata"]["pending_action"]:
        print("WARNING: CloudCompare min center metadata is still missing.")


if __name__ == "__main__":
    main()
