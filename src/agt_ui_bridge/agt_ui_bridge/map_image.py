"""Load and save Nav2 map images without ROS dependencies."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image, ImageOps
import yaml


UNKNOWN_PIXEL = 205
FREE_PIXEL = 254
OCCUPIED_PIXEL = 0


@dataclass(frozen=True)
class LoadedNav2MapImage:
    yaml_path: Path
    image_path: Path
    image: np.ndarray
    metadata: dict


def load_nav2_map_image(yaml_path: str | Path) -> LoadedNav2MapImage:
    yaml_path = Path(yaml_path).expanduser().resolve()
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image = Image.open(image_path).convert("L")
    if bool(metadata.get("negate", 0)):
        image = ImageOps.invert(image)
    return LoadedNav2MapImage(
        yaml_path=yaml_path,
        image_path=image_path.resolve(),
        image=np.asarray(image, dtype=np.uint8).copy(),
        metadata=metadata,
    )


def save_nav2_map_image(
    yaml_path: str | Path, metadata: dict, image: np.ndarray
) -> Path:
    yaml_path = Path(yaml_path).expanduser().resolve()
    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(image.astype(np.uint8), mode="L").save(image_path)
    yaml_path.write_text(
        yaml.safe_dump(metadata, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return image_path


def nav2_pixel_to_grid_value(pixel: int) -> int:
    if pixel == OCCUPIED_PIXEL:
        return 100
    if pixel == FREE_PIXEL:
        return 0
    return -1


def grid_value_to_nav2_pixel(value: int) -> int:
    if value >= 100:
        return OCCUPIED_PIXEL
    if value == 0:
        return FREE_PIXEL
    return UNKNOWN_PIXEL
