#!/usr/bin/env python3
"""Convert a CloudCompare grayscale raster into a conservative trinary Nav2 map."""

from __future__ import annotations

import argparse
from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image, ImageFilter


UNKNOWN_PIXEL = 205
FREE_PIXEL = 254
OCCUPIED_PIXEL = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert a grayscale observed raster into a trinary Nav2 map draft. "
            "Pixels near the dominant background gray stay unknown, brighter "
            "structure becomes occupied, and the remaining observed area becomes free."
        )
    )
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--unknown-value", type=int, default=None)
    parser.add_argument("--unknown-margin", type=int, default=4)
    parser.add_argument("--occupied-threshold", type=int, default=168)
    parser.add_argument(
        "--classification",
        choices=("intensity", "point-topology"),
        default="intensity",
        help=(
            "intensity classifies by gray level; point-topology treats every pixel "
            "outside the background band as occupied, enclosed blank space as free, "
            "and edge-connected blank space as unknown"
        ),
    )
    parser.add_argument(
        "--closure-size",
        type=int,
        default=11,
        help="Odd morphology kernel used only to seal the boundary for topology detection",
    )
    return parser.parse_args()


def edge_connected_mask(passable: np.ndarray) -> np.ndarray:
    """Return passable pixels connected to an image edge using 8-connectivity."""
    height, width = passable.shape
    outside = np.zeros_like(passable, dtype=bool)
    queue: deque[tuple[int, int]] = deque()

    for x in range(width):
        queue.append((0, x))
        queue.append((height - 1, x))
    for y in range(1, height - 1):
        queue.append((y, 0))
        queue.append((y, width - 1))

    while queue:
        y, x = queue.popleft()
        if outside[y, x] or not passable[y, x]:
            continue
        outside[y, x] = True
        for next_y in range(max(0, y - 1), min(height, y + 2)):
            for next_x in range(max(0, x - 1), min(width, x + 2)):
                if not outside[next_y, next_x] and passable[next_y, next_x]:
                    queue.append((next_y, next_x))
    return outside


def classify_point_topology(
    image: np.ndarray,
    background_low: int,
    background_high: int,
    closure_size: int,
) -> np.ndarray:
    if closure_size < 1 or closure_size % 2 == 0:
        raise ValueError("--closure-size must be a positive odd integer")

    background = (image >= background_low) & (image <= background_high)
    point_mask = ~background

    # Closing is only a flood-fill barrier; output obstacles keep their source width.
    barrier_image = Image.fromarray(point_mask.astype(np.uint8) * 255, mode="L")
    if closure_size > 1:
        barrier_image = barrier_image.filter(ImageFilter.MaxFilter(closure_size))
        barrier_image = barrier_image.filter(ImageFilter.MinFilter(closure_size))
    barrier = np.asarray(barrier_image, dtype=np.uint8) > 0
    outside = edge_connected_mask(~barrier)

    result = np.full(image.shape, FREE_PIXEL, dtype=np.uint8)
    result[outside] = UNKNOWN_PIXEL
    result[point_mask] = OCCUPIED_PIXEL
    return result


def main() -> None:
    args = parse_args()
    if not args.input.is_file():
        raise FileNotFoundError(f"input image not found: {args.input}")

    image = np.asarray(Image.open(args.input).convert("L"), dtype=np.uint8)
    histogram = np.bincount(image.reshape(-1), minlength=256)
    dominant = int(histogram.argmax()) if args.unknown_value is None else args.unknown_value
    unknown_low = max(0, dominant - args.unknown_margin)
    unknown_high = min(255, dominant + args.unknown_margin)

    if args.classification == "point-topology":
        result = classify_point_topology(
            image, unknown_low, unknown_high, args.closure_size
        )
    else:
        result = np.full(image.shape, FREE_PIXEL, dtype=np.uint8)
        unknown_mask = (image >= unknown_low) & (image <= unknown_high)
        occupied_mask = image >= args.occupied_threshold
        result[unknown_mask] = UNKNOWN_PIXEL
        result[occupied_mask] = OCCUPIED_PIXEL

    args.output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(result, mode="L").save(args.output)

    print(f"Input              : {args.input}")
    print(f"Output             : {args.output}")
    print(f"Dominant gray      : {dominant}")
    print(f"Unknown band       : [{unknown_low}, {unknown_high}]")
    print(f"Classification     : {args.classification}")
    if args.classification == "point-topology":
        print(f"Boundary closure   : {args.closure_size} px")
    else:
        print(f"Occupied threshold : >= {args.occupied_threshold}")
    unique, counts = np.unique(result, return_counts=True)
    for pixel, count in zip(unique.tolist(), counts.tolist()):
        label = {
            OCCUPIED_PIXEL: "occupied",
            FREE_PIXEL: "free",
            UNKNOWN_PIXEL: "unknown",
        }.get(pixel, str(pixel))
        print(f"  {label:8s}: {count}")


if __name__ == "__main__":
    main()
