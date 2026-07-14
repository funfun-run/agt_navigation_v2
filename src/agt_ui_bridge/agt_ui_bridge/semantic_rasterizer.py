"""Rasterize validated semantic polygons onto an existing map geometry."""

from dataclasses import dataclass

from PIL import Image, ImageChops, ImageDraw, ImageOps

from .map_transform import MapTransform


@dataclass(frozen=True)
class RasterizedMask:
    width: int
    height: int
    data: tuple
    occupied_count: int


def rasterize_keepout_mask(
    semantic_map,
    map_geometry,
    outside_field_is_keepout=True,
    free_value=0,
    occupied_value=100,
):
    """Return bottom-up OccupancyGrid data without modifying the base map."""
    _validate_values(free_value, occupied_value)
    transform = MapTransform(map_geometry)
    size = (map_geometry.width, map_geometry.height)
    field_union = Image.new("L", size, 0)
    keepout_union = Image.new("L", size, 0)

    for feature in semantic_map.features:
        if not feature.enabled or feature.geometry_type != "Polygon":
            continue
        if feature.feature_type == "field_boundary":
            field_union = ImageChops.lighter(
                field_union, _polygon_image(feature.coordinates, transform, size)
            )
        elif feature.feature_type in {"exclusion_zone", "keepout_zone"}:
            keepout_union = ImageChops.lighter(
                keepout_union,
                _polygon_image(feature.coordinates, transform, size),
            )

    blocked = (
        ImageOps.invert(field_union)
        if outside_field_is_keepout
        else Image.new("L", size, 0)
    )
    blocked = ImageChops.lighter(blocked, keepout_union)
    pixels = blocked.load()
    data = []
    for grid_y in range(map_geometry.height):
        image_y = map_geometry.height - 1 - grid_y
        data.extend(
            occupied_value if pixels[grid_x, image_y] else free_value
            for grid_x in range(map_geometry.width)
        )
    return RasterizedMask(
        width=map_geometry.width,
        height=map_geometry.height,
        data=tuple(data),
        occupied_count=sum(value == occupied_value for value in data),
    )


def _polygon_image(coordinates, transform, size):
    image = Image.new("L", size, 0)
    draw = ImageDraw.Draw(image)
    outer = [_image_point(point, transform) for point in coordinates[0]]
    draw.polygon(outer, fill=255)
    for hole in coordinates[1:]:
        draw.polygon(
            [_image_point(point, transform) for point in hole], fill=0
        )
    return image


def _image_point(point, transform):
    image_x, image_y = transform.world_to_image_unbounded(point[0], point[1])
    return image_x - 0.5, image_y - 0.5


def _validate_values(free_value, occupied_value):
    for name, value in (
        ("free_value", free_value),
        ("occupied_value", occupied_value),
    ):
        if not isinstance(value, int) or isinstance(value, bool):
            raise ValueError(f"{name} must be an integer")
        if not 0 <= value <= 100:
            raise ValueError(f"{name} must be between 0 and 100")
    if free_value == occupied_value:
        raise ValueError("free and occupied mask values must differ")
