import math
from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from agt_ui_bridge.map_transform import MapGeometry, MapTransform  # noqa: E402


EXAMPLE_MAP = (
    PACKAGE_ROOT.parents[1]
    / "docs/interfaces/examples/semantic_map/example_map.yaml"
)


def test_grid_and_image_coordinates_apply_the_pgm_y_flip():
    transform = MapTransform(MapGeometry(0.5, width=4, height=3))

    assert transform.grid_to_image_pixel(0, 0) == (0.5, 2.5)
    assert transform.grid_to_image_pixel(3, 2) == (3.5, 0.5)
    assert transform.image_pixel_to_grid(0.5, 2.5) == (0, 0)
    assert transform.image_pixel_to_grid(3.5, 0.5) == (3, 2)


def test_grid_world_round_trip_handles_resolution_origin_and_yaw():
    geometry = MapGeometry(
        resolution=1.0,
        width=5,
        height=4,
        origin_x=10.0,
        origin_y=20.0,
        origin_yaw=math.pi / 2.0,
    )
    transform = MapTransform(geometry)

    world = transform.grid_to_world(0, 0)
    assert world == pytest.approx((9.5, 20.5))
    assert transform.world_to_grid(*world) == (0, 0)

    for grid_x in range(geometry.width):
        for grid_y in range(geometry.height):
            world_x, world_y = transform.grid_to_world(grid_x, grid_y)
            assert transform.world_to_grid(world_x, world_y) == (grid_x, grid_y)


def test_scene_world_round_trip_supports_offset_and_scaling():
    geometry = MapGeometry(
        resolution=0.2,
        width=20,
        height=10,
        origin_x=-3.0,
        origin_y=1.5,
        origin_yaw=0.3,
    )
    transform = MapTransform(
        geometry, scene_origin=(25.0, -10.0), scene_units_per_cell=8.0
    )
    world = transform.grid_to_world(7, 4)
    scene = transform.world_to_scene(*world)
    recovered = transform.scene_to_world(*scene)

    assert recovered == pytest.approx(world, abs=0.5 * geometry.resolution)


def test_boundaries_reject_points_outside_the_map():
    transform = MapTransform(MapGeometry(1.0, width=2, height=2))

    with pytest.raises(ValueError, match="grid cell outside"):
        transform.grid_to_world(2, 0)
    with pytest.raises(ValueError, match="image point outside"):
        transform.image_pixel_to_grid(-0.1, 0.0)
    with pytest.raises(ValueError, match="world point outside"):
        transform.world_to_grid(2.0, 0.0)


def test_continuous_world_transform_accepts_map_outline_vertices():
    transform = MapTransform(MapGeometry(1.0, width=2, height=2))

    assert transform.image_to_world(0.0, 2.0) == pytest.approx((0.0, 0.0))
    assert transform.world_to_image(2.0, 2.0) == pytest.approx((2.0, 0.0))
    assert transform.world_to_scene(0.0, 0.0) == pytest.approx((0.0, 2.0))


def test_geometry_loads_dimensions_and_nonzero_origin_from_nav2_yaml():
    geometry = MapGeometry.from_nav2_yaml(EXAMPLE_MAP)

    assert geometry.width == 10
    assert geometry.height == 10
    assert geometry.resolution == 1.0
    assert geometry.origin_x == -1.0
    assert geometry.origin_y == -2.0
