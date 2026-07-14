import math
from pathlib import Path
import sys

import pytest
from shapely.geometry import Point, Polygon


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from agt_ui_bridge.map_transform import MapGeometry, MapTransform  # noqa: E402
from agt_ui_bridge.semantic_model import (  # noqa: E402
    SemanticFeature,
    SemanticMap,
)
from agt_ui_bridge.semantic_rasterizer import (  # noqa: E402
    rasterize_keepout_mask,
)


def _polygon(feature_id, feature_type, ring, enabled=True):
    return SemanticFeature(
        id=feature_id,
        feature_type=feature_type,
        name=feature_id,
        geometry_type="Polygon",
        coordinates=[[list(point) for point in ring]],
        enabled=enabled,
    )


def _semantic_map(transform=None, disabled_keepout=False):
    rings = {
        "field": [(1, 1), (9, 1), (9, 9), (1, 9), (1, 1)],
        "exclusion": [(3, 3), (5, 3), (5, 5), (3, 5), (3, 3)],
        "keepout": [(6, 6), (7, 6), (7, 7), (6, 7), (6, 6)],
    }
    if transform is not None:
        rings = {
            name: [transform._local_to_world(x, y) for x, y in ring]
            for name, ring in rings.items()
        }
    return SemanticMap(
        map_id="synthetic",
        features=[
            _polygon("field_01", "field_boundary", rings["field"]),
            _polygon("exclusion_01", "exclusion_zone", rings["exclusion"]),
            _polygon(
                "keepout_01",
                "keepout_zone",
                rings["keepout"],
                enabled=not disabled_keepout,
            ),
        ],
    )


def _cell(mask, x, y):
    return mask.data[y * mask.width + x]


def test_field_outside_exclusion_and_keepout_are_rasterized():
    geometry = MapGeometry(resolution=1.0, width=10, height=10)
    mask = rasterize_keepout_mask(_semantic_map(), geometry)

    assert len(mask.data) == 100
    assert _cell(mask, 0, 0) == 100
    assert _cell(mask, 2, 2) == 0
    assert _cell(mask, 4, 4) == 100
    assert _cell(mask, 6, 6) == 100
    assert 0 < mask.occupied_count < 100


def test_field_outside_policy_and_disabled_zone_are_respected():
    geometry = MapGeometry(resolution=1.0, width=10, height=10)
    mask = rasterize_keepout_mask(
        _semantic_map(disabled_keepout=True),
        geometry,
        outside_field_is_keepout=False,
    )

    assert _cell(mask, 0, 0) == 0
    assert _cell(mask, 4, 4) == 100
    assert _cell(mask, 6, 6) == 0


def test_origin_yaw_produces_the_same_grid_mask():
    baseline_geometry = MapGeometry(resolution=1.0, width=10, height=10)
    baseline = rasterize_keepout_mask(_semantic_map(), baseline_geometry)
    rotated_geometry = MapGeometry(
        resolution=1.0,
        width=10,
        height=10,
        origin_x=12.0,
        origin_y=-4.0,
        origin_yaw=0.63,
    )
    transform = MapTransform(rotated_geometry)
    rotated = rasterize_keepout_mask(
        _semantic_map(transform=transform), rotated_geometry
    )

    assert rotated.data == baseline.data


def test_polygon_boundary_error_is_at_most_one_grid_cell():
    geometry = MapGeometry(resolution=0.5, width=20, height=20)
    semantic_map = _semantic_map()
    mask = rasterize_keepout_mask(semantic_map, geometry)
    transform = MapTransform(geometry)
    field = Polygon(semantic_map.features[0].coordinates[0])
    zones = [
        Polygon(feature.coordinates[0]) for feature in semantic_map.features[1:]
    ]
    boundaries = [field.boundary] + [zone.boundary for zone in zones]

    for grid_y in range(geometry.height):
        for grid_x in range(geometry.width):
            point = Point(transform.grid_to_world(grid_x, grid_y))
            expected = not field.covers(point) or any(
                zone.covers(point) for zone in zones
            )
            actual = _cell(mask, grid_x, grid_y) == 100
            if actual != expected:
                distance = min(point.distance(boundary) for boundary in boundaries)
                assert distance <= geometry.resolution * math.sqrt(2.0)


@pytest.mark.parametrize(
    "free_value,occupied_value",
    [(-1, 100), (0, 101), (50, 50), (0.0, 100)],
)
def test_invalid_mask_values_are_rejected(free_value, occupied_value):
    geometry = MapGeometry(resolution=1.0, width=10, height=10)
    with pytest.raises(ValueError):
        rasterize_keepout_mask(
            _semantic_map(),
            geometry,
            free_value=free_value,
            occupied_value=occupied_value,
        )
