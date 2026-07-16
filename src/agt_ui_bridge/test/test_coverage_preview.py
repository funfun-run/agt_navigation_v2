from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from agt_ui_bridge.coverage_preview import (
    CoveragePreviewError,
    derive_inter_row_aisles,
)
from agt_ui_bridge.semantic_model import SemanticFeature, SemanticMap


def _line(identifier, y, reverse=False):
    points = [[0.0, y], [10.0, y]]
    if reverse:
        points.reverse()
    return SemanticFeature(
        id=identifier,
        feature_type="row_centerline",
        name=identifier,
        geometry_type="LineString",
        coordinates=points,
    )


def test_inter_row_preview_generates_midlines_between_crop_rows():
    semantic_map = SemanticMap(
        map_id="test",
        features=[
            _line("row_01", 4.0),
            _line("row_02", 2.0, reverse=True),
            _line("row_03", 0.0),
            SemanticFeature(
                id="direction_01",
                feature_type="work_direction",
                name="direction",
                geometry_type="LineString",
                coordinates=[[0.0, 0.0], [1.0, 0.0]],
            ),
        ],
    )

    output = derive_inter_row_aisles(semantic_map)
    aisles = [item for item in output.features if item.feature_type == "row_centerline"]

    assert len(aisles) == 2
    assert aisles[0].coordinates == [[0.0, 3.0], [10.0, 3.0]]
    assert aisles[1].coordinates == [[0.0, 1.0], [10.0, 1.0]]
    assert len([item for item in semantic_map.features if item.feature_type == "row_centerline"]) == 3


def test_inter_row_preview_requires_two_rows():
    semantic_map = SemanticMap(map_id="test", features=[_line("row_01", 0.0)])
    with pytest.raises(CoveragePreviewError):
        derive_inter_row_aisles(semantic_map)
