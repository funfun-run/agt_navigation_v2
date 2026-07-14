import json
from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from agt_ui_bridge.semantic_model import (  # noqa: E402
    SemanticFeature,
    SemanticMap,
)
from agt_ui_bridge.semantic_scene import SemanticScene  # noqa: E402


VALID_MAP = (
    PACKAGE_ROOT.parents[1]
    / "docs/interfaces/examples/semantic_map/semantic/semantic_map.geojson"
)


def _scene():
    document = json.loads(VALID_MAP.read_text(encoding="utf-8"))
    return SemanticScene(SemanticMap.from_geojson(document))


def test_add_remove_undo_and_redo_preserve_feature_geometry():
    scene = _scene()
    original_count = len(scene.features)
    feature = SemanticFeature(
        id="row_02",
        feature_type="row_centerline",
        name="Row 2",
        geometry_type="LineString",
        coordinates=[[1.0, 2.0], [7.0, 2.0]],
    )

    scene.add(feature)
    assert scene.dirty
    assert len(scene.features) == original_count + 1
    assert scene.undo()
    assert scene.get("row_02") is None
    assert scene.redo()
    assert scene.get("row_02").coordinates == feature.coordinates

    scene.remove("row_02")
    assert scene.get("row_02") is None
    assert scene.undo()
    assert scene.get("row_02") is not None


def test_duplicate_ids_and_missing_replacements_are_rejected():
    scene = _scene()
    existing = scene.features[0]
    with pytest.raises(ValueError, match="duplicate feature id"):
        scene.add(existing)

    missing = SemanticFeature(
        id="missing_01",
        feature_type="row_centerline",
        name="Missing",
        geometry_type="LineString",
        coordinates=[[0.0, 0.0], [1.0, 0.0]],
    )
    with pytest.raises(KeyError):
        scene.replace(missing)


def test_mark_saved_only_clears_dirty_state():
    scene = _scene()
    scene.remove("row_01")
    assert scene.can_undo
    scene.mark_saved()
    assert not scene.dirty
    assert scene.can_undo


def test_renaming_a_feature_is_one_undoable_operation():
    scene = _scene()
    original = scene.get("row_01")
    renamed = SemanticFeature.from_geojson(original.to_geojson())
    renamed.id = "row_primary"
    renamed.name = "Primary row"

    scene.replace_by_id("row_01", renamed)
    assert scene.get("row_01") is None
    assert scene.get("row_primary").name == "Primary row"
    assert scene.undo()
    assert scene.get("row_01") is not None
