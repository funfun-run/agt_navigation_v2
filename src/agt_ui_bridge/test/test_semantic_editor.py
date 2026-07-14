import importlib.util
import os
from pathlib import Path
import shutil
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtCore import QPointF, Qt  # noqa: E402
from PyQt5.QtWidgets import QApplication, QMessageBox  # noqa: E402
import pytest  # noqa: E402


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

SCRIPT = PACKAGE_ROOT / "scripts/semantic_editor_qt5.py"
SPEC = importlib.util.spec_from_file_location("semantic_editor_qt5", SCRIPT)
EDITOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EDITOR)

from agt_ui_bridge.semantic_model import SemanticFeature  # noqa: E402
from agt_ui_bridge.semantic_validation import validate_task  # noqa: E402


EXAMPLE_ROOT = REPOSITORY_ROOT / "docs/interfaces/examples/semantic_map"
PROFILE = REPOSITORY_ROOT / "profiles/platforms/bunker.yaml"


@pytest.fixture(scope="module")
def application():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def editor_files(tmp_path):
    shutil.copy2(EXAMPLE_ROOT / "example_map.yaml", tmp_path / "test_map.yaml")
    shutil.copy2(EXAMPLE_ROOT / "example_map.pgm", tmp_path / "example_map.pgm")
    return tmp_path / "test_map.yaml"


@pytest.fixture
def window(application, editor_files):
    editor = EDITOR.SemanticEditorWindow(
        platform_profile_path=PROFILE,
        map_path=editor_files,
    )
    yield editor
    if editor.model_scene is not None:
        editor.model_scene.mark_saved()
    editor.close()
    application.processEvents()


def _required_features():
    return [
        SemanticFeature(
            id="field_01",
            feature_type="field_boundary",
            name="Field",
            geometry_type="Polygon",
            coordinates=[
                [[0.0, 0.0], [8.0, 0.0], [8.0, 6.0], [0.0, 6.0], [0.0, 0.0]]
            ],
        ),
        SemanticFeature(
            id="exclusion_01",
            feature_type="exclusion_zone",
            name="Obstacle",
            geometry_type="Polygon",
            coordinates=[
                [[3.0, 2.0], [4.0, 2.0], [4.0, 3.0], [3.0, 3.0], [3.0, 2.0]]
            ],
        ),
        SemanticFeature(
            id="entry_01",
            feature_type="entry_pose",
            name="Entry",
            geometry_type="Point",
            coordinates=[1.0, 1.0],
            properties={"yaw": 0.0},
        ),
        SemanticFeature(
            id="direction_01",
            feature_type="work_direction",
            name="Direction",
            geometry_type="LineString",
            coordinates=[[1.0, 1.0], [7.0, 1.0]],
        ),
    ]


def _populate_required_features(window):
    for feature in _required_features():
        window.add_feature(feature)


def _scene_points(window, world_points):
    return [QPointF(*window.transformer.world_to_scene(*point)) for point in world_points]


def test_base_map_is_read_only_and_vehicle_width_comes_from_profile(window):
    assert window.transformer.geometry.width == 10
    assert window.transformer.geometry.height == 10
    assert window.coverage.robot_width == pytest.approx(0.938)
    assert window.navigation_footprint == [
        [0.5915, 0.469],
        [0.5915, -0.469],
        [-0.5915, -0.469],
        [-0.5915, 0.469],
    ]
    assert window.map_pixmap is not None
    assert window.model_scene.features == []


def test_features_and_layers_are_rendered_independently(window):
    _populate_required_features(window)
    assert set(window._feature_items) == {
        "field_01",
        "exclusion_01",
        "entry_01",
        "direction_01",
    }
    assert window.object_tree.topLevelItemCount() == 4

    window.selected_feature_id = "exclusion_01"
    window.refresh_scene()
    assert any(
        isinstance(item, EDITOR.VertexHandle)
        for item in window.graphics_scene.items()
    )
    window._layer_checks["exclusion_zone"].setChecked(False)
    assert "exclusion_01" not in window._feature_items
    assert "field_01" in window._feature_items
    assert not any(
        isinstance(item, EDITOR.VertexHandle)
        for item in window.graphics_scene.items()
    )
    window._layer_checks["exclusion_zone"].setChecked(True)


def test_scene_clicks_create_required_map_frame_geometry(window):
    assert window.finish_feature_from_scene(
        "field_boundary",
        _scene_points(window, [[0.0, 0.0], [8.0, 0.0], [8.0, 6.0], [0.0, 6.0]]),
        feature_id="field_01",
        name="Field",
    )
    assert window.finish_feature_from_scene(
        "exclusion_zone",
        _scene_points(window, [[3.0, 2.0], [4.0, 2.0], [4.0, 3.0], [3.0, 3.0]]),
        feature_id="exclusion_01",
        name="Obstacle",
    )
    assert window.finish_feature_from_scene(
        "entry_pose",
        _scene_points(window, [[0.5, 0.5], [1.5, 0.5]]),
        feature_id="entry_01",
        name="Entry",
    )
    assert window.finish_feature_from_scene(
        "work_direction",
        _scene_points(window, [[1.0, 1.0], [7.0, 1.0]]),
        feature_id="direction_01",
        name="Direction",
    )

    field = window.model_scene.get("field_01")
    assert field.frame_id == "map"
    assert field.coordinates[0][0] == field.coordinates[0][-1]
    assert window.model_scene.get("entry_01").properties["yaw"] == pytest.approx(0.0)
    assert validate_task(window.model_scene.semantic_map, window.coverage).valid


def test_vertex_drag_updates_closed_polygon_and_is_undoable(window):
    _populate_required_features(window)
    window.selected_feature_id = "field_01"
    scene_x, scene_y = window.transformer.world_to_scene(0.5, 0.5)

    window._vertex_moved("field_01", 0, QPointF(scene_x, scene_y))
    moved = window.model_scene.get("field_01")
    assert moved.coordinates[0][0] == pytest.approx([0.5, 0.5])
    assert moved.coordinates[0][-1] == pytest.approx([0.5, 0.5])

    window.undo()
    restored = window.model_scene.get("field_01")
    assert restored.coordinates[0][0] == pytest.approx([0.0, 0.0])
    assert restored.coordinates[0][-1] == pytest.approx([0.0, 0.0])


def test_valid_task_saves_reloads_and_does_not_modify_base_map(window, tmp_path):
    _populate_required_features(window)
    original_map = window.map_path.read_bytes()
    semantic_path = tmp_path / "saved" / "semantic_map.geojson"

    assert window._save_to(semantic_path)
    assert semantic_path.is_file()
    assert semantic_path.with_name("coverage.yaml").is_file()
    assert window.map_path.read_bytes() == original_map

    reloaded = EDITOR.SemanticEditorWindow(
        platform_profile_path=PROFILE,
        semantic_path=semantic_path,
    )
    try:
        assert not reloaded.read_only
        assert [
            feature.to_geojson() for feature in reloaded.model_scene.features
        ] == [feature.to_geojson() for feature in window.model_scene.features]
    finally:
        reloaded.model_scene.mark_saved()
        reloaded.close()


def test_invalid_task_is_blocked_before_any_file_is_written(
    window, tmp_path, monkeypatch
):
    messages = []
    monkeypatch.setattr(
        QMessageBox,
        "warning",
        lambda *args: messages.append(args[2]) or QMessageBox.Ok,
    )
    destination = tmp_path / "invalid" / "semantic_map.geojson"

    assert not window._save_to(destination)
    assert not destination.exists()
    assert messages
    assert "field_boundary" in messages[0]


def test_spatial_error_is_highlighted_with_code_and_object_id(window):
    _populate_required_features(window)
    exclusion = window.model_scene.get("exclusion_01")
    exclusion.coordinates = [
        [[7.5, 2.0], [8.5, 2.0], [8.5, 3.0], [7.5, 3.0], [7.5, 2.0]]
    ]
    window.model_scene.replace(exclusion)
    window.refresh_scene()

    assert window._feature_items["exclusion_01"].pen().style() == Qt.DashLine
    messages = [
        window.validation_list.item(index).text()
        for index in range(window.validation_list.count())
    ]
    assert any(
        "exclusion_outside_field" in message and "exclusion_01" in message
        for message in messages
    )


def test_unsaved_prompt_supports_cancel_and_discard(window, monkeypatch):
    window.add_feature(_required_features()[0])
    monkeypatch.setattr(
        QMessageBox,
        "question",
        lambda *args: QMessageBox.Cancel,
    )
    assert not window._confirm_discard_changes()

    monkeypatch.setattr(
        QMessageBox,
        "question",
        lambda *args: QMessageBox.Discard,
    )
    assert window._confirm_discard_changes()


def test_object_id_and_name_edit_are_one_undoable_change(window):
    window.add_feature(_required_features()[0])
    window.selected_feature_id = "field_01"
    window.id_edit.setText("field_primary")
    window.name_edit.setText("Primary field")

    window.apply_properties()
    assert window.model_scene.get("field_01") is None
    assert window.model_scene.get("field_primary").name == "Primary field"
    window.undo()
    assert window.model_scene.get("field_01") is not None
