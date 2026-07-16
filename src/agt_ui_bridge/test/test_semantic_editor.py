import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtCore import QPointF, Qt  # noqa: E402
from PyQt5.QtWidgets import QApplication, QMessageBox  # noqa: E402
from diagnostic_msgs.msg import (  # noqa: E402
    DiagnosticArray,
    DiagnosticStatus,
    KeyValue,
)
from nav_msgs.msg import Path as NavPath  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from std_msgs.msg import String  # noqa: E402
import numpy as np  # noqa: E402
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


def test_wheel_zoom_can_recover_from_a_tiny_prefit_scale(window):
    window.view.resetTransform()
    window.view.scale(0.001, 0.001)
    before = window.view.transform().m11()

    assert window.view.zoom_by_wheel_delta(120)
    assert window.view.transform().m11() > before
    assert window.view.zoom_by_wheel_delta(-120)
    assert window.view.transform().m11() == pytest.approx(before)


def test_first_show_refits_map_to_final_viewport(window, application):
    scale_before_show = window.view.transform().m11()

    window.show()
    application.processEvents()

    assert window.view.transform().m11() > scale_before_show


def test_new_empty_task_shows_onboarding_instead_of_four_errors(window):
    messages = [
        window.validation_list.item(index).text()
        for index in range(window.validation_list.count())
    ]

    assert messages == [
        "开始标注 · 请依次绘制作业区、内部障碍、入口位姿和作业方向"
    ]
    assert not any(message.startswith("ERROR") for message in messages)


def test_incomplete_task_shows_required_features_as_pending(window):
    window.add_feature(_required_features()[0])
    messages = [
        window.validation_list.item(index).text()
        for index in range(window.validation_list.count())
    ]

    assert "待绘制 · 内部障碍（保存前必需）" in messages
    assert "待绘制 · 入口位姿（保存前必需）" in messages
    assert "待绘制 · 作业方向（保存前必需）" in messages
    assert not any("missing_feature_type" in message for message in messages)


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


def test_semantic_lines_use_high_contrast_colors_and_halos(window):
    _populate_required_features(window)

    for item in window._feature_items.values():
        assert isinstance(item, EDITOR.ContrastPathItem)
        assert item.pen().widthF() == pytest.approx(3.2)
        assert item.pen().color().lightness() < 160

    window.tool = "work_direction"
    window.graphics_scene.draw_points = [QPointF(1.0, 1.0), QPointF(8.0, 1.0)]
    window.graphics_scene._update_preview()
    preview = window.graphics_scene.preview_item
    assert isinstance(preview, EDITOR.ContrastPathItem)
    assert preview.pen().color() == EDITOR.FEATURE_COLORS["work_direction"]
    assert preview.pen().widthF() == pytest.approx(3.0)


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


def test_map_brush_can_modify_and_save_the_base_map(window):
    assert window.map_array is not None
    original_pixel = int(window.map_array[0, 0])

    window.set_tool("map_occupied")
    window.map_brush_size = 1
    window.apply_map_brush(QPointF(0.0, 0.0))

    assert int(window.map_array[0, 0]) == EDITOR.OCCUPIED_PIXEL
    assert window.map_dirty
    assert window._save_map_in_place()
    assert not window.map_dirty

    reloaded = EDITOR.SemanticEditorWindow(
        platform_profile_path=PROFILE,
        map_path=window.map_path,
    )
    try:
        assert int(reloaded.map_array[0, 0]) == EDITOR.OCCUPIED_PIXEL
    finally:
        reloaded.close()

    window.map_array[0, 0] = original_pixel
    window._update_map_pixmap()
    window.map_dirty = True
    assert window._save_map_in_place()


def test_map_brush_size_controls_pixel_width(window):
    window.set_tool("map_free")
    window.map_brush_size = 1
    window._change_map_brush(1)
    assert window.map_brush_size == 2

    window.apply_map_brush(QPointF(5.0, 5.0))
    assert np.all(window.map_array[5:7, 5:7] == EDITOR.FREE_PIXEL)

    window.map_brush_size = 1
    window._change_map_brush(-1)
    assert window.map_brush_size == 1
    window.map_dirty = False


def test_map_line_draws_a_continuous_trinary_stroke(window):
    window.set_tool("map_occupied")
    window.set_map_draw_mode("line")
    window.map_brush_size = 1

    window.apply_map_line(QPointF(2.0, 4.0), QPointF(12.0, 4.0))

    assert np.all(window.map_array[4, 2:13] == EDITOR.OCCUPIED_PIXEL)
    assert window.map_dirty
    window.map_dirty = False


def test_map_stroke_can_be_undone_and_redone(window):
    window.set_tool("map_occupied")
    original = window.map_array.copy()
    window.begin_map_edit()
    window.apply_map_line(QPointF(2.0, 3.0), QPointF(8.0, 3.0))
    changed = window.map_array.copy()
    assert window.commit_map_edit()

    window.undo()
    assert np.array_equal(window.map_array, original)
    window.redo()
    assert np.array_equal(window.map_array, changed)
    window.map_array = original
    window._update_map_pixmap()
    window.map_dirty = False


def test_finished_feature_switches_to_vertex_editing(window, monkeypatch):
    monkeypatch.setattr(window, "_prompt_text", lambda *args: args[-1])
    window.set_tool("field_boundary")

    assert window.finish_feature_from_scene(
        "field_boundary",
        [
            window._world_point([1.0, 1.0]),
            window._world_point([7.0, 1.0]),
            window._world_point([7.0, 5.0]),
            window._world_point([1.0, 5.0]),
        ],
    )

    assert window.tool == "select"
    assert window.selected_feature_id == "field_01"


def test_self_intersecting_polygon_stays_as_repairable_draft(window, monkeypatch):
    warnings = []
    monkeypatch.setattr(QMessageBox, "warning", lambda *args: warnings.append(args))
    window.set_tool("field_boundary")
    window.graphics_scene.draw_points = [
        QPointF(2, 2),
        QPointF(12, 12),
        QPointF(12, 2),
        QPointF(2, 12),
    ]

    window.graphics_scene.finish_drawing()

    assert len(window.graphics_scene.draw_points) == 4
    assert not window.model_scene.features
    assert warnings


def test_selected_vertex_supports_fine_keyboard_nudge_and_undo(window):
    window.add_feature(_required_features()[0])
    original = list(window.model_scene.get("field_01").coordinates[0][0])
    window.set_tool("select")
    window.selected_feature_id = "field_01"
    window.selected_vertex = ("field_01", 0)

    assert window._nudge_selected_vertex(1.0, 0.0, fine=True)
    moved = window.model_scene.get("field_01").coordinates[0][0]
    assert moved != original
    window.undo()
    assert window.model_scene.get("field_01").coordinates[0][0] == original


def test_selecting_vertex_does_not_delete_event_target_synchronously(window):
    window.add_feature(_required_features()[0])
    window.set_tool("select")
    window.selected_feature_id = "field_01"
    window.refresh_scene()
    handle = next(
        item
        for item in window.graphics_scene.items()
        if isinstance(item, EDITOR.VertexHandle)
    )

    handle.setSelected(True)

    assert handle.scene() is window.graphics_scene
    assert window.selected_vertex == (handle.feature_id, handle.vertex_key)


def test_preview_mode_derives_aisles_and_selects_dubins(window):
    window.add_feature(_required_features()[3])
    for index, y_value in enumerate((5.0, 3.0, 1.0), start=1):
        window.add_feature(
            SemanticFeature(
                id=f"row_{index:02d}",
                feature_type="row_centerline",
                name="Crop row",
                geometry_type="LineString",
                coordinates=[[1.0, y_value], [7.0, y_value]],
            )
        )
    window.preview_mode_combo.setCurrentIndex(
        window.preview_mode_combo.findData("inter_row_aisles")
    )
    window.preview_path_combo.setCurrentIndex(
        window.preview_path_combo.findData("dubins")
    )

    semantic_map, coverage = window._preview_task()
    rows = [item for item in semantic_map.features if item.feature_type == "row_centerline"]

    assert len(rows) == 2
    assert coverage.planning_mode == "annotated_rows"
    assert not coverage.allow_reverse


def test_preview_path_is_rendered_and_summarized(window):
    window._preview_run_active = True
    window._preview_accept_after_ns = 0
    message = NavPath()
    for x_value in (1.0, 2.0, 4.0):
        pose = PoseStamped()
        pose.pose.position.x = x_value
        pose.pose.position.y = 1.0
        message.poses.append(pose)

    window._preview_path_callback(message)

    assert window._preview_path_summary["路径点"] == 3
    assert window._preview_path_summary["长度"] == "3.00 m"
    assert window._preview_world_points == [(1.0, 1.0), (2.0, 1.0), (4.0, 1.0)]


def test_preview_ignores_latched_metrics_until_current_path_arrives(window):
    window._preview_run_active = True
    window._preview_has_current_path = False
    report = String()
    report.data = json.dumps(
        {
            "estimated_motion_time": 115.22,
            "estimated_turn_count": 2,
            "reverse_path_length": 0.49,
        }
    )

    window._preview_report_callback(report)

    assert "预计时间" not in window._preview_path_summary
    assert "预计转弯" not in window._preview_path_summary
    assert "倒车距离" not in window._preview_path_summary


def test_preview_waiting_reason_is_cleared_when_planning_starts(window):
    window._preview_run_active = True
    window._preview_accept_after_ns = 0

    waiting = DiagnosticArray()
    waiting_status = DiagnosticStatus()
    waiting_status.name = "agt_coverage_request_adapter"
    waiting_status.message = "WAITING_FOR_SERVER"
    waiting_status.values = [
        KeyValue(key="error_code", value="action_server_unavailable"),
        KeyValue(key="detail", value="annotated_rows coverage action is not ready"),
    ]
    waiting.status = [waiting_status]
    window._preview_status_callback(waiting)

    assert "正在自动重试" in window._preview_status
    assert window._preview_path_summary["等待原因"] == "action_server_unavailable"
    assert window._preview_path_summary["详情"] == (
        "annotated_rows coverage action is not ready"
    )
    assert "错误码" not in window._preview_path_summary

    planning = DiagnosticArray()
    planning_status = DiagnosticStatus()
    planning_status.name = "agt_coverage_request_adapter"
    planning_status.message = "PLANNING"
    planning_status.values = [KeyValue(key="error_code", value="none")]
    planning.status = [planning_status]
    window._preview_status_callback(planning)

    assert window._preview_status == "PLANNING"
    assert "等待原因" not in window._preview_path_summary
    assert "错误码" not in window._preview_path_summary
    assert "详情" not in window._preview_path_summary


def test_preview_rejection_exposes_diagnostic_detail(window):
    window._preview_run_active = True
    window._preview_accept_after_ns = 0
    message = DiagnosticArray()
    status = DiagnosticStatus()
    status.name = "agt_coverage_request_adapter"
    status.message = "REJECTED"
    status.values = [
        KeyValue(key="error_code", value="task_load_failed"),
        KeyValue(key="detail", value="example load failure"),
    ]
    message.status = [status]

    window._preview_status_callback(message)

    assert window._preview_path_summary["错误码"] == "task_load_failed"
    assert window._preview_path_summary["详情"] == "example load failure"
