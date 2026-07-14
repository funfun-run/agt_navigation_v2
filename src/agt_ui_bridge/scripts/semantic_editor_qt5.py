#!/usr/bin/env python3

import argparse
from copy import deepcopy
from dataclasses import dataclass
import math
import os
from pathlib import Path
import re
import sys

from PIL import Image, ImageOps
from PyQt5.QtCore import QPoint, QPointF, Qt
from PyQt5.QtGui import QColor, QImage, QPainter, QPainterPath, QPen, QPixmap
from PyQt5.QtWidgets import (
    QAction,
    QActionGroup,
    QApplication,
    QCheckBox,
    QDockWidget,
    QFileDialog,
    QFormLayout,
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsPathItem,
    QGraphicsPixmapItem,
    QGraphicsScene,
    QGraphicsView,
    QHBoxLayout,
    QInputDialog,
    QLineEdit,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)
import yaml

from agt_ui_bridge.map_transform import MapGeometry, MapTransform
from agt_ui_bridge.platform_profile import (
    load_platform_profile,
    resolve_platform_profile,
)
from agt_ui_bridge.semantic_io import (
    SemanticFileError,
    load_semantic_task,
    save_semantic_task,
    sha256_file,
)
from agt_ui_bridge.semantic_model import (
    CoverageParameters,
    SemanticFeature,
    SemanticMap,
)
from agt_ui_bridge.semantic_scene import SemanticScene
from agt_ui_bridge.semantic_validation import ValidationContext, validate_task


FEATURE_LABELS = {
    "field_boundary": "作业区",
    "exclusion_zone": "内部障碍",
    "row_centerline": "作物行",
    "entry_pose": "入口位姿",
    "work_direction": "作业方向",
    "headland_zone": "地头区",
    "keepout_zone": "禁行区",
}
FEATURE_COLORS = {
    "field_boundary": QColor("#2f9f72"),
    "exclusion_zone": QColor("#d85c41"),
    "row_centerline": QColor("#e5bd55"),
    "entry_pose": QColor("#3f83d5"),
    "work_direction": QColor("#36a9b5"),
    "headland_zone": QColor("#d28a3f"),
    "keepout_zone": QColor("#b94a62"),
}
DRAW_TO_FEATURE = {
    "field_boundary": "field_boundary",
    "exclusion_zone": "exclusion_zone",
    "row_centerline": "row_centerline",
    "entry_pose": "entry_pose",
    "work_direction": "work_direction",
}


@dataclass
class EditorDefaults:
    robot_profile: str = "bunker"
    planning_mode: str = "polygon"
    operation_width: float = 0.60
    headland_width: float = 1.50
    allow_reverse: bool = True
    preferred_swath_angle: float = 0.0
    minimum_boundary_clearance: float = 0.0
    history_limit: int = 100

    @classmethod
    def from_yaml(cls, path):
        if not path:
            return cls()
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        values = data.get("semantic_editor", data)
        return cls(
            robot_profile=str(values.get("robot_profile", "bunker")),
            planning_mode=str(values.get("planning_mode", "polygon")),
            operation_width=float(values.get("operation_width", 0.60)),
            headland_width=float(values.get("headland_width", 1.50)),
            allow_reverse=bool(values.get("allow_reverse", True)),
            preferred_swath_angle=float(
                values.get("preferred_swath_angle", 0.0)
            ),
            minimum_boundary_clearance=float(
                values.get("minimum_boundary_clearance", 0.0)
            ),
            history_limit=int(values.get("history_limit", 100)),
        )


class SemanticGraphicsView(QGraphicsView):
    def __init__(self, scene, parent=None):
        super().__init__(scene, parent)
        self.setRenderHints(self.renderHints() | QPainter.Antialiasing)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setMouseTracking(True)
        self._panning = False
        self._pan_start = QPoint()

    def wheelEvent(self, event):
        factor = 1.15 if event.angleDelta().y() > 0 else 1.0 / 1.15
        next_scale = self.transform().m11() * factor
        if 0.05 <= next_scale <= 100.0:
            self.scale(factor, factor)

    def mousePressEvent(self, event):
        if event.button() == Qt.MiddleButton:
            self._panning = True
            self._pan_start = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._panning:
            delta = event.pos() - self._pan_start
            self._pan_start = event.pos()
            self.horizontalScrollBar().setValue(
                self.horizontalScrollBar().value() - delta.x()
            )
            self.verticalScrollBar().setValue(
                self.verticalScrollBar().value() - delta.y()
            )
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MiddleButton and self._panning:
            self._panning = False
            self.unsetCursor()
            event.accept()
            return
        super().mouseReleaseEvent(event)


class FeatureGraphicsItem(QGraphicsPathItem):
    def __init__(self, feature_id, feature_type, path, invalid=False):
        super().__init__(path)
        self.feature_id = feature_id
        color = QColor("#ff3b30") if invalid else FEATURE_COLORS[feature_type]
        pen = QPen(color, 2.2)
        pen.setCosmetic(True)
        if invalid:
            pen.setStyle(Qt.DashLine)
        self.setPen(pen)
        fill = QColor(color)
        is_polygon = feature_type.endswith("zone") or feature_type == "field_boundary"
        fill.setAlpha(45 if is_polygon else 0)
        self.setBrush(fill)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setZValue(10)
        self.setToolTip(f"{feature_id} · {FEATURE_LABELS[feature_type]}")


class VertexHandle(QGraphicsEllipseItem):
    def __init__(self, feature_id, vertex_key, position, bounds, callback):
        super().__init__(-4.5, -4.5, 9.0, 9.0)
        self.feature_id = feature_id
        self.vertex_key = vertex_key
        self._bounds = bounds
        self._callback = callback
        self._start_position = QPointF(position)
        self.setPos(position)
        self.setBrush(QColor("#f6f1df"))
        pen = QPen(QColor("#18211f"), 1.5)
        pen.setCosmetic(True)
        self.setPen(pen)
        self.setFlags(
            QGraphicsItem.ItemIsMovable
            | QGraphicsItem.ItemIsSelectable
            | QGraphicsItem.ItemSendsGeometryChanges
            | QGraphicsItem.ItemIgnoresTransformations
        )
        self.setZValue(30)

    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemPositionChange and self.scene() is not None:
            point = value
            return QPointF(
                min(max(point.x(), self._bounds.left()), self._bounds.right()),
                min(max(point.y(), self._bounds.top()), self._bounds.bottom()),
            )
        return super().itemChange(change, value)

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        if self.pos() != self._start_position:
            self._callback(self.feature_id, self.vertex_key, self.pos())


class SemanticGraphicsScene(QGraphicsScene):
    def __init__(self, editor):
        super().__init__(editor)
        self.editor = editor
        self.draw_points = []
        self.preview_item = None

    def cancel_drawing(self):
        self.draw_points.clear()
        if self.preview_item is not None:
            self.removeItem(self.preview_item)
            self.preview_item = None
        self.editor.statusBar().showMessage("已取消当前绘制", 2500)

    def finish_drawing(self):
        if not self.draw_points:
            return
        minimum = 3 if self.editor.tool in {"field_boundary", "exclusion_zone"} else 2
        if self.editor.tool == "entry_pose":
            minimum = 2
        if len(self.draw_points) < minimum:
            self.editor.statusBar().showMessage(
                f"当前对象至少需要 {minimum} 个点", 3000
            )
            return
        points = list(self.draw_points)
        self.cancel_drawing()
        self.editor.finish_feature_from_scene(self.editor.tool, points)

    def mousePressEvent(self, event):
        if self.editor.tool == "select" or self.editor.read_only:
            super().mousePressEvent(event)
            return
        if event.button() == Qt.RightButton:
            self.finish_drawing()
            event.accept()
            return
        if event.button() != Qt.LeftButton:
            super().mousePressEvent(event)
            return
        point = event.scenePos()
        if not self.sceneRect().contains(point):
            return
        self.draw_points.append(QPointF(point))
        self._update_preview()
        if self.editor.tool in {"entry_pose", "work_direction"} and len(self.draw_points) == 2:
            self.finish_drawing()
        event.accept()

    def mouseDoubleClickEvent(self, event):
        if self.editor.tool in {
            "field_boundary",
            "exclusion_zone",
            "row_centerline",
        }:
            if not self.draw_points or self.draw_points[-1] != event.scenePos():
                self.draw_points.append(QPointF(event.scenePos()))
            self.finish_drawing()
            event.accept()
            return
        super().mouseDoubleClickEvent(event)

    def mouseMoveEvent(self, event):
        self.editor.update_cursor_position(event.scenePos())
        super().mouseMoveEvent(event)

    def _update_preview(self):
        if self.preview_item is not None:
            self.removeItem(self.preview_item)
        path = QPainterPath()
        path.moveTo(self.draw_points[0])
        for point in self.draw_points[1:]:
            path.lineTo(point)
        self.preview_item = QGraphicsPathItem(path)
        pen = QPen(QColor("#f6f1df"), 1.5, Qt.DashLine)
        pen.setCosmetic(True)
        self.preview_item.setPen(pen)
        self.preview_item.setZValue(40)
        self.addItem(self.preview_item)


class SemanticEditorWindow(QMainWindow):
    def __init__(
        self,
        defaults=None,
        platform_profile_path=None,
        map_path=None,
        semantic_path=None,
    ):
        super().__init__()
        self.defaults = defaults or EditorDefaults()
        self.platform_profile_path = (
            Path(platform_profile_path).resolve()
            if platform_profile_path
            else None
        )
        self.platform = None
        self.navigation_footprint = []
        self.map_path = None
        self.semantic_path = None
        self.coverage_path = None
        self.coverage = None
        self.model_scene = None
        self.transformer = None
        self.map_pixmap = None
        self.read_only = False
        self.tool = "select"
        self.selected_feature_id = None
        self._rendering = False
        self._feature_items = {}
        self._layer_checks = {}
        self._tool_actions = {}

        self.setWindowTitle("AGT 农业语义地图编辑器")
        self.resize(1420, 880)
        self.graphics_scene = SemanticGraphicsScene(self)
        self.view = SemanticGraphicsView(self.graphics_scene, self)
        self._build_ui()
        self.graphics_scene.selectionChanged.connect(self._selection_changed)
        self.statusBar().showMessage("加载 Nav2 地图或语义任务开始标注")

        if semantic_path:
            self.load_task(semantic_path)
        elif map_path:
            self.load_base_map(map_path)

    def _build_ui(self):
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #17201e; color: #e9eadf; }
            QToolBar { background: #23302c; border: 0; spacing: 5px; }
            QToolButton, QPushButton { background: #31423c; padding: 6px; border-radius: 3px; }
            QToolButton:checked { background: #d69a43; color: #17201e; }
            QLineEdit, QTreeWidget, QListWidget { background: #101714; border: 1px solid #40534c; }
            QDockWidget::title { background: #23302c; padding: 5px; }
            """
        )
        self.setCentralWidget(self.view)
        file_toolbar = self.addToolBar("文件")
        for label, callback in (
            ("加载底图", self.choose_base_map),
            ("打开任务", self.choose_task),
            ("保存", self.save),
            ("另存为", self.save_as),
            ("重新加载", self.reload),
        ):
            action = QAction(label, self)
            action.triggered.connect(callback)
            file_toolbar.addAction(action)
            if label == "保存":
                self.save_action = action
        file_toolbar.addSeparator()
        fit_action = QAction("适配窗口", self)
        fit_action.triggered.connect(self.fit_map)
        file_toolbar.addAction(fit_action)

        edit_toolbar = self.addToolBar("编辑")
        undo_action = QAction("撤销", self)
        undo_action.setShortcut("Ctrl+Z")
        undo_action.triggered.connect(self.undo)
        redo_action = QAction("重做", self)
        redo_action.setShortcut("Ctrl+Shift+Z")
        redo_action.triggered.connect(self.redo)
        delete_action = QAction("删除", self)
        delete_action.setShortcut("Delete")
        delete_action.triggered.connect(self.delete_selected)
        edit_toolbar.addActions([undo_action, redo_action, delete_action])

        tools = self.addToolBar("绘制")
        tool_group = QActionGroup(self)
        tool_group.setExclusive(True)
        for tool, label in (
            ("select", "选择/顶点"),
            ("field_boundary", "作业区"),
            ("exclusion_zone", "内部障碍"),
            ("row_centerline", "作物行"),
            ("entry_pose", "入口位姿"),
            ("work_direction", "作业方向"),
        ):
            action = QAction(label, self)
            action.setCheckable(True)
            action.triggered.connect(
                lambda checked, selected=tool: checked and self.set_tool(selected)
            )
            tool_group.addAction(action)
            tools.addAction(action)
            self._tool_actions[tool] = action
        self._tool_actions["select"].setChecked(True)

        self._build_object_dock()
        self._build_layer_dock()
        self._build_validation_dock()

    def _build_object_dock(self):
        dock = QDockWidget("语义对象", self)
        panel = QWidget()
        layout = QVBoxLayout(panel)
        self.object_tree = QTreeWidget()
        self.object_tree.setHeaderLabels(["ID", "类型", "名称"])
        self.object_tree.itemSelectionChanged.connect(self._tree_selection_changed)
        layout.addWidget(self.object_tree, 1)
        form = QFormLayout()
        self.id_edit = QLineEdit()
        self.name_edit = QLineEdit()
        form.addRow("对象 ID", self.id_edit)
        form.addRow("名称", self.name_edit)
        layout.addLayout(form)
        buttons = QHBoxLayout()
        apply_button = QPushButton("应用属性")
        apply_button.clicked.connect(self.apply_properties)
        delete_button = QPushButton("删除对象")
        delete_button.clicked.connect(self.delete_selected)
        buttons.addWidget(apply_button)
        buttons.addWidget(delete_button)
        layout.addLayout(buttons)
        dock.setWidget(panel)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)

    def _build_layer_dock(self):
        dock = QDockWidget("图层", self)
        panel = QWidget()
        layout = QVBoxLayout(panel)
        base_check = QCheckBox("基础 OccupancyGrid")
        base_check.setChecked(True)
        base_check.toggled.connect(self.refresh_scene)
        self._layer_checks["base_map"] = base_check
        layout.addWidget(base_check)
        for feature_type, label in FEATURE_LABELS.items():
            checkbox = QCheckBox(label)
            checkbox.setChecked(True)
            checkbox.toggled.connect(self.refresh_scene)
            self._layer_checks[feature_type] = checkbox
            layout.addWidget(checkbox)
        footprint_check = QCheckBox("车辆 footprint 预览")
        footprint_check.setChecked(True)
        footprint_check.toggled.connect(self.refresh_scene)
        self._layer_checks["footprint"] = footprint_check
        layout.addStretch(1)
        dock.setWidget(panel)
        self.addDockWidget(Qt.LeftDockWidgetArea, dock)

    def _build_validation_dock(self):
        dock = QDockWidget("合法性", self)
        self.validation_list = QListWidget()
        dock.setWidget(self.validation_list)
        self.addDockWidget(Qt.BottomDockWidgetArea, dock)

    def choose_base_map(self):
        if not self._confirm_discard_changes():
            return
        path, _ = QFileDialog.getOpenFileName(
            self, "加载 Nav2 地图", "", "Nav2 map (*.yaml)"
        )
        if path:
            try:
                self.load_base_map(path)
            except Exception as exc:
                QMessageBox.critical(self, "地图加载失败", str(exc))

    def choose_task(self):
        if not self._confirm_discard_changes():
            return
        path, _ = QFileDialog.getOpenFileName(
            self, "打开语义地图", "", "GeoJSON (*.geojson *.json)"
        )
        if path:
            try:
                self.load_task(path)
            except Exception as exc:
                QMessageBox.critical(self, "任务加载失败", str(exc))

    def load_base_map(self, map_path, profile_path=None):
        map_path = Path(map_path).expanduser().resolve()
        profile_path = profile_path or self.platform_profile_path
        if profile_path is None:
            profile_path = resolve_platform_profile(self.defaults.robot_profile)
        if profile_path is None:
            selected, _ = QFileDialog.getOpenFileName(
                self, "选择平台 profile", "", "YAML (*.yaml)"
            )
            if not selected:
                raise SemanticFileError("new semantic task requires a platform profile")
            profile_path = Path(selected)
        self._set_platform_profile(profile_path)
        self._set_base_map(map_path)

        map_id = map_path.stem
        semantic_map = SemanticMap(map_id=map_id)
        default_semantic_dir = map_path.parent / "semantic"
        coverage_path = default_semantic_dir / "coverage.yaml"
        base_map = os.path.relpath(map_path, coverage_path.parent)
        self.coverage = CoverageParameters(
            map_id=map_id,
            base_map=base_map,
            base_map_sha256=sha256_file(map_path),
            robot_profile=self.platform["name"],
            planning_mode=self.defaults.planning_mode,
            robot_width=self.platform["robot_width"],
            operation_width=self.defaults.operation_width,
            min_turning_radius=self.platform["min_turning_radius"],
            headland_width=self.defaults.headland_width,
            allow_reverse=self.defaults.allow_reverse,
            preferred_swath_angle=self.defaults.preferred_swath_angle,
        )
        self.model_scene = SemanticScene(
            semantic_map, history_limit=self.defaults.history_limit
        )
        self.semantic_path = default_semantic_dir / "semantic_map.geojson"
        self.coverage_path = coverage_path
        self.read_only = False
        self.selected_feature_id = None
        self.refresh_scene()
        self.fit_map()
        self._update_title()

    def load_task(self, semantic_path):
        task = load_semantic_task(semantic_path)
        profile_path = self.platform_profile_path or resolve_platform_profile(
            task.coverage.robot_profile
        )
        if profile_path is not None:
            self._set_platform_profile(profile_path)
        else:
            self.platform = None
            self.navigation_footprint = []
        self._set_base_map(task.base_map_path)
        self.coverage = task.coverage
        self.model_scene = SemanticScene(
            task.semantic_map, history_limit=self.defaults.history_limit
        )
        self.semantic_path = task.semantic_path
        self.coverage_path = task.coverage_path
        self.read_only = task.read_only
        full_report = validate_task(
            self.model_scene.semantic_map,
            self.coverage,
            context=self._validation_context(),
        )
        if not full_report.valid:
            self.read_only = True
            task.warnings.extend(issue.code for issue in full_report.issues)
        self.selected_feature_id = None
        self.refresh_scene()
        self.fit_map()
        self._update_title()
        if task.warnings:
            QMessageBox.warning(
                self,
                "任务以只读方式打开",
                "\n".join(dict.fromkeys(task.warnings)),
            )

    def _set_platform_profile(self, path):
        path = Path(path).expanduser().resolve()
        self.platform = load_platform_profile(path)
        self.navigation_footprint = self.platform["footprint"]
        self.platform_profile_path = path

    def _set_base_map(self, map_path):
        self.map_path = Path(map_path).expanduser().resolve()
        self.transformer = MapTransform(MapGeometry.from_nav2_yaml(self.map_path))
        metadata = yaml.safe_load(self.map_path.read_text(encoding="utf-8"))
        image_path = Path(metadata["image"])
        if not image_path.is_absolute():
            image_path = self.map_path.parent / image_path
        image = Image.open(image_path).convert("L")
        if bool(metadata.get("negate", 0)):
            image = ImageOps.invert(image)
        data = image.tobytes("raw", "L")
        qimage = QImage(
            data,
            image.width,
            image.height,
            image.width,
            QImage.Format_Grayscale8,
        ).copy()
        self.map_pixmap = QPixmap.fromImage(qimage)
        self.graphics_scene.setSceneRect(
            0.0, 0.0, float(image.width), float(image.height)
        )

    def set_tool(self, tool):
        if self.read_only and tool != "select":
            self._tool_actions["select"].setChecked(True)
            return
        self.graphics_scene.cancel_drawing()
        self.tool = tool
        self.refresh_scene()
        if tool == "select":
            self.statusBar().showMessage("选择对象或拖动顶点；中键平移，滚轮缩放")
        elif tool in {"field_boundary", "exclusion_zone", "row_centerline"}:
            self.statusBar().showMessage("左键添加顶点，双击/右键/Enter 完成，Esc 取消")
        else:
            self.statusBar().showMessage("点击位置，再点击方向点")

    def finish_feature_from_scene(self, tool, scene_points, feature_id=None, name=None):
        if self.read_only or self.model_scene is None:
            return False
        try:
            world_points = [
                list(self.transformer.scene_to_world(point.x(), point.y()))
                for point in scene_points
            ]
        except ValueError as exc:
            QMessageBox.warning(self, "坐标超出地图", str(exc))
            return False

        feature_type = DRAW_TO_FEATURE[tool]
        proposed_id = self._next_feature_id(feature_type)
        feature_id = feature_id or self._prompt_text(
            "对象 ID", "输入唯一 snake_case ID", proposed_id
        )
        if not feature_id:
            return False
        name = name or self._prompt_text(
            "对象名称", "输入显示名称", FEATURE_LABELS[feature_type]
        )
        if name is None:
            return False

        properties = {}
        if feature_type in {"field_boundary", "exclusion_zone"}:
            coordinates = [world_points + [world_points[0]]]
            geometry_type = "Polygon"
        elif feature_type == "row_centerline":
            coordinates = world_points
            geometry_type = "LineString"
        elif feature_type == "work_direction":
            coordinates = world_points[:2]
            geometry_type = "LineString"
        else:
            start, direction = world_points[:2]
            coordinates = start
            geometry_type = "Point"
            properties["yaw"] = math.atan2(
                direction[1] - start[1], direction[0] - start[0]
            )
        feature = SemanticFeature(
            id=feature_id,
            feature_type=feature_type,
            name=name,
            geometry_type=geometry_type,
            coordinates=coordinates,
            properties=properties,
        )
        try:
            self.model_scene.add(feature)
        except ValueError as exc:
            QMessageBox.warning(self, "对象创建失败", str(exc))
            return False
        self.selected_feature_id = feature.id
        self.refresh_scene()
        return True

    def add_feature(self, feature):
        self.model_scene.add(feature)
        self.selected_feature_id = feature.id
        self.refresh_scene()

    def _prompt_text(self, title, label, initial):
        value, accepted = QInputDialog.getText(
            self, title, label, QLineEdit.Normal, initial
        )
        return value.strip() if accepted else None

    def _next_feature_id(self, feature_type):
        prefix = {
            "field_boundary": "field",
            "exclusion_zone": "exclusion",
            "row_centerline": "row",
            "entry_pose": "entry",
            "work_direction": "direction",
        }[feature_type]
        index = 1
        while self.model_scene.get(f"{prefix}_{index:02d}") is not None:
            index += 1
        return f"{prefix}_{index:02d}"

    def refresh_scene(self):
        if self._rendering:
            return
        self._rendering = True
        try:
            selected_id = self.selected_feature_id
            self.graphics_scene.clear()
            self.graphics_scene.preview_item = None
            self.graphics_scene.draw_points.clear()
            self._feature_items = {}
            if self.map_pixmap is not None and self._layer_visible("base_map"):
                map_item = QGraphicsPixmapItem(self.map_pixmap)
                map_item.setZValue(-100)
                self.graphics_scene.addItem(map_item)
            if self.model_scene is None:
                return
            report = validate_task(
                self.model_scene.semantic_map,
                self.coverage,
                context=self._validation_context(),
            )
            invalid_ids = {
                issue.object_id
                for issue in report.issues
                if issue.object_id not in {"<document>"} and issue.severity == "ERROR"
            }
            for feature in self.model_scene.features:
                if not self._layer_visible(feature.feature_type):
                    continue
                item = FeatureGraphicsItem(
                    feature.id,
                    feature.feature_type,
                    self._feature_path(feature),
                    invalid=feature.id in invalid_ids,
                )
                self.graphics_scene.addItem(item)
                self._feature_items[feature.id] = item
                if feature.id == selected_id:
                    item.setSelected(True)
            self._render_footprints()
            if (
                self.tool == "select"
                and selected_id
                and not self.read_only
                and self.model_scene.get(selected_id) is not None
                and self._layer_visible(
                    self.model_scene.get(selected_id).feature_type
                )
            ):
                self._render_handles(self.model_scene.get(selected_id))
            self._refresh_tree()
            self._refresh_validation(report)
        finally:
            self._rendering = False

    def _feature_path(self, feature):
        path = QPainterPath()
        if feature.geometry_type == "Polygon":
            points = [self._world_point(point) for point in feature.coordinates[0]]
            if points:
                path.moveTo(points[0])
                for point in points[1:]:
                    path.lineTo(point)
                path.closeSubpath()
        elif feature.geometry_type == "LineString":
            points = [self._world_point(point) for point in feature.coordinates]
            if points:
                path.moveTo(points[0])
                for point in points[1:]:
                    path.lineTo(point)
        elif feature.geometry_type == "Point":
            point = self._world_point(feature.coordinates)
            radius = max(0.12 / self.transformer.geometry.resolution, 0.12)
            path.addEllipse(point, radius, radius)
            yaw = float(feature.properties.get("yaw", 0.0))
            endpoint = [
                feature.coordinates[0] + 0.8 * math.cos(yaw),
                feature.coordinates[1] + 0.8 * math.sin(yaw),
            ]
            path.moveTo(point)
            path.lineTo(self._world_point(endpoint))
        return path

    def _world_point(self, coordinate):
        scene_x, scene_y = self.transformer.world_to_scene_unbounded(
            coordinate[0], coordinate[1]
        )
        return QPointF(scene_x, scene_y)

    def _render_handles(self, feature):
        handles = []
        if feature.geometry_type == "Polygon":
            handles = list(enumerate(feature.coordinates[0][:-1]))
        elif feature.geometry_type == "LineString":
            handles = list(enumerate(feature.coordinates))
        elif feature.geometry_type == "Point":
            handles = [(0, feature.coordinates)]
            yaw = float(feature.properties.get("yaw", 0.0))
            handles.append(
                (
                    "yaw",
                    [
                        feature.coordinates[0] + 0.8 * math.cos(yaw),
                        feature.coordinates[1] + 0.8 * math.sin(yaw),
                    ],
                )
            )
        for key, coordinate in handles:
            handle = VertexHandle(
                feature.id,
                key,
                self._world_point(coordinate),
                self.graphics_scene.sceneRect(),
                self._vertex_moved,
            )
            self.graphics_scene.addItem(handle)

    def _vertex_moved(self, feature_id, vertex_key, scene_position):
        feature = self.model_scene.get(feature_id)
        if feature is None:
            return
        updated = SemanticFeature.from_geojson(feature.to_geojson())
        try:
            world = list(
                self.transformer.scene_to_world(
                    scene_position.x(), scene_position.y()
                )
            )
        except ValueError:
            self.refresh_scene()
            return
        if vertex_key == "yaw":
            updated.properties["yaw"] = math.atan2(
                world[1] - updated.coordinates[1],
                world[0] - updated.coordinates[0],
            )
        elif updated.geometry_type == "Polygon":
            updated.coordinates[0][vertex_key] = world
            if vertex_key == 0:
                updated.coordinates[0][-1] = list(world)
        elif updated.geometry_type == "LineString":
            updated.coordinates[vertex_key] = world
        else:
            updated.coordinates = world
        self.model_scene.replace(updated)
        self.selected_feature_id = updated.id
        self.refresh_scene()

    def _render_footprints(self):
        if not self.navigation_footprint or not self._layer_visible("footprint"):
            return
        for feature in self.model_scene.features:
            if feature.feature_type != "entry_pose" or not feature.enabled:
                continue
            yaw = float(feature.properties.get("yaw", 0.0))
            cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
            points = []
            for local_x, local_y in self.navigation_footprint:
                points.append(
                    self._world_point(
                        [
                            feature.coordinates[0]
                            + cos_yaw * local_x
                            - sin_yaw * local_y,
                            feature.coordinates[1]
                            + sin_yaw * local_x
                            + cos_yaw * local_y,
                        ]
                    )
                )
            path = QPainterPath()
            path.moveTo(points[0])
            for point in points[1:] + [points[0]]:
                path.lineTo(point)
            item = QGraphicsPathItem(path)
            pen = QPen(QColor("#f6f1df"), 1.2, Qt.DotLine)
            pen.setCosmetic(True)
            item.setPen(pen)
            item.setZValue(5)
            self.graphics_scene.addItem(item)

    def _layer_visible(self, name):
        checkbox = self._layer_checks.get(name)
        return checkbox is None or checkbox.isChecked()

    def _validation_context(self):
        return ValidationContext(
            map_geometry=(self.transformer.geometry if self.transformer else None),
            navigation_footprint=tuple(
                tuple(point) for point in self.navigation_footprint
            ),
            minimum_boundary_clearance=self.defaults.minimum_boundary_clearance,
            base_map_path=self.map_path,
        )

    def _selection_changed(self):
        if self._rendering:
            return
        selected = [
            item
            for item in self.graphics_scene.selectedItems()
            if isinstance(item, FeatureGraphicsItem)
        ]
        self.selected_feature_id = selected[0].feature_id if selected else None
        self.model_scene.selected_feature_id = self.selected_feature_id
        self.refresh_scene()

    def _tree_selection_changed(self):
        if self._rendering:
            return
        items = self.object_tree.selectedItems()
        self.selected_feature_id = items[0].data(0, Qt.UserRole) if items else None
        self.model_scene.selected_feature_id = self.selected_feature_id
        self.refresh_scene()

    def _refresh_tree(self):
        self.object_tree.blockSignals(True)
        self.object_tree.clear()
        for feature in self.model_scene.features:
            item = QTreeWidgetItem(
                [
                    feature.id,
                    FEATURE_LABELS.get(feature.feature_type, feature.feature_type),
                    feature.name,
                ]
            )
            item.setData(0, Qt.UserRole, feature.id)
            self.object_tree.addTopLevelItem(item)
            if feature.id == self.selected_feature_id:
                item.setSelected(True)
        self.object_tree.blockSignals(False)
        feature = self.model_scene.get(self.selected_feature_id)
        self.id_edit.setText(feature.id if feature else "")
        self.name_edit.setText(feature.name if feature else "")

    def _refresh_validation(self, report):
        self.validation_list.clear()
        if report.valid:
            self.validation_list.addItem("OK · 当前任务结构合法")
            return
        for issue in report.issues:
            self.validation_list.addItem(
                f"{issue.severity} · {issue.code} · {issue.object_id}: {issue.message}"
            )

    def apply_properties(self):
        if self.read_only or not self.selected_feature_id:
            return
        feature = self.model_scene.get(self.selected_feature_id)
        updated = SemanticFeature.from_geojson(feature.to_geojson())
        new_id = self.id_edit.text().strip()
        new_name = self.name_edit.text().strip()
        if not re.fullmatch(r"[a-z][a-z0-9_]*", new_id):
            QMessageBox.warning(self, "ID 非法", "ID 必须使用小写 snake_case")
            return
        updated.id = new_id
        updated.name = new_name
        try:
            self.model_scene.replace_by_id(self.selected_feature_id, updated)
        except ValueError as exc:
            QMessageBox.warning(self, "属性修改失败", str(exc))
            return
        self.selected_feature_id = updated.id
        self.refresh_scene()

    def delete_selected(self):
        if self.read_only or not self.selected_feature_id:
            return
        self.model_scene.remove(self.selected_feature_id)
        self.selected_feature_id = None
        self.refresh_scene()

    def undo(self):
        if self.model_scene and not self.read_only and self.model_scene.undo():
            self.selected_feature_id = self.model_scene.selected_feature_id
            self.refresh_scene()

    def redo(self):
        if self.model_scene and not self.read_only and self.model_scene.redo():
            self.selected_feature_id = self.model_scene.selected_feature_id
            self.refresh_scene()

    def save(self):
        if self.semantic_path is None:
            return self.save_as()
        return self._save_to(self.semantic_path)

    def save_as(self):
        if self.read_only or self.model_scene is None:
            return False
        initial = self.semantic_path or Path("semantic_map.geojson")
        path, _ = QFileDialog.getSaveFileName(
            self, "保存语义地图", str(initial), "GeoJSON (*.geojson)"
        )
        if not path:
            return False
        if not path.endswith(".geojson"):
            path += ".geojson"
        return self._save_to(Path(path))

    def _save_to(self, semantic_path):
        if self.read_only or self.model_scene is None:
            return False
        semantic_path = Path(semantic_path).expanduser().resolve()
        coverage_path = semantic_path.with_name("coverage.yaml")
        coverage = deepcopy(self.coverage)
        coverage.base_map = os.path.relpath(self.map_path, coverage_path.parent)
        coverage.base_map_sha256 = sha256_file(self.map_path)
        context = self._validation_context()
        report = validate_task(
            self.model_scene.semantic_map,
            coverage,
            context=context,
        )
        if not report.valid:
            self._refresh_validation(report)
            QMessageBox.warning(
                self,
                "任务不合法，禁止保存",
                "\n".join(
                    f"{issue.code} [{issue.object_id}]" for issue in report.issues
                ),
            )
            return False
        try:
            save_semantic_task(
                self.model_scene.semantic_map,
                coverage,
                semantic_path,
                coverage_path,
                validation_context=context,
            )
        except (OSError, SemanticFileError) as exc:
            QMessageBox.critical(self, "保存失败", str(exc))
            return False
        self.coverage = coverage
        self.semantic_path = semantic_path
        self.coverage_path = coverage_path
        self.model_scene.mark_saved()
        self._update_title()
        self.statusBar().showMessage(f"已保存 {semantic_path}", 5000)
        return True

    def reload(self):
        if not self._confirm_discard_changes():
            return
        try:
            if self.semantic_path and self.semantic_path.is_file():
                self.load_task(self.semantic_path)
            elif self.map_path:
                self.load_base_map(self.map_path, self.platform_profile_path)
        except Exception as exc:
            QMessageBox.critical(self, "重新加载失败", str(exc))

    def fit_map(self):
        if self.map_pixmap is not None:
            self.view.fitInView(
                self.graphics_scene.sceneRect(), Qt.KeepAspectRatio
            )

    def update_cursor_position(self, scene_point):
        if self.transformer is None:
            return
        try:
            world_x, world_y = self.transformer.scene_to_world(
                scene_point.x(), scene_point.y()
            )
        except ValueError:
            return
        self.statusBar().showMessage(
            f"map: x={world_x:.3f} m, y={world_y:.3f} m · 中键平移 · 滚轮缩放"
        )

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.graphics_scene.cancel_drawing()
            return
        if event.key() in {Qt.Key_Return, Qt.Key_Enter}:
            self.graphics_scene.finish_drawing()
            return
        super().keyPressEvent(event)

    def closeEvent(self, event):
        if self._confirm_discard_changes():
            event.accept()
        else:
            event.ignore()

    def _confirm_discard_changes(self):
        if self.model_scene is None or not self.model_scene.dirty:
            return True
        result = QMessageBox.question(
            self,
            "存在未保存修改",
            "语义任务尚未保存。是否先保存？",
            QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if result == QMessageBox.Save:
            return bool(self.save())
        return result == QMessageBox.Discard

    def _update_title(self):
        suffix = " [只读]" if self.read_only else ""
        map_id = self.model_scene.semantic_map.map_id if self.model_scene else "未加载"
        self.setWindowTitle(f"AGT 农业语义地图编辑器 · {map_id}{suffix}")


def default_config_path():
    try:
        from ament_index_python.packages import get_package_share_directory

        return str(
            Path(get_package_share_directory("agt_ui_bridge"))
            / "config"
            / "semantic_editor.yaml"
        )
    except (ImportError, LookupError):
        return None


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description="AGT semantic map editor")
    parser.add_argument("--map", default="")
    parser.add_argument("--semantic-map", default="")
    parser.add_argument("--platform-profile", default="")
    parser.add_argument("--config", default=default_config_path() or "")
    return parser.parse_known_args(argv)[0]


def main(argv=None):
    arguments = parse_arguments(argv)
    app = QApplication.instance() or QApplication(sys.argv)
    window = SemanticEditorWindow(
        defaults=EditorDefaults.from_yaml(arguments.config),
        platform_profile_path=arguments.platform_profile or None,
        map_path=arguments.map or None,
        semantic_path=arguments.semantic_map or None,
    )
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
