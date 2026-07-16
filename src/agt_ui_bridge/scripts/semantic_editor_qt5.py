#!/usr/bin/env python3

import argparse
from copy import deepcopy
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
import secrets
import sys
import tempfile

import numpy as np
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Path as NavPath
from PyQt5.QtCore import QPoint, QPointF, QProcess, QProcessEnvironment, QTimer, Qt
from PyQt5.QtGui import QColor, QImage, QPainter, QPainterPath, QPen, QPixmap
from PyQt5.QtWidgets import (
    QAction,
    QActionGroup,
    QApplication,
    QCheckBox,
    QComboBox,
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
    QLabel,
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
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import yaml
from shapely.geometry import Polygon
from shapely.validation import explain_validity

from agt_ui_bridge.map_image import (
    FREE_PIXEL,
    OCCUPIED_PIXEL,
    UNKNOWN_PIXEL,
    load_nav2_map_image,
    save_nav2_map_image,
)
from agt_ui_bridge.coverage_preview import (
    CoveragePreviewError,
    derive_inter_row_aisles,
)
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
    "field_boundary": QColor("#007a3d"),
    "exclusion_zone": QColor("#d00000"),
    "row_centerline": QColor("#8a5800"),
    "entry_pose": QColor("#0057d9"),
    "work_direction": QColor("#007c83"),
    "headland_zone": QColor("#a44800"),
    "keepout_zone": QColor("#a00050"),
}
DRAW_TO_FEATURE = {
    "field_boundary": "field_boundary",
    "exclusion_zone": "exclusion_zone",
    "row_centerline": "row_centerline",
    "entry_pose": "entry_pose",
    "work_direction": "work_direction",
}
MAP_EDIT_TO_PIXEL = {
    "map_occupied": OCCUPIED_PIXEL,
    "map_free": FREE_PIXEL,
    "map_unknown": UNKNOWN_PIXEL,
}
LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


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
    MIN_SCALE = 0.0001
    MAX_SCALE = 100.0

    def __init__(self, scene, parent=None):
        super().__init__(scene, parent)
        self.setRenderHints(self.renderHints() | QPainter.Antialiasing)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setMouseTracking(True)
        self._panning = False
        self._pan_start = QPoint()

    def wheelEvent(self, event):
        if self.zoom_by_wheel_delta(event.angleDelta().y()):
            event.accept()
        else:
            event.ignore()

    def zoom_by_wheel_delta(self, delta):
        if delta == 0:
            return False
        current_scale = abs(self.transform().m11())
        if not math.isfinite(current_scale) or current_scale <= 0.0:
            return False

        factor = 1.15 if delta > 0 else 1.0 / 1.15
        next_scale = current_scale * factor
        if delta < 0 and next_scale < self.MIN_SCALE:
            if current_scale <= self.MIN_SCALE:
                return False
            factor = self.MIN_SCALE / current_scale
        elif delta > 0 and next_scale > self.MAX_SCALE:
            if current_scale >= self.MAX_SCALE:
                return False
            factor = self.MAX_SCALE / current_scale

        self.scale(factor, factor)
        return True

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


class ContrastPathItem(QGraphicsPathItem):
    def paint(self, painter, option, widget=None):
        halo_pen = QPen(self.pen())
        halo_pen.setColor(QColor(255, 255, 255, 220))
        halo_pen.setWidthF(self.pen().widthF() + 3.0)
        painter.save()
        painter.setPen(halo_pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawPath(self.path())
        painter.restore()
        super().paint(painter, option, widget)


class FeatureGraphicsItem(ContrastPathItem):
    def __init__(self, feature_id, feature_type, path, invalid=False):
        super().__init__(path)
        self.feature_id = feature_id
        color = QColor("#ff3b30") if invalid else FEATURE_COLORS[feature_type]
        pen = QPen(color, 3.2)
        pen.setCosmetic(True)
        if invalid:
            pen.setStyle(Qt.DashLine)
        self.setPen(pen)
        fill = QColor(color)
        is_polygon = feature_type.endswith("zone") or feature_type == "field_boundary"
        fill.setAlpha(60 if is_polygon else 0)
        self.setBrush(fill)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setZValue(10)
        self.setToolTip(f"{feature_id} · {FEATURE_LABELS[feature_type]}")


class VertexHandle(QGraphicsEllipseItem):
    def __init__(self, feature_id, vertex_key, position, bounds, callback):
        super().__init__(-5.5, -5.5, 11.0, 11.0)
        self.feature_id = feature_id
        self.vertex_key = vertex_key
        self._bounds = bounds
        self._callback = callback
        self._start_position = QPointF(position)
        self.setPos(position)
        self.setBrush(QColor("#ffd60a"))
        pen = QPen(QColor("#111111"), 2.0)
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
            feature_id = self.feature_id
            vertex_key = self.vertex_key
            position = QPointF(self.pos())
            callback = self._callback
            QTimer.singleShot(
                0, lambda: callback(feature_id, vertex_key, position)
            )


class SemanticGraphicsScene(QGraphicsScene):
    def __init__(self, editor):
        super().__init__(editor)
        self.editor = editor
        self.draw_points = []
        self.preview_item = None
        self.map_stroke_start = None
        self.map_stroke_last = None

    def cancel_drawing(self):
        self.draw_points.clear()
        self.map_stroke_start = None
        self.map_stroke_last = None
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
        if self.editor.tool in {"field_boundary", "exclusion_zone"}:
            polygon = Polygon([(point.x(), point.y()) for point in self.draw_points])
            if not polygon.is_valid:
                QMessageBox.warning(
                    self.editor,
                    "区域边界无效",
                    f"边界发生交叉，尚未完成：{explain_validity(polygon)}\n"
                    "按 Backspace 撤回最后一个点后继续绘制，或按 Esc 取消。",
                )
                return
        points = list(self.draw_points)
        self.cancel_drawing()
        self.editor.finish_feature_from_scene(self.editor.tool, points)

    def remove_last_draw_point(self):
        if not self.draw_points:
            return False
        self.draw_points.pop()
        if self.draw_points:
            self._update_preview()
        elif self.preview_item is not None:
            self.removeItem(self.preview_item)
            self.preview_item = None
        self.editor.statusBar().showMessage("已撤回草稿最后一个点", 2000)
        return True

    def mousePressEvent(self, event):
        if self.editor.tool in MAP_EDIT_TO_PIXEL and not self.editor.read_only:
            if event.button() == Qt.LeftButton:
                point = event.scenePos()
                if self.sceneRect().contains(point):
                    self.map_stroke_start = QPointF(point)
                    self.map_stroke_last = QPointF(point)
                    self.editor.begin_map_edit()
                    if self.editor.map_draw_mode == "brush":
                        self.editor.apply_map_brush(point, refresh=False)
                        self.editor._refresh_map_item()
                    else:
                        self._update_map_line_preview(point)
                    event.accept()
                    return
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
        if (
            self.editor.tool in MAP_EDIT_TO_PIXEL
            and not self.editor.read_only
            and event.buttons() & Qt.LeftButton
        ):
            point = event.scenePos()
            if self.sceneRect().contains(point):
                if self.editor.map_draw_mode == "brush":
                    start = self.map_stroke_last or point
                    self.editor.apply_map_line(start, point, refresh=False)
                    self.map_stroke_last = QPointF(point)
                    self.editor._refresh_map_item()
                else:
                    self._update_map_line_preview(point)
                event.accept()
                return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if (
            self.editor.tool in MAP_EDIT_TO_PIXEL
            and not self.editor.read_only
            and event.button() == Qt.LeftButton
            and self.map_stroke_start is not None
        ):
            point = event.scenePos()
            if self.editor.map_draw_mode == "line":
                self.editor.apply_map_line(self.map_stroke_start, point)
            else:
                self.editor._refresh_map_item()
            self.map_stroke_start = None
            self.map_stroke_last = None
            self.editor.commit_map_edit()
            if self.preview_item is not None:
                self.removeItem(self.preview_item)
                self.preview_item = None
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def _update_map_line_preview(self, point):
        if self.preview_item is not None:
            self.removeItem(self.preview_item)
        path = QPainterPath(self.map_stroke_start)
        path.lineTo(point)
        self.preview_item = QGraphicsPathItem(path)
        pixel = MAP_EDIT_TO_PIXEL[self.editor.tool]
        color = QColor(pixel, pixel, pixel)
        pen = QPen(color, float(self.editor.map_brush_size), Qt.DashLine)
        pen.setCosmetic(True)
        self.preview_item.setPen(pen)
        self.preview_item.setZValue(50)
        self.addItem(self.preview_item)

    def _update_preview(self):
        if self.preview_item is not None:
            self.removeItem(self.preview_item)
        path = QPainterPath()
        path.moveTo(self.draw_points[0])
        for point in self.draw_points[1:]:
            path.lineTo(point)
        self.preview_item = ContrastPathItem(path)
        preview_color = FEATURE_COLORS[DRAW_TO_FEATURE[self.editor.tool]]
        pen = QPen(preview_color, 3.0, Qt.DashLine)
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
        self._map_item = None
        self.map_array = None
        self.map_metadata = None
        self.map_dirty = False
        self.map_brush_size = 3
        self.map_draw_mode = "brush"
        self.selected_vertex = None
        self._map_undo_stack = []
        self._map_redo_stack = []
        self._map_edit_before = None
        self._edit_timeline = []
        self._redo_timeline = []
        self.read_only = False
        self.tool = "select"
        self.selected_feature_id = None
        self._rendering = False
        self._refresh_pending = False
        self._feature_items = {}
        self._layer_checks = {}
        self._tool_actions = {}
        self._fit_after_first_show = True
        self._preview_process = None
        self._preview_tempdir = None
        self._preview_world_points = []
        self._preview_path_summary = {}
        self._preview_status = "尚未生成路线"
        self._preview_run_active = False
        self._preview_accept_after_ns = 0
        self._preview_has_current_path = False
        self._preview_domain_id = None
        self._preview_context = None
        self._preview_node = None
        self._preview_spin_timer = None

        self.setWindowTitle("AGT 农业语义地图编辑器")
        self.resize(1420, 880)
        self.graphics_scene = SemanticGraphicsScene(self)
        self.view = SemanticGraphicsView(self.graphics_scene, self)
        self._build_ui()
        self._init_preview_ros()
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
        redo_action.setShortcuts(["Ctrl+Shift+Z", "Ctrl+Y"])
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

        map_toolbar = self.addToolBar("底图编辑")
        for tool, label in (
            ("map_occupied", "地图障碍"),
            ("map_free", "地图自由"),
            ("map_unknown", "地图未知"),
        ):
            action = QAction(label, self)
            action.setCheckable(True)
            action.triggered.connect(
                lambda checked, selected=tool: checked and self.set_tool(selected)
            )
            tool_group.addAction(action)
            map_toolbar.addAction(action)
            self._tool_actions[tool] = action
        map_toolbar.addSeparator()
        draw_mode_group = QActionGroup(self)
        draw_mode_group.setExclusive(True)
        for mode, label in (("brush", "自由画笔"), ("line", "画直线")):
            action = QAction(label, self)
            action.setCheckable(True)
            action.setChecked(mode == self.map_draw_mode)
            action.triggered.connect(
                lambda checked, selected=mode: checked
                and self.set_map_draw_mode(selected)
            )
            draw_mode_group.addAction(action)
            map_toolbar.addAction(action)
        map_toolbar.addSeparator()
        grow_brush = QAction("画笔+", self)
        grow_brush.triggered.connect(lambda: self._change_map_brush(1))
        shrink_brush = QAction("画笔-", self)
        shrink_brush.triggered.connect(lambda: self._change_map_brush(-1))
        map_toolbar.addActions([grow_brush, shrink_brush])

        self._build_object_dock()
        self._build_layer_dock()
        self._build_validation_dock()
        self._build_preview_dock()

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
        preview_check = QCheckBox("路线预览")
        preview_check.setChecked(True)
        preview_check.toggled.connect(self.refresh_scene)
        self._layer_checks["route_preview"] = preview_check
        layout.addWidget(preview_check)
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

    def _build_preview_dock(self):
        dock = QDockWidget("路线预览", self)
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.addWidget(QLabel("输入语义"))
        self.preview_mode_combo = QComboBox()
        self.preview_mode_combo.addItem("作物行间道路（推荐）", "inter_row_aisles")
        self.preview_mode_combo.addItem("标注线就是道路", "annotated_rows")
        self.preview_mode_combo.addItem("区域自动覆盖", "polygon")
        layout.addWidget(self.preview_mode_combo)
        layout.addWidget(QLabel("连接模型"))
        self.preview_path_combo = QComboBox()
        self.preview_path_combo.addItem("Reeds-Shepp（允许倒车）", "reeds_shepp")
        self.preview_path_combo.addItem("Dubins（仅前进）", "dubins")
        layout.addWidget(self.preview_path_combo)
        buttons = QHBoxLayout()
        generate_button = QPushButton("生成路线")
        generate_button.clicked.connect(self.start_coverage_preview)
        stop_button = QPushButton("停止预览")
        stop_button.clicked.connect(self.stop_coverage_preview)
        buttons.addWidget(generate_button)
        buttons.addWidget(stop_button)
        layout.addLayout(buttons)
        self.preview_info = QLabel(self._preview_status)
        self.preview_info.setWordWrap(True)
        self.preview_info.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.preview_info)
        dock.setWidget(panel)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)

    def _init_preview_ros(self):
        if not rclpy.ok():
            return
        # Offline planning is isolated from stale preview nodes and the live robot graph.
        self._preview_domain_id = 20 + secrets.randbelow(180)
        self._preview_context = rclpy.Context()
        self._preview_context.init(domain_id=self._preview_domain_id)
        self._preview_node = rclpy.create_node(
            "agt_semantic_editor_preview", context=self._preview_context
        )
        self._preview_node.create_subscription(
            NavPath, "/agt/coverage/path_preview", self._preview_path_callback, LATCHED_QOS
        )
        self._preview_node.create_subscription(
            DiagnosticArray,
            "/agt/coverage/status",
            self._preview_status_callback,
            LATCHED_QOS,
        )
        self._preview_node.create_subscription(
            String,
            "/agt/coverage/simulation_report",
            self._preview_report_callback,
            LATCHED_QOS,
        )
        self._preview_spin_timer = QTimer(self)
        self._preview_spin_timer.timeout.connect(
            lambda: rclpy.spin_once(self._preview_node, timeout_sec=0.0)
        )
        self._preview_spin_timer.start(30)

    def start_coverage_preview(self):
        if self.model_scene is None or self.map_path is None:
            QMessageBox.warning(self, "无法预览", "请先加载并保存语义任务")
            return False
        if self.model_scene.dirty or self.map_dirty:
            if not self.save():
                return False
        self.stop_coverage_preview(clear_route=True)
        try:
            semantic_map, coverage = self._preview_task()
            self._preview_tempdir = tempfile.TemporaryDirectory(
                prefix="agt_editor_coverage_preview_"
            )
            root = Path(self._preview_tempdir.name)
            semantic_path = root / "semantic_map.geojson"
            coverage_path = root / "coverage.yaml"
            coverage.base_map = str(self.map_path)
            coverage.base_map_sha256 = sha256_file(self.map_path)
            save_semantic_task(
                semantic_map,
                coverage,
                semantic_path,
                coverage_path,
                validation_context=self._validation_context(),
            )
        except (CoveragePreviewError, OSError, SemanticFileError, ValueError) as exc:
            QMessageBox.warning(self, "预览任务无效", str(exc))
            self._cleanup_preview_tempdir()
            return False

        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        environment = QProcessEnvironment.systemEnvironment()
        environment.insert("ROS_DOMAIN_ID", str(self._preview_domain_id))
        process.setProcessEnvironment(environment)
        process.readyReadStandardOutput.connect(
            lambda active=process: self._preview_process_output(active)
        )
        process.finished.connect(
            lambda exit_code, exit_status, active=process: (
                self._preview_process_finished(active, exit_code, exit_status)
            )
        )
        self._preview_process = process
        report_path = Path(self._preview_tempdir.name) / "simulation_report.json"
        arguments = [
            "launch",
            "agt_coverage_planning",
            "coverage_preview.launch.py",
            f"map:={self.map_path}",
            f"semantic_map:={semantic_path}",
            f"platform_profile:={self.platform_profile_path}",
            "start_rviz:=false",
            f"simulation_report_path:={report_path}",
        ]
        self._preview_status = "正在启动离线规划（执行始终关闭）..."
        self._preview_run_active = True
        self._preview_has_current_path = False
        self._preview_accept_after_ns = (
            self._preview_node.get_clock().now().nanoseconds
            if self._preview_node is not None
            else 0
        )
        self._update_preview_info()
        self._preview_process.start("ros2", arguments)
        if not self._preview_process.waitForStarted(3000):
            self._preview_run_active = False
            self._preview_status = (
                "规划进程启动失败。请先 source 覆盖依赖工作区和本仓库 install。"
            )
            self._update_preview_info()
            return False
        return True

    def _preview_task(self):
        semantic_map = deepcopy(self.model_scene.semantic_map)
        coverage = deepcopy(self.coverage)
        mode = self.preview_mode_combo.currentData()
        if mode == "inter_row_aisles":
            semantic_map = derive_inter_row_aisles(semantic_map)
            coverage.planning_mode = "annotated_rows"
            coverage.row_interpretation = "direct_swaths"
        elif mode == "annotated_rows":
            coverage.planning_mode = "annotated_rows"
            coverage.row_interpretation = "direct_swaths"
        else:
            coverage.planning_mode = "polygon"
        coverage.allow_reverse = self.preview_path_combo.currentData() == "reeds_shepp"
        return semantic_map, coverage

    def stop_coverage_preview(self, clear_route=False):
        self._preview_run_active = False
        self._preview_has_current_path = False
        process = self._preview_process
        self._preview_process = None
        if process is not None and process.state() != QProcess.NotRunning:
            process.terminate()
            if not process.waitForFinished(8000):
                process.kill()
                process.waitForFinished(2000)
        self._cleanup_preview_tempdir()
        if clear_route:
            self._preview_world_points = []
            self._preview_path_summary = {}
            self.refresh_scene()
        self._preview_status = "预览已停止" if process is not None else self._preview_status
        self._update_preview_info()

    def _cleanup_preview_tempdir(self):
        if self._preview_tempdir is not None:
            self._preview_tempdir.cleanup()
            self._preview_tempdir = None

    def _preview_process_output(self, process):
        if process is not self._preview_process:
            return
        text = bytes(process.readAllStandardOutput()).decode(
            "utf-8", errors="replace"
        )
        if "Package 'agt_coverage_planning' not found" in text:
            self._preview_status = "找不到覆盖规划包，请 source agt_coverage_ws/install/setup.bash"
            self._update_preview_info()

    def _preview_process_finished(self, process, exit_code, _exit_status):
        if process is not self._preview_process:
            return
        self._preview_run_active = False
        if exit_code != 0 and not self._preview_world_points:
            self._preview_status = f"规划进程已退出（code={exit_code}）"
        self._update_preview_info()

    def _preview_message_is_current(self, message):
        if not self._preview_run_active:
            return False
        header = getattr(message, "header", None)
        stamp = getattr(header, "stamp", None)
        if stamp is None or self._preview_accept_after_ns <= 0:
            return True
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        return stamp_ns >= self._preview_accept_after_ns

    def _preview_path_callback(self, message):
        if not self._preview_message_is_current(message):
            return
        points = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in message.poses
        ]
        self._preview_world_points = points
        self._preview_has_current_path = bool(points)
        if not points:
            for key in (
                "路径点",
                "长度",
                "预计时间",
                "预计转弯",
                "倒车距离",
            ):
                self._preview_path_summary.pop(key, None)
            self.refresh_scene()
            self._update_preview_info()
            return
        length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        self._preview_path_summary.update(
            {"路径点": len(points), "长度": f"{length:.2f} m"}
        )
        self.refresh_scene()
        self._update_preview_info()

    def _preview_status_callback(self, message):
        if not self._preview_message_is_current(message):
            return
        status = next(
            (item for item in message.status if item.name == "agt_coverage_request_adapter"),
            None,
        )
        if status is None:
            return
        values = {item.key: item.value for item in status.values}
        state = status.message
        error_code = values.get("error_code")
        detail = values.get("detail", "")
        if state == "WAITING_FOR_SERVER":
            self._preview_status = "WAITING_FOR_SERVER（服务器启动中，正在自动重试）"
            self._preview_path_summary["等待原因"] = error_code or "server_unavailable"
            if detail:
                self._preview_path_summary["详情"] = detail
            self._preview_path_summary.pop("错误码", None)
        else:
            self._preview_status = state
            self._preview_path_summary.pop("等待原因", None)
            if error_code not in {None, "none"}:
                self._preview_path_summary["错误码"] = error_code
            else:
                self._preview_path_summary.pop("错误码", None)
            if state in {"REJECTED", "FAILED"} and detail:
                self._preview_path_summary["详情"] = detail
            else:
                self._preview_path_summary.pop("详情", None)
        self._update_preview_info()

    def _preview_report_callback(self, message):
        # String has no header, so only accept metrics after this run's fresh path.
        if not self._preview_run_active or not self._preview_has_current_path:
            return
        try:
            report = json.loads(message.data)
        except (TypeError, ValueError):
            return
        for key, label, suffix in (
            ("estimated_motion_time", "预计时间", " s"),
            ("estimated_turn_count", "预计转弯", ""),
            ("reverse_path_length", "倒车距离", " m"),
        ):
            value = report.get(key)
            if value is not None:
                self._preview_path_summary[label] = f"{float(value):.2f}{suffix}"
        self._update_preview_info()

    def _update_preview_info(self):
        if not hasattr(self, "preview_info"):
            return
        mode = self.preview_mode_combo.currentText()
        model = self.preview_path_combo.currentText()
        details = "\n".join(
            f"{key}: {value}" for key, value in self._preview_path_summary.items()
        )
        text = f"{mode}\n{model}\n状态: {self._preview_status}"
        if details:
            text += "\n" + details
        self.preview_info.setText(text)

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
        loaded_map = load_nav2_map_image(self.map_path)
        self.map_metadata = loaded_map.metadata
        self.map_array = loaded_map.image
        self.map_dirty = False
        self._map_undo_stack.clear()
        self._map_redo_stack.clear()
        self._map_edit_before = None
        self._edit_timeline.clear()
        self._redo_timeline.clear()
        self._update_map_pixmap()
        self.graphics_scene.setSceneRect(
            0.0, 0.0, float(self.map_array.shape[1]), float(self.map_array.shape[0])
        )

    def _update_map_pixmap(self):
        if self.map_array is None:
            self.map_pixmap = None
            return
        data = self.map_array.tobytes("C")
        qimage = QImage(
            data,
            self.map_array.shape[1],
            self.map_array.shape[0],
            self.map_array.shape[1],
            QImage.Format_Grayscale8,
        ).copy()
        self.map_pixmap = QPixmap.fromImage(qimage)

    def _refresh_map_item(self):
        if self._map_item is not None and self.map_pixmap is not None:
            self._map_item.setPixmap(self.map_pixmap)

    def set_tool(self, tool):
        if self.read_only and tool != "select":
            self._tool_actions["select"].setChecked(True)
            return
        if self.graphics_scene.draw_points and tool != self.tool:
            result = QMessageBox.question(
                self,
                "放弃未完成绘制？",
                "切换工具会丢弃当前尚未完成的顶点。是否继续？",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if result != QMessageBox.Yes:
                self._tool_actions[self.tool].setChecked(True)
                return
        self.graphics_scene.cancel_drawing()
        self.tool = tool
        if tool != "select":
            self.selected_vertex = None
        self.refresh_scene()
        if tool == "select":
            self.statusBar().showMessage("选择对象或拖动顶点；中键平移，滚轮缩放")
        elif tool in MAP_EDIT_TO_PIXEL:
            self._show_map_tool_status()
        elif tool in {"field_boundary", "exclusion_zone", "row_centerline"}:
            self.statusBar().showMessage("左键添加顶点，双击/右键/Enter 完成，Esc 取消")
        else:
            self.statusBar().showMessage("点击位置，再点击方向点")

    def _change_map_brush(self, delta):
        self.map_brush_size = max(1, min(30, self.map_brush_size + delta))
        if self.tool in MAP_EDIT_TO_PIXEL:
            self._show_map_tool_status(2000)

    def set_map_draw_mode(self, mode):
        if mode not in {"brush", "line"}:
            raise ValueError(f"unsupported map draw mode: {mode}")
        self.graphics_scene.cancel_drawing()
        self.map_draw_mode = mode
        if self.tool in MAP_EDIT_TO_PIXEL:
            self._show_map_tool_status()

    def _show_map_tool_status(self, timeout=0):
        operation = "拖动连续绘制" if self.map_draw_mode == "brush" else "拖动预览，松开画直线"
        self.statusBar().showMessage(
            f"底图：{operation}；宽度 {self.map_brush_size} 像素", timeout
        )

    def apply_map_brush(self, scene_point, refresh=True):
        if self.map_array is None or self.tool not in MAP_EDIT_TO_PIXEL:
            return
        image_x = int(scene_point.x())
        image_y = int(scene_point.y())
        if not self.transformer.contains_image(image_x, image_y):
            return
        before = (self.map_brush_size - 1) // 2
        after = self.map_brush_size // 2
        x0 = max(0, image_x - before)
        x1 = min(self.map_array.shape[1], image_x + after + 1)
        y0 = max(0, image_y - before)
        y1 = min(self.map_array.shape[0], image_y + after + 1)
        new_pixel = MAP_EDIT_TO_PIXEL[self.tool]
        patch = self.map_array[y0:y1, x0:x1]
        if np.all(patch == new_pixel):
            return
        self.map_array[y0:y1, x0:x1] = new_pixel
        self.map_dirty = True
        self._update_map_pixmap()
        if refresh:
            self._refresh_map_item()
        self._update_title()

    def apply_map_line(self, start, end, refresh=True):
        delta_x = end.x() - start.x()
        delta_y = end.y() - start.y()
        steps = max(1, int(math.ceil(max(abs(delta_x), abs(delta_y)))))
        for index in range(steps + 1):
            ratio = index / steps
            self.apply_map_brush(
                QPointF(start.x() + delta_x * ratio, start.y() + delta_y * ratio),
                refresh=False,
            )
        if refresh:
            self._refresh_map_item()

    def begin_map_edit(self):
        if self.map_array is not None and self._map_edit_before is None:
            self._map_edit_before = self.map_array.copy()

    def commit_map_edit(self):
        if self._map_edit_before is None or self.map_array is None:
            return False
        before = self._map_edit_before
        self._map_edit_before = None
        if np.array_equal(before, self.map_array):
            return False
        self._map_undo_stack.append(before)
        if len(self._map_undo_stack) > self.defaults.history_limit:
            self._map_undo_stack.pop(0)
        self._map_redo_stack.clear()
        self._record_edit_domain("map")
        return True

    def _record_edit_domain(self, domain):
        self._edit_timeline.append(domain)
        if len(self._edit_timeline) > self.defaults.history_limit:
            self._edit_timeline.pop(0)
        self._redo_timeline.clear()

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
        self._record_edit_domain("semantic")
        self.selected_feature_id = feature.id
        self._tool_actions["select"].setChecked(True)
        if self.tool != "select":
            self.set_tool("select")
        else:
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
            self._map_item = None
            self.graphics_scene.preview_item = None
            self.graphics_scene.draw_points.clear()
            self._feature_items = {}
            if self.map_pixmap is not None and self._layer_visible("base_map"):
                map_item = QGraphicsPixmapItem(self.map_pixmap)
                map_item.setZValue(-100)
                self.graphics_scene.addItem(map_item)
                self._map_item = map_item
            self._render_route_preview()
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

    def _schedule_scene_refresh(self):
        if self._refresh_pending:
            return
        self._refresh_pending = True

        def refresh_after_event():
            self._refresh_pending = False
            self.refresh_scene()

        QTimer.singleShot(0, refresh_after_event)

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
            if self.selected_vertex == (feature.id, key):
                handle.setSelected(True)
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
        self._record_edit_domain("semantic")
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
            item = ContrastPathItem(path)
            pen = QPen(QColor("#17202a"), 2.0, Qt.DotLine)
            pen.setCosmetic(True)
            item.setPen(pen)
            item.setZValue(5)
            self.graphics_scene.addItem(item)

    def _render_route_preview(self):
        if len(self._preview_world_points) < 2 or not self._layer_visible(
            "route_preview"
        ):
            return
        path = QPainterPath(self._world_point(self._preview_world_points[0]))
        for point in self._preview_world_points[1:]:
            path.lineTo(self._world_point(point))
        item = ContrastPathItem(path)
        pen = QPen(QColor("#ff3b30"), 3.0)
        pen.setCosmetic(True)
        item.setPen(pen)
        item.setZValue(45)
        item.setToolTip("离线覆盖路线预览：不可执行")
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
        selected_handles = [
            item
            for item in self.graphics_scene.selectedItems()
            if isinstance(item, VertexHandle)
        ]
        selected = [
            item
            for item in self.graphics_scene.selectedItems()
            if isinstance(item, FeatureGraphicsItem)
        ]
        if selected_handles:
            handle = selected_handles[0]
            self.selected_feature_id = handle.feature_id
            self.selected_vertex = (handle.feature_id, handle.vertex_key)
            self.model_scene.selected_feature_id = self.selected_feature_id
            self._refresh_tree()
            return
        else:
            self.selected_feature_id = selected[0].feature_id if selected else None
            self.selected_vertex = None
        self.model_scene.selected_feature_id = self.selected_feature_id
        # Never clear the scene while Qt is still dispatching an item event.
        self._schedule_scene_refresh()

    def _tree_selection_changed(self):
        if self._rendering:
            return
        items = self.object_tree.selectedItems()
        self.selected_feature_id = items[0].data(0, Qt.UserRole) if items else None
        self.selected_vertex = None
        self.model_scene.selected_feature_id = self.selected_feature_id
        self._schedule_scene_refresh()

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
        if (
            not self.read_only
            and not self.model_scene.features
            and all(issue.code == "missing_feature_type" for issue in report.issues)
        ):
            self.validation_list.addItem(
                "开始标注 · 请依次绘制作业区、内部障碍、入口位姿和作业方向"
            )
            return
        for issue in report.issues:
            if issue.code == "missing_feature_type":
                label = FEATURE_LABELS.get(issue.object_id, issue.object_id)
                self.validation_list.addItem(
                    f"待绘制 · {label}（保存前必需）"
                )
                continue
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
        self._record_edit_domain("semantic")
        self.selected_feature_id = updated.id
        self.refresh_scene()

    def delete_selected(self):
        if self.read_only or not self.selected_feature_id:
            return
        self.model_scene.remove(self.selected_feature_id)
        self._record_edit_domain("semantic")
        self.selected_feature_id = None
        self.refresh_scene()

    def undo(self):
        if self.read_only:
            return
        domain = self._edit_timeline.pop() if self._edit_timeline else "semantic"
        if domain == "map" and self._map_undo_stack:
            self._map_redo_stack.append(self.map_array.copy())
            self.map_array = self._map_undo_stack.pop()
            self.map_dirty = True
            self._update_map_pixmap()
            self._refresh_map_item()
            self._redo_timeline.append("map")
            self._update_title()
            self.statusBar().showMessage("已撤销底图笔划", 2000)
        elif self.model_scene and self.model_scene.undo():
            self.selected_feature_id = self.model_scene.selected_feature_id
            self.selected_vertex = None
            self.refresh_scene()
            self._redo_timeline.append("semantic")
        else:
            if domain != "semantic":
                self._edit_timeline.append(domain)
            self.statusBar().showMessage("没有可撤销的操作", 2000)

    def redo(self):
        if self.read_only:
            return
        domain = self._redo_timeline.pop() if self._redo_timeline else "semantic"
        if domain == "map" and self._map_redo_stack:
            self._map_undo_stack.append(self.map_array.copy())
            self.map_array = self._map_redo_stack.pop()
            self.map_dirty = True
            self._update_map_pixmap()
            self._refresh_map_item()
            self._edit_timeline.append("map")
            self._update_title()
            self.statusBar().showMessage("已重做底图笔划", 2000)
        elif self.model_scene and self.model_scene.redo():
            self.selected_feature_id = self.model_scene.selected_feature_id
            self.selected_vertex = None
            self.refresh_scene()
            self._edit_timeline.append("semantic")
        else:
            if domain != "semantic":
                self._redo_timeline.append(domain)
            self.statusBar().showMessage("没有可重做的操作", 2000)

    def save(self):
        if self.map_dirty and not self._save_map_in_place():
            return False
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

    def _save_map_in_place(self):
        if self.map_array is None or self.map_path is None or self.map_metadata is None:
            return False
        try:
            save_nav2_map_image(self.map_path, self.map_metadata, self.map_array)
        except OSError as exc:
            QMessageBox.critical(self, "底图保存失败", str(exc))
            return False
        self.map_dirty = False
        self._update_title()
        self.statusBar().showMessage(f"已保存底图 {self.map_path}", 5000)
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

    def showEvent(self, event):
        super().showEvent(event)
        if self._fit_after_first_show:
            self._fit_after_first_show = False
            QTimer.singleShot(0, self.fit_map)

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
        if event.key() == Qt.Key_Backspace:
            if self.graphics_scene.remove_last_draw_point():
                return
        direction = {
            Qt.Key_Left: (-1.0, 0.0),
            Qt.Key_Right: (1.0, 0.0),
            Qt.Key_Up: (0.0, -1.0),
            Qt.Key_Down: (0.0, 1.0),
        }.get(event.key())
        if direction and self._nudge_selected_vertex(*direction, fine=bool(event.modifiers() & Qt.ShiftModifier)):
            return
        super().keyPressEvent(event)

    def _nudge_selected_vertex(self, delta_x, delta_y, fine=False):
        if self.read_only or self.tool != "select" or self.selected_vertex is None:
            return False
        feature_id, vertex_key = self.selected_vertex
        feature = self.model_scene.get(feature_id)
        if feature is None:
            return False
        if vertex_key == "yaw":
            yaw = float(feature.properties.get("yaw", 0.0))
            coordinate = [
                feature.coordinates[0] + 0.8 * math.cos(yaw),
                feature.coordinates[1] + 0.8 * math.sin(yaw),
            ]
        elif feature.geometry_type == "Polygon":
            coordinate = feature.coordinates[0][vertex_key]
        elif feature.geometry_type == "LineString":
            coordinate = feature.coordinates[vertex_key]
        else:
            coordinate = feature.coordinates
        point = self._world_point(coordinate)
        scale = 0.2 if fine else 1.0
        moved = QPointF(point.x() + delta_x * scale, point.y() + delta_y * scale)
        if not self.graphics_scene.sceneRect().contains(moved):
            return False
        self._vertex_moved(feature_id, vertex_key, moved)
        self.selected_vertex = (feature_id, vertex_key)
        self.refresh_scene()
        return True

    def closeEvent(self, event):
        if self._confirm_discard_changes():
            self.stop_coverage_preview()
            if self._preview_spin_timer is not None:
                self._preview_spin_timer.stop()
            if self._preview_node is not None:
                self._preview_node.destroy_node()
                self._preview_node = None
            if self._preview_context is not None:
                self._preview_context.shutdown()
                self._preview_context = None
            event.accept()
        else:
            event.ignore()

    def _confirm_discard_changes(self):
        scene_dirty = self.model_scene is not None and self.model_scene.dirty
        if not scene_dirty and not self.map_dirty:
            return True
        result = QMessageBox.question(
            self,
            "存在未保存修改",
            "底图或语义任务尚未保存。是否先保存？",
            QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if result == QMessageBox.Save:
            return bool(self.save())
        return result == QMessageBox.Discard

    def _update_title(self):
        suffix = " [只读]" if self.read_only else ""
        if self.map_dirty or (self.model_scene is not None and self.model_scene.dirty):
            suffix += " *"
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
    if not rclpy.ok():
        rclpy.init(args=sys.argv)
    app = QApplication.instance() or QApplication(sys.argv)
    try:
        window = SemanticEditorWindow(
            defaults=EditorDefaults.from_yaml(arguments.config),
            platform_profile_path=arguments.platform_profile or None,
            map_path=arguments.map or None,
            semantic_path=arguments.semantic_map or None,
        )
        window.show()
        return app.exec_()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
