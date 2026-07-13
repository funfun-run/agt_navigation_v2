#!/usr/bin/env python3

import sys
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.srv import LoadMap, SaveMap
from nav_msgs.msg import OccupancyGrid
from PyQt5.QtCore import QPoint, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QImage, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication,
    QButtonGroup,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QRadioButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


MAP_QOS = QoSProfile(depth=1)
MAP_QOS.reliability = ReliabilityPolicy.RELIABLE
MAP_QOS.durability = DurabilityPolicy.TRANSIENT_LOCAL


class MapCanvas(QWidget):
    map_changed = pyqtSignal()
    pose_clicked = pyqtSignal(float, float, str)

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumSize(720, 520)
        self.setMouseTracking(True)
        self.grid = None
        self.info = None
        self.tool = "occupied"
        self.brush_size = 3
        self.click_mode = "edit"
        self._scale = 1.0
        self._offset = QPoint(0, 0)

    def set_map(self, msg: OccupancyGrid) -> None:
        self.info = msg.info
        self.grid = np.asarray(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        ).copy()
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#18201e"))
        if self.grid is None:
            painter.setPen(QColor("#d7ded8"))
            painter.drawText(self.rect(), Qt.AlignCenter, "等待地图或点击“加载地图”")
            return
        display = np.flipud(self.grid)
        rgb = np.empty((*display.shape, 3), dtype=np.uint8)
        rgb[display < 0] = (104, 112, 108)
        rgb[display == 0] = (238, 239, 225)
        rgb[display > 0] = (33, 39, 37)
        image = QImage(
            rgb.data, rgb.shape[1], rgb.shape[0], 3 * rgb.shape[1], QImage.Format_RGB888
        ).copy()
        self._scale = min(self.width() / image.width(), self.height() / image.height())
        width = max(1, int(image.width() * self._scale))
        height = max(1, int(image.height() * self._scale))
        self._offset = QPoint((self.width() - width) // 2, (self.height() - height) // 2)
        painter.drawImage(
            self._offset.x(), self._offset.y(), image.scaled(width, height, Qt.IgnoreAspectRatio)
        )
        painter.setPen(QPen(QColor("#e7a84b"), 2))
        painter.drawRect(self._offset.x(), self._offset.y(), width, height)

    def _cell_at(self, point: QPoint):
        if self.grid is None:
            return None
        x = int((point.x() - self._offset.x()) / self._scale)
        display_y = int((point.y() - self._offset.y()) / self._scale)
        y = self.grid.shape[0] - 1 - display_y
        if 0 <= x < self.grid.shape[1] and 0 <= y < self.grid.shape[0]:
            return x, y
        return None

    def _world_at(self, x: int, y: int):
        yaw = 2.0 * np.arctan2(
            self.info.origin.orientation.z, self.info.origin.orientation.w
        )
        local_x = (x + 0.5) * self.info.resolution
        local_y = (y + 0.5) * self.info.resolution
        world_x = self.info.origin.position.x + np.cos(yaw) * local_x - np.sin(yaw) * local_y
        world_y = self.info.origin.position.y + np.sin(yaw) * local_x + np.cos(yaw) * local_y
        return float(world_x), float(world_y)

    def _apply_brush(self, point: QPoint) -> None:
        cell = self._cell_at(point)
        if cell is None:
            return
        x, y = cell
        radius = max(0, self.brush_size - 1)
        x0, x1 = max(0, x - radius), min(self.grid.shape[1], x + radius + 1)
        y0, y1 = max(0, y - radius), min(self.grid.shape[0], y + radius + 1)
        value = {"occupied": 100, "free": 0, "unknown": -1}[self.tool]
        self.grid[y0:y1, x0:x1] = value
        self.map_changed.emit()
        self.update()

    def mousePressEvent(self, event) -> None:
        if event.button() != Qt.LeftButton:
            return
        cell = self._cell_at(event.pos())
        if cell is None:
            return
        if self.click_mode == "edit":
            self._apply_brush(event.pos())
        else:
            world = self._world_at(*cell)
            self.pose_clicked.emit(world[0], world[1], self.click_mode)
            self.click_mode = "edit"

    def mouseMoveEvent(self, event) -> None:
        if self.click_mode == "edit" and event.buttons() & Qt.LeftButton:
            self._apply_brush(event.pos())


class MapEditorNode(Node):
    def __init__(self) -> None:
        super().__init__("agt_qt5_map_editor")
        self.latest_map = None
        self.map_dirty = False
        self.freeze_source = False
        self.create_subscription(
            OccupancyGrid, "/agt/map/edited", self._edited_map_callback, MAP_QOS
        )
        self.create_subscription(
            OccupancyGrid, "/agt/map/global_occupancy", self._source_map_callback, MAP_QOS
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/agt/map/edited", MAP_QOS)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.load_client = self.create_client(LoadMap, "/agt/map/load")
        self.save_client = self.create_client(SaveMap, "/agt/map/save")

    def _source_map_callback(self, msg: OccupancyGrid) -> None:
        if self.freeze_source:
            return
        self.latest_map = msg
        self.map_dirty = True

    def _edited_map_callback(self, msg: OccupancyGrid) -> None:
        self.freeze_source = True
        self.latest_map = msg
        self.map_dirty = True

    def publish_grid(self, grid: np.ndarray, template: OccupancyGrid) -> None:
        msg = OccupancyGrid()
        msg.header = template.header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = template.info
        msg.data = grid.astype(np.int8).reshape(-1).tolist()
        self.latest_map = msg
        self.freeze_source = True
        self.map_pub.publish(msg)

    def publish_pose(self, x: float, y: float, mode: str) -> None:
        if mode == "initialpose":
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.orientation.w = 1.0
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.0685
            self.initial_pose_pub.publish(msg)
        else:
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.orientation.w = 1.0
            self.goal_pub.publish(msg)


class MapEditorWindow(QMainWindow):
    def __init__(self, node: MapEditorNode) -> None:
        super().__init__()
        self.node = node
        self.current_map = None
        self.pending = None
        self.setWindowTitle("AGT 二维地图编辑器")
        self.resize(1100, 760)
        self.canvas = MapCanvas()
        self.canvas.map_changed.connect(self._mark_edited)
        self.canvas.pose_clicked.connect(self._publish_pose)

        root = QWidget()
        layout = QVBoxLayout(root)
        controls = QHBoxLayout()
        load_btn = QPushButton("加载地图")
        save_btn = QPushButton("保存地图")
        publish_btn = QPushButton("发布编辑地图")
        initial_btn = QPushButton("设置初始位姿")
        goal_btn = QPushButton("设置导航目标")
        load_btn.clicked.connect(self._load)
        save_btn.clicked.connect(self._save)
        publish_btn.clicked.connect(self._publish)
        initial_btn.clicked.connect(lambda: self._set_click_mode("initialpose"))
        goal_btn.clicked.connect(lambda: self._set_click_mode("goal"))
        for button in (load_btn, save_btn, publish_btn, initial_btn, goal_btn):
            controls.addWidget(button)
        controls.addStretch(1)

        tools = QHBoxLayout()
        group = QButtonGroup(self)
        for label, value in (("障碍", "occupied"), ("自由", "free"), ("未知", "unknown")):
            radio = QRadioButton(label)
            radio.toggled.connect(
                lambda checked, tool=value: checked and setattr(self.canvas, "tool", tool)
            )
            group.addButton(radio)
            tools.addWidget(radio)
            if value == "occupied":
                radio.setChecked(True)
        tools.addWidget(QLabel("画笔格数"))
        brush = QSpinBox()
        brush.setRange(1, 30)
        brush.setValue(3)
        brush.valueChanged.connect(lambda value: setattr(self.canvas, "brush_size", value))
        tools.addWidget(brush)
        tools.addStretch(1)

        self.status = QLabel("等待 /agt/map/global_occupancy")
        layout.addLayout(controls)
        layout.addLayout(tools)
        layout.addWidget(self.canvas, 1)
        layout.addWidget(self.status)
        self.setCentralWidget(root)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(20)

    def _mark_edited(self) -> None:
        self.node.freeze_source = True
        self.status.setText("地图已修改，尚未发布")

    def _tick(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)
        if self.node.map_dirty:
            self.node.map_dirty = False
            self.current_map = self.node.latest_map
            self.canvas.set_map(self.current_map)
            self.status.setText(
                f"地图 {self.current_map.info.width} x {self.current_map.info.height}，"
                f"分辨率 {self.current_map.info.resolution:.3f} m"
            )
        if self.pending is not None and self.pending.done():
            operation, future = self.pending
            self.pending = None
            try:
                result = future.result()
                ok = (
                    result.result == LoadMap.Response.RESULT_SUCCESS
                    if operation == "load"
                    else result.result
                )
                self.status.setText(f"{operation} {'成功' if ok else '失败'}")
            except Exception as exc:
                QMessageBox.critical(self, "地图操作失败", str(exc))

    def _load(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "加载 Nav2 地图", "runtime/maps", "YAML (*.yaml)")
        if not path:
            return
        if not self.node.load_client.service_is_ready():
            QMessageBox.warning(self, "服务不可用", "请先启动 agt_ui_bridge map_io_bridge。")
            return
        request = LoadMap.Request()
        request.map_url = str(Path(path).resolve())
        self.pending = ("load", self.node.load_client.call_async(request))

    def _save(self) -> None:
        self._publish()
        path, _ = QFileDialog.getSaveFileName(
            self,
            "保存 Nav2 地图",
            "runtime/maps/edited_map.yaml",
            "YAML (*.yaml)",
        )
        if not path:
            return
        if not self.node.save_client.service_is_ready():
            QMessageBox.warning(self, "服务不可用", "请先启动 agt_ui_bridge map_io_bridge。")
            return
        request = SaveMap.Request()
        request.map_topic = "/agt/map/edited"
        request.map_url = str(Path(path).resolve())
        request.image_format = "pgm"
        request.map_mode = "trinary"
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65
        self.pending = ("save", self.node.save_client.call_async(request))

    def _publish(self) -> None:
        if self.current_map is None or self.canvas.grid is None:
            return
        self.node.publish_grid(self.canvas.grid, self.current_map)
        self.current_map = self.node.latest_map
        self.status.setText("编辑地图已发布到 /agt/map/edited")

    def _set_click_mode(self, mode: str) -> None:
        self.canvas.click_mode = mode
        self.status.setText("请在地图上单击位置；当前方向默认为 0 rad")

    def _publish_pose(self, x: float, y: float, mode: str) -> None:
        self.node.publish_pose(x, y, mode)
        self.status.setText(f"已发布 {mode}: x={x:.2f}, y={y:.2f}, yaw=0")


def main(args=None) -> None:
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    node = MapEditorNode()
    window = MapEditorWindow(node)
    window.show()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
