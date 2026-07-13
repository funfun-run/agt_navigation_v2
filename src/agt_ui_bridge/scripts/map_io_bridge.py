#!/usr/bin/env python3

from pathlib import Path
from urllib.parse import unquote, urlparse

import numpy as np
import rclpy
import yaml
from nav2_msgs.srv import LoadMap, SaveMap
from nav_msgs.msg import OccupancyGrid
from PIL import Image
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


MAP_QOS = QoSProfile(depth=1)
MAP_QOS.reliability = ReliabilityPolicy.RELIABLE
MAP_QOS.durability = DurabilityPolicy.TRANSIENT_LOCAL


def local_path(url: str) -> Path:
    parsed = urlparse(url)
    if parsed.scheme == "file":
        return Path(unquote(parsed.path)).expanduser()
    if parsed.scheme:
        raise ValueError(f"unsupported map URL scheme: {parsed.scheme}")
    return Path(url).expanduser()


def quaternion_to_yaw(z: float, w: float) -> float:
    return float(np.arctan2(2.0 * w * z, 1.0 - 2.0 * z * z))


def load_occupancy_map(yaml_path: Path, frame_id: str) -> OccupancyGrid:
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    pixels = np.asarray(Image.open(image_path).convert("L"), dtype=np.float32)
    negate = bool(metadata.get("negate", 0))
    probability = pixels / 255.0 if negate else (255.0 - pixels) / 255.0
    occupied_thresh = float(metadata.get("occupied_thresh", 0.65))
    free_thresh = float(metadata.get("free_thresh", 0.25))
    values = np.full(probability.shape, -1, dtype=np.int8)
    values[probability >= occupied_thresh] = 100
    values[probability <= free_thresh] = 0
    # Nav2/map_saver uses 205 as the canonical unknown pixel in trinary maps.
    values[pixels == 205] = -1
    values = np.flipud(values)

    origin = metadata.get("origin", [0.0, 0.0, 0.0])
    yaw = float(origin[2])
    msg = OccupancyGrid()
    msg.header.frame_id = frame_id
    msg.info.resolution = float(metadata["resolution"])
    msg.info.width = int(values.shape[1])
    msg.info.height = int(values.shape[0])
    msg.info.origin.position.x = float(origin[0])
    msg.info.origin.position.y = float(origin[1])
    msg.info.origin.orientation.z = float(np.sin(yaw * 0.5))
    msg.info.origin.orientation.w = float(np.cos(yaw * 0.5))
    msg.data = values.reshape(-1).tolist()
    return msg


def save_occupancy_map(
    msg: OccupancyGrid,
    map_url: str,
    image_format: str,
    free_thresh: float,
    occupied_thresh: float,
) -> Path:
    destination = local_path(map_url).resolve()
    image_format = (image_format or "pgm").lower()
    if image_format not in {"pgm", "png", "bmp"}:
        raise ValueError(f"unsupported image format: {image_format}")
    yaml_path = (
        destination
        if destination.suffix.lower() == ".yaml"
        else destination.with_suffix(".yaml")
    )
    image_path = yaml_path.with_suffix(f".{image_format}")
    yaml_path.parent.mkdir(parents=True, exist_ok=True)

    values = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    pixels = np.full(values.shape, 205, dtype=np.uint8)
    pixels[values == 0] = 254
    pixels[values >= 100] = 0
    Image.fromarray(np.flipud(pixels), mode="L").save(image_path)

    origin = msg.info.origin
    metadata = {
        "image": image_path.name,
        "mode": "trinary",
        "resolution": float(msg.info.resolution),
        "origin": [
            float(origin.position.x),
            float(origin.position.y),
            quaternion_to_yaw(origin.orientation.z, origin.orientation.w),
        ],
        "negate": 0,
        "occupied_thresh": float(occupied_thresh),
        "free_thresh": float(free_thresh),
    }
    yaml_path.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")
    return yaml_path


class MapIoBridge(Node):
    def __init__(self) -> None:
        super().__init__("agt_map_io_bridge")
        self.declare_parameter("source_map_topic", "/agt/map/global_occupancy")
        self.declare_parameter("edited_map_topic", "/agt/map/edited")
        self.declare_parameter("frame_id", "map")
        self._latest_map = None
        self._has_edited_map = False
        source_topic = str(self.get_parameter("source_map_topic").value)
        edited_topic = str(self.get_parameter("edited_map_topic").value)
        self._publisher = self.create_publisher(OccupancyGrid, edited_topic, MAP_QOS)
        self.create_subscription(OccupancyGrid, source_topic, self._receive_source_map, MAP_QOS)
        self.create_subscription(OccupancyGrid, edited_topic, self._receive_edited_map, MAP_QOS)
        self.create_service(LoadMap, "/agt/map/load", self._load)
        self.create_service(SaveMap, "/agt/map/save", self._save)
        self.get_logger().info(
            f"map I/O ready: source={source_topic} edited={edited_topic}"
        )

    def _receive_source_map(self, msg: OccupancyGrid) -> None:
        if not self._has_edited_map:
            self._latest_map = msg

    def _receive_edited_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg
        self._has_edited_map = True

    def _load(self, request: LoadMap.Request, response: LoadMap.Response):
        try:
            loaded = load_occupancy_map(
                local_path(request.map_url).resolve(),
                str(self.get_parameter("frame_id").value),
            )
            loaded.header.stamp = self.get_clock().now().to_msg()
            loaded.info.map_load_time = loaded.header.stamp
            self._latest_map = loaded
            self._has_edited_map = True
            self._publisher.publish(loaded)
            response.map = loaded
            response.result = LoadMap.Response.RESULT_SUCCESS
        except FileNotFoundError as exc:
            self.get_logger().error(str(exc))
            response.result = LoadMap.Response.RESULT_MAP_DOES_NOT_EXIST
        except (KeyError, TypeError, ValueError, yaml.YAMLError, OSError) as exc:
            self.get_logger().error(f"failed to load map: {exc}")
            response.result = LoadMap.Response.RESULT_INVALID_MAP_DATA
        return response

    def _save(self, request: SaveMap.Request, response: SaveMap.Response):
        if self._latest_map is None:
            self.get_logger().error("cannot save before a map has been received")
            response.result = False
            return response
        try:
            free_thresh = request.free_thresh if request.free_thresh > 0.0 else 0.25
            occupied_thresh = (
                request.occupied_thresh if request.occupied_thresh > 0.0 else 0.65
            )
            path = save_occupancy_map(
                self._latest_map,
                request.map_url,
                request.image_format,
                free_thresh,
                occupied_thresh,
            )
            self.get_logger().info(f"saved edited map to {path}")
            response.result = True
        except (ValueError, OSError) as exc:
            self.get_logger().error(f"failed to save map: {exc}")
            response.result = False
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapIoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
