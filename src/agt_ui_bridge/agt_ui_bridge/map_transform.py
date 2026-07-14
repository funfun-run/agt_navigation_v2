"""Coordinate transforms for Nav2 maps without Qt or ROS dependencies."""

from dataclasses import dataclass
import math
from pathlib import Path

from PIL import Image
import yaml


@dataclass(frozen=True)
class MapGeometry:
    resolution: float
    width: int
    height: int
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_yaw: float = 0.0
    frame_id: str = "map"

    def __post_init__(self):
        if self.resolution <= 0.0:
            raise ValueError("map resolution must be positive")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("map width and height must be positive")
        if self.frame_id != "map":
            raise ValueError("semantic map geometry must use frame_id 'map'")
        values = (self.origin_x, self.origin_y, self.origin_yaw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("map origin must contain finite values")

    @classmethod
    def from_nav2_yaml(cls, yaml_path, frame_id="map"):
        yaml_path = Path(yaml_path)
        metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        image_path = Path(metadata["image"])
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        with Image.open(image_path) as image:
            width, height = image.size
        origin = metadata.get("origin", [0.0, 0.0, 0.0])
        if len(origin) != 3:
            raise ValueError("Nav2 map origin must contain x, y, and yaw")
        return cls(
            resolution=float(metadata["resolution"]),
            width=int(width),
            height=int(height),
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            origin_yaw=float(origin[2]),
            frame_id=frame_id,
        )


class MapTransform:
    """Convert between map grid, image, Qt scene, and map-world coordinates."""

    def __init__(
        self,
        geometry,
        scene_origin=(0.0, 0.0),
        scene_units_per_cell=1.0,
    ):
        if scene_units_per_cell <= 0.0:
            raise ValueError("scene_units_per_cell must be positive")
        self.geometry = geometry
        self.scene_origin_x = float(scene_origin[0])
        self.scene_origin_y = float(scene_origin[1])
        self.scene_units_per_cell = float(scene_units_per_cell)
        self._cos_yaw = math.cos(geometry.origin_yaw)
        self._sin_yaw = math.sin(geometry.origin_yaw)

    def contains_grid(self, grid_x, grid_y):
        return (
            0 <= grid_x < self.geometry.width
            and 0 <= grid_y < self.geometry.height
        )

    def grid_to_world(self, grid_x, grid_y, center=True):
        if not self.contains_grid(grid_x, grid_y):
            raise ValueError(f"grid cell outside map: ({grid_x}, {grid_y})")
        offset = 0.5 if center else 0.0
        local_x = (float(grid_x) + offset) * self.geometry.resolution
        local_y = (float(grid_y) + offset) * self.geometry.resolution
        return self._local_to_world(local_x, local_y)

    def world_to_grid(self, world_x, world_y):
        local_x, local_y = self._world_to_local(world_x, world_y)
        grid_x = math.floor(local_x / self.geometry.resolution)
        grid_y = math.floor(local_y / self.geometry.resolution)
        if not self.contains_grid(grid_x, grid_y):
            raise ValueError(f"world point outside map: ({world_x}, {world_y})")
        return grid_x, grid_y

    def grid_to_image_pixel(self, grid_x, grid_y, center=True):
        if not self.contains_grid(grid_x, grid_y):
            raise ValueError(f"grid cell outside map: ({grid_x}, {grid_y})")
        offset = 0.5 if center else 0.0
        image_x = float(grid_x) + offset
        image_y = self.geometry.height - float(grid_y) - offset
        return image_x, image_y

    def image_pixel_to_grid(self, image_x, image_y):
        if not self.contains_image(image_x, image_y):
            raise ValueError(f"image point outside map: ({image_x}, {image_y})")
        display_x = math.floor(image_x)
        display_y = math.floor(image_y)
        return display_x, self.geometry.height - 1 - display_y

    def contains_image(self, image_x, image_y):
        return (
            0.0 <= image_x < self.geometry.width
            and 0.0 <= image_y < self.geometry.height
        )

    def image_to_world(self, image_x, image_y):
        if not self.contains_image_extent(image_x, image_y):
            raise ValueError(f"image point outside map: ({image_x}, {image_y})")
        local_x = float(image_x) * self.geometry.resolution
        local_y = (
            self.geometry.height - float(image_y)
        ) * self.geometry.resolution
        return self._local_to_world(local_x, local_y)

    def world_to_image(self, world_x, world_y):
        image_x, image_y = self.world_to_image_unbounded(world_x, world_y)
        if not self.contains_image_extent(image_x, image_y):
            raise ValueError(f"world point outside map: ({world_x}, {world_y})")
        return image_x, image_y

    def world_to_image_unbounded(self, world_x, world_y):
        local_x, local_y = self._world_to_local(world_x, world_y)
        image_x = local_x / self.geometry.resolution
        image_y = self.geometry.height - local_y / self.geometry.resolution
        return image_x, image_y

    def contains_image_extent(self, image_x, image_y):
        return (
            0.0 <= image_x <= self.geometry.width
            and 0.0 <= image_y <= self.geometry.height
        )

    def scene_to_world(self, scene_x, scene_y):
        image_x = (
            float(scene_x) - self.scene_origin_x
        ) / self.scene_units_per_cell
        image_y = (
            float(scene_y) - self.scene_origin_y
        ) / self.scene_units_per_cell
        return self.image_to_world(image_x, image_y)

    def world_to_scene(self, world_x, world_y):
        image_x, image_y = self.world_to_image(world_x, world_y)
        return self._image_to_scene(image_x, image_y)

    def world_to_scene_unbounded(self, world_x, world_y):
        image_x, image_y = self.world_to_image_unbounded(world_x, world_y)
        return self._image_to_scene(image_x, image_y)

    def _image_to_scene(self, image_x, image_y):
        return (
            self.scene_origin_x + image_x * self.scene_units_per_cell,
            self.scene_origin_y + image_y * self.scene_units_per_cell,
        )

    def _local_to_world(self, local_x, local_y):
        return (
            self.geometry.origin_x
            + self._cos_yaw * local_x
            - self._sin_yaw * local_y,
            self.geometry.origin_y
            + self._sin_yaw * local_x
            + self._cos_yaw * local_y,
        )

    def _world_to_local(self, world_x, world_y):
        delta_x = float(world_x) - self.geometry.origin_x
        delta_y = float(world_y) - self.geometry.origin_y
        return (
            self._cos_yaw * delta_x + self._sin_yaw * delta_y,
            -self._sin_yaw * delta_x + self._cos_yaw * delta_y,
        )
