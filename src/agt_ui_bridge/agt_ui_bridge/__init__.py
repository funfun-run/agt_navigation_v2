"""UI-independent semantic map support for AGT tools."""

from .map_image import (
    FREE_PIXEL,
    OCCUPIED_PIXEL,
    UNKNOWN_PIXEL,
    LoadedNav2MapImage,
    grid_value_to_nav2_pixel,
    load_nav2_map_image,
    nav2_pixel_to_grid_value,
    save_nav2_map_image,
)
from .map_transform import MapGeometry, MapTransform
from .platform_profile import load_platform_profile, resolve_platform_profile
from .semantic_model import CoverageParameters, SemanticFeature, SemanticMap
from .semantic_rasterizer import RasterizedMask, rasterize_keepout_mask

__all__ = [
    "CoverageParameters",
    "FREE_PIXEL",
    "grid_value_to_nav2_pixel",
    "LoadedNav2MapImage",
    "MapGeometry",
    "MapTransform",
    "nav2_pixel_to_grid_value",
    "load_nav2_map_image",
    "OCCUPIED_PIXEL",
    "load_platform_profile",
    "resolve_platform_profile",
    "save_nav2_map_image",
    "SemanticFeature",
    "SemanticMap",
    "RasterizedMask",
    "rasterize_keepout_mask",
    "UNKNOWN_PIXEL",
]
