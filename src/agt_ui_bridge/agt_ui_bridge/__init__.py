"""UI-independent semantic map support for AGT tools."""

from .map_transform import MapGeometry, MapTransform
from .platform_profile import load_platform_profile, resolve_platform_profile
from .semantic_model import CoverageParameters, SemanticFeature, SemanticMap
from .semantic_rasterizer import RasterizedMask, rasterize_keepout_mask

__all__ = [
    "CoverageParameters",
    "MapGeometry",
    "MapTransform",
    "load_platform_profile",
    "resolve_platform_profile",
    "SemanticFeature",
    "SemanticMap",
    "RasterizedMask",
    "rasterize_keepout_mask",
]
