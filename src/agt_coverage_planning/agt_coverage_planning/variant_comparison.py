"""Configuration and metrics for visualization-only coverage variants."""

from dataclasses import dataclass
import math
from pathlib import Path
import re

from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union
import yaml


ALLOWED_ROUTE_MODES = {"BOUSTROPHEDON", "SNAKE", "SPIRAL"}
ALLOWED_PATH_MODES = {"DUBIN", "REEDS_SHEPP"}
IDENTIFIER = re.compile(r"^[a-z][a-z0-9_]*$")


class VariantComparisonError(ValueError):
    """Stable failure raised for invalid comparison configuration or geometry."""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = str(code)


@dataclass(frozen=True)
class CoverageVariant:
    variant_id: str
    label: str
    route_mode: str
    path_mode: str
    swath_angle_offset: float = 0.0
    spiral_n: int = 4

    def to_dict(self):
        return {
            "variant_id": self.variant_id,
            "label": self.label,
            "route_mode": self.route_mode,
            "path_mode": self.path_mode,
            "swath_angle_offset": _stable(self.swath_angle_offset),
            "spiral_n": self.spiral_n,
        }


def load_variants(path):
    """Load and validate a deterministic list of route candidates."""
    source = Path(path).expanduser().resolve()
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    entries = document.get("variants") if isinstance(document, dict) else None
    if not isinstance(entries, list) or len(entries) < 2:
        raise VariantComparisonError(
            "insufficient_variants", "comparison requires at least two variants"
        )

    variants = []
    identifiers = set()
    for index, entry in enumerate(entries):
        if not isinstance(entry, dict):
            raise VariantComparisonError(
                "invalid_variant", f"variant {index} must be a mapping"
            )
        variant_id = str(entry.get("id", "")).strip()
        if not IDENTIFIER.fullmatch(variant_id) or variant_id in identifiers:
            raise VariantComparisonError(
                "invalid_variant_id", f"invalid or duplicate variant id: {variant_id}"
            )
        route_mode = str(entry.get("route_mode", "")).strip().upper()
        path_mode = str(entry.get("path_mode", "")).strip().upper()
        if route_mode not in ALLOWED_ROUTE_MODES:
            raise VariantComparisonError(
                "unsupported_route_mode", f"unsupported route mode: {route_mode}"
            )
        if path_mode not in ALLOWED_PATH_MODES:
            raise VariantComparisonError(
                "unsupported_path_mode", f"unsupported path mode: {path_mode}"
            )
        try:
            offset = math.radians(float(entry.get("swath_angle_offset_deg", 0.0)))
            spiral_n = int(entry.get("spiral_n", 4))
        except (TypeError, ValueError) as exc:
            raise VariantComparisonError(
                "invalid_variant_value", f"variant {variant_id} has invalid values"
            ) from exc
        if not math.isfinite(offset) or abs(offset) > math.pi / 2.0:
            raise VariantComparisonError(
                "invalid_swath_angle_offset",
                f"variant {variant_id} angle offset must be within +/-90 degrees",
            )
        if spiral_n <= 0:
            raise VariantComparisonError(
                "invalid_spiral_n", f"variant {variant_id} spiral_n must be positive"
            )
        identifiers.add(variant_id)
        variants.append(
            CoverageVariant(
                variant_id=variant_id,
                label=str(entry.get("label", variant_id)).strip() or variant_id,
                route_mode=route_mode,
                path_mode=path_mode,
                swath_angle_offset=offset,
                spiral_n=spiral_n,
            )
        )
    return tuple(variants)


def coverage_area_metrics(field_ring, exclusion_rings, swaths, operation_width):
    """Compute area metrics only from complete, authoritative SWATH geometry."""
    width = float(operation_width)
    if not math.isfinite(width) or width <= 0.0:
        raise VariantComparisonError(
            "invalid_operation_width", "operation width must be positive"
        )
    target = Polygon(field_ring)
    if not target.is_valid or target.area <= 0.0:
        raise VariantComparisonError("invalid_field", "field polygon is invalid")
    exclusions = [Polygon(ring) for ring in exclusion_rings]
    if any(not polygon.is_valid for polygon in exclusions):
        raise VariantComparisonError(
            "invalid_exclusion", "exclusion polygon is invalid"
        )
    if exclusions:
        target = target.difference(unary_union(exclusions))
    if target.is_empty or target.area <= 0.0:
        raise VariantComparisonError("empty_target_area", "workable field is empty")

    strips = []
    for index, (start, end) in enumerate(swaths):
        line = LineString((start, end))
        if line.length <= 1e-9:
            raise VariantComparisonError(
                "zero_length_swath", f"swath {index} has zero length"
            )
        strip = line.buffer(width / 2.0, cap_style=2, join_style=2).intersection(target)
        if not strip.is_empty:
            strips.append(strip)
    if not strips:
        raise VariantComparisonError("empty_swath_coverage", "swaths cover no field area")

    covered = unary_union(strips)
    overlap_area = max(0.0, sum(strip.area for strip in strips) - covered.area)
    return {
        "target_area": _stable(target.area),
        "covered_area": _stable(covered.area),
        "missed_area": _stable(max(0.0, target.area - covered.area)),
        "overlap_area": _stable(overlap_area),
        "coverage_rate": _stable(min(1.0, covered.area / target.area)),
        "overlap_rate": _stable(overlap_area / target.area),
    }


def rank_candidates(candidates):
    """Rank geometric candidates while keeping semantic readiness explicit."""
    ranked = sorted(
        (
            candidate
            for candidate in candidates
            if candidate.get("status") == "SUCCEEDED"
            and candidate.get("estimated_motion_time") is not None
        ),
        key=lambda item: (
            float(item["estimated_motion_time"]),
            float(item["total_path_length"]),
            str(item["variant_id"]),
        ),
    )
    for rank, candidate in enumerate(ranked, start=1):
        candidate["geometric_rank"] = rank
    return tuple(candidate["variant_id"] for candidate in ranked)


def _stable(value):
    return round(float(value), 9)
