"""Semantic task validation shared by file I/O and the Qt editor."""

from dataclasses import dataclass, field
import hashlib
import math
from pathlib import Path
import re

from shapely.geometry import LineString, Point, Polygon
from shapely.validation import explain_validity


FEATURE_GEOMETRY = {
    "field_boundary": "Polygon",
    "exclusion_zone": "Polygon",
    "entry_pose": "Point",
    "work_direction": "LineString",
    "row_centerline": "LineString",
    "headland_zone": "Polygon",
    "keepout_zone": "Polygon",
}
REQUIRED_FEATURE_COUNTS = {
    "field_boundary": 1,
    "exclusion_zone": 1,
    "entry_pose": 1,
    "work_direction": 1,
}
FEATURE_ID = re.compile(r"^[a-z][a-z0-9_]*$")
GEOMETRY_EPSILON = 1e-9


@dataclass(frozen=True)
class ValidationIssue:
    code: str
    message: str
    severity: str = "ERROR"
    object_id: str = "<document>"


@dataclass
class ValidationReport:
    issues: list = field(default_factory=list)

    @property
    def valid(self):
        return not any(issue.severity == "ERROR" for issue in self.issues)

    def add(self, code, message, object_id="<document>", severity="ERROR"):
        self.issues.append(
            ValidationIssue(code, message, severity=severity, object_id=object_id)
        )


@dataclass(frozen=True)
class ValidationContext:
    """Optional map and vehicle context for complete spatial validation."""

    map_geometry: object = None
    navigation_footprint: tuple = ()
    minimum_boundary_clearance: float = 0.0
    base_map_path: object = None


def validate_semantic_map(semantic_map, context=None):
    report = ValidationReport()
    if semantic_map.schema_version != "1.0":
        report.add(
            "unsupported_schema_version",
            f"unsupported schema version: {semantic_map.schema_version}",
        )
    if semantic_map.frame_id != "map":
        report.add("invalid_frame", "semantic map frame_id must be map")
    if not semantic_map.map_id:
        report.add("missing_map_id", "semantic map map_id is required")

    seen_ids = set()
    enabled_counts = {feature_type: 0 for feature_type in FEATURE_GEOMETRY}
    geometries = {}
    for feature in semantic_map.features:
        object_id = feature.id or "<missing>"
        if feature.id in seen_ids:
            report.add(
                "duplicate_feature_id",
                f"duplicate feature id: {feature.id}",
                object_id,
            )
        seen_ids.add(feature.id)
        if not FEATURE_ID.fullmatch(feature.id):
            report.add(
                "invalid_feature_id",
                "feature id must use lowercase snake_case",
                object_id,
            )
        if feature.frame_id != "map":
            report.add(
                "invalid_feature_frame",
                "feature frame_id must be map",
                object_id,
            )
        if not isinstance(feature.enabled, bool):
            report.add(
                "invalid_enabled",
                "feature enabled must be boolean",
                object_id,
            )
        expected_geometry = FEATURE_GEOMETRY.get(feature.feature_type)
        if expected_geometry is None:
            report.add(
                "unknown_feature_type",
                f"unknown feature type: {feature.feature_type}",
                object_id,
            )
            continue
        if feature.enabled:
            enabled_counts[feature.feature_type] += 1
        if feature.geometry_type != expected_geometry:
            report.add(
                "invalid_geometry_type",
                f"{feature.feature_type} requires {expected_geometry}",
                object_id,
            )
            continue
        if _validate_geometry(feature, report):
            geometry = _to_shapely(feature)
            geometries[feature.id] = geometry
            if feature.geometry_type == "Polygon" and not geometry.is_valid:
                reason = explain_validity(geometry)
                report.add(
                    (
                        "polygon_self_intersection"
                        if "Self-intersection" in reason
                        else "invalid_polygon_topology"
                    ),
                    reason,
                    object_id,
                )

    for feature_type, minimum_count in REQUIRED_FEATURE_COUNTS.items():
        if enabled_counts[feature_type] < minimum_count:
            report.add(
                "missing_feature_type",
                f"at least {minimum_count} enabled {feature_type} is required",
                feature_type,
            )

    _validate_spatial_relationships(semantic_map, geometries, context, report)
    return report


def validate_task(semantic_map, coverage, context=None):
    report = validate_semantic_map(semantic_map, context=context)
    if coverage.schema_version != "1.0":
        report.add(
            "unsupported_coverage_schema",
            f"unsupported coverage schema: {coverage.schema_version}",
        )
    if coverage.map_id != semantic_map.map_id:
        report.add("map_id_mismatch", "semantic map and coverage map_id differ")
    if coverage.frame_id != "map" or coverage.frame_id != semantic_map.frame_id:
        report.add("coverage_frame_mismatch", "coverage frame_id must be map")
    if coverage.planning_mode not in {"polygon", "annotated_rows"}:
        report.add(
            "invalid_planning_mode",
            f"unsupported planning mode: {coverage.planning_mode}",
        )
    if not isinstance(coverage.allow_reverse, bool):
        report.add("invalid_allow_reverse", "allow_reverse must be boolean")
    for field_name in ("robot_width", "operation_width"):
        value = getattr(coverage, field_name)
        if not math.isfinite(value) or value <= 0.0:
            report.add("invalid_coverage_value", f"{field_name} must be positive")
    for field_name in ("min_turning_radius", "headland_width"):
        value = getattr(coverage, field_name)
        if not math.isfinite(value) or value < 0.0:
            report.add(
                "invalid_coverage_value", f"{field_name} must be non-negative"
            )
    if not math.isfinite(coverage.preferred_swath_angle):
        report.add(
            "invalid_coverage_value", "preferred_swath_angle must be finite"
        )
    _validate_base_map_hash(coverage, context, report)
    return report


def _validate_geometry(feature, report):
    object_id = feature.id or "<missing>"
    coordinates = feature.coordinates
    if feature.geometry_type == "Point":
        if not _is_point(coordinates):
            report.add("invalid_point", "Point requires two finite values", object_id)
            return False
        yaw = feature.properties.get("yaw")
        if not _is_finite_number(yaw):
            report.add(
                "invalid_entry_yaw", "entry_pose requires finite yaw", object_id
            )
            return False
    elif feature.geometry_type == "LineString":
        if not isinstance(coordinates, list) or len(coordinates) < 2:
            report.add(
                "line_too_short", "LineString requires at least two points", object_id
            )
            return False
        if not all(_is_point(point) for point in coordinates):
            report.add(
                "invalid_line_coordinate",
                "LineString coordinates must be finite points",
                object_id,
            )
            return False
        if feature.feature_type == "work_direction":
            start = coordinates[0]
            end = coordinates[-1]
            if math.hypot(end[0] - start[0], end[1] - start[1]) <= GEOMETRY_EPSILON:
                report.add(
                    "zero_length_work_direction",
                    "work_direction endpoints must differ",
                    object_id,
                )
                return False
    elif feature.geometry_type == "Polygon":
        if not isinstance(coordinates, list) or not coordinates:
            report.add("invalid_polygon", "Polygon requires an outer ring", object_id)
            return False
        ring = coordinates[0]
        if not isinstance(ring, list) or not all(_is_point(point) for point in ring):
            report.add(
                "invalid_polygon_coordinate",
                "Polygon ring must contain finite points",
                object_id,
            )
            return False
        valid = True
        if len(ring) < 4 or ring[0] != ring[-1]:
            report.add(
                "polygon_not_closed",
                "Polygon ring must be closed with at least three vertices",
                object_id,
            )
            valid = False
        vertices = ring[:-1] if ring and ring[0] == ring[-1] else ring
        if len({tuple(point) for point in vertices}) < 3:
            report.add(
                "polygon_too_small",
                "Polygon requires at least three unique vertices",
                object_id,
            )
            valid = False
        return valid
    return True


def _validate_spatial_relationships(semantic_map, geometries, context, report):
    enabled = [feature for feature in semantic_map.features if feature.enabled]
    fields = _valid_polygons(enabled, geometries, "field_boundary")
    exclusions = _valid_polygons(enabled, geometries, "exclusion_zone")

    for feature, exclusion in exclusions:
        containing_fields = [field for _, field in fields if field.covers(exclusion)]
        if not containing_fields:
            report.add(
                "exclusion_outside_field",
                "exclusion_zone must be contained by an enabled field_boundary",
                feature.id,
            )

    for feature in enabled:
        geometry = geometries.get(feature.id)
        if geometry is None or feature.feature_type != "entry_pose":
            continue
        containing_fields = [field for _, field in fields if field.covers(geometry)]
        if not containing_fields:
            report.add(
                "entry_outside_field",
                "entry_pose must be inside an enabled field_boundary",
                feature.id,
            )
        if any(exclusion.covers(geometry) for _, exclusion in exclusions):
            report.add(
                "entry_inside_exclusion",
                "entry_pose must not be inside an exclusion_zone",
                feature.id,
            )

    if context is None:
        return
    _validate_context(context, report)
    map_polygon = _map_polygon(context.map_geometry)
    if map_polygon is not None:
        for feature in enabled:
            geometry = geometries.get(feature.id)
            if geometry is not None and not map_polygon.covers(geometry):
                report.add(
                    "coordinate_outside_map",
                    "all feature coordinates must be inside the base map extent",
                    feature.id,
                )
    _validate_entry_footprints(enabled, geometries, fields, exclusions, context, report)


def _validate_entry_footprints(enabled, geometries, fields, exclusions, context, report):
    if not context.navigation_footprint:
        return
    template = Polygon(context.navigation_footprint)
    if not template.is_valid or template.is_empty or template.area <= GEOMETRY_EPSILON:
        report.add(
            "invalid_platform_footprint",
            f"platform navigation footprint is invalid: {explain_validity(template)}",
        )
        return

    clearance = float(context.minimum_boundary_clearance)
    for feature in enabled:
        if feature.feature_type != "entry_pose" or feature.id not in geometries:
            continue
        transformed = _transform_footprint(
            context.navigation_footprint,
            feature.coordinates,
            feature.properties["yaw"],
        )
        containing_fields = [field for _, field in fields if field.covers(transformed)]
        if not containing_fields or any(
            transformed.intersects(field.boundary) for field in containing_fields
        ):
            report.add(
                "entry_footprint_outside_field",
                "navigation footprint at entry_pose must fit strictly inside a field",
                feature.id,
            )
        if any(transformed.intersects(zone) for _, zone in exclusions):
            report.add(
                "entry_footprint_intersects_exclusion",
                "navigation footprint at entry_pose intersects an exclusion_zone",
                feature.id,
            )
        if containing_fields and clearance > 0.0:
            field_clearance = max(
                transformed.distance(field.boundary) for field in containing_fields
            )
            obstacle_clearance = min(
                (transformed.distance(zone) for _, zone in exclusions),
                default=math.inf,
            )
            actual = min(field_clearance, obstacle_clearance)
            if actual + GEOMETRY_EPSILON < clearance:
                report.add(
                    "insufficient_boundary_clearance",
                    (
                        f"entry footprint boundary clearance {actual:.3f} m "
                        f"is below {clearance:.3f} m"
                    ),
                    feature.id,
                )


def _valid_polygons(features, geometries, feature_type):
    return [
        (feature, geometries[feature.id])
        for feature in features
        if feature.feature_type == feature_type
        and feature.id in geometries
        and geometries[feature.id].is_valid
    ]


def _to_shapely(feature):
    if feature.geometry_type == "Point":
        return Point(feature.coordinates)
    if feature.geometry_type == "LineString":
        return LineString(feature.coordinates)
    return Polygon(feature.coordinates[0], feature.coordinates[1:])


def _map_polygon(geometry):
    if geometry is None:
        return None
    width = geometry.width * geometry.resolution
    height = geometry.height * geometry.resolution
    cos_yaw = math.cos(geometry.origin_yaw)
    sin_yaw = math.sin(geometry.origin_yaw)

    def world(local_x, local_y):
        return (
            geometry.origin_x + cos_yaw * local_x - sin_yaw * local_y,
            geometry.origin_y + sin_yaw * local_x + cos_yaw * local_y,
        )

    return Polygon(
        [world(0.0, 0.0), world(width, 0.0), world(width, height), world(0.0, height)]
    )


def _transform_footprint(footprint, position, yaw):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return Polygon(
        [
            (
                position[0] + cos_yaw * local_x - sin_yaw * local_y,
                position[1] + sin_yaw * local_x + cos_yaw * local_y,
            )
            for local_x, local_y in footprint
        ]
    )


def _validate_context(context, report):
    if (
        not math.isfinite(context.minimum_boundary_clearance)
        or context.minimum_boundary_clearance < 0.0
    ):
        report.add(
            "invalid_boundary_clearance",
            "minimum boundary clearance must be finite and non-negative",
        )


def _validate_base_map_hash(coverage, context, report):
    if context is None or context.base_map_path is None:
        return
    path = Path(context.base_map_path)
    if not path.is_file():
        report.add("base_map_missing", "base map YAML does not exist")
        return
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != coverage.base_map_sha256:
        report.add(
            "base_map_hash_mismatch",
            "base map SHA256 does not match coverage.yaml",
        )


def _is_point(value):
    return (
        isinstance(value, list)
        and len(value) == 2
        and all(_is_finite_number(component) for component in value)
    )


def _is_finite_number(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )
