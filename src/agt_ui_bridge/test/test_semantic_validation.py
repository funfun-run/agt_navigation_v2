from copy import deepcopy
import json
from pathlib import Path
import sys

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from agt_ui_bridge.semantic_model import SemanticMap  # noqa: E402
from agt_ui_bridge.map_transform import MapGeometry  # noqa: E402
from agt_ui_bridge.semantic_validation import (  # noqa: E402
    FEATURE_GEOMETRY,
    REQUIRED_FEATURE_COUNTS,
    ValidationContext,
    validate_semantic_map,
)


VALID_MAP = (
    PACKAGE_ROOT.parents[1]
    / "docs/interfaces/examples/semantic_map/semantic/semantic_map.geojson"
)


def _valid_document():
    return json.loads(VALID_MAP.read_text(encoding="utf-8"))


FOOTPRINT = (
    (0.5915, 0.4690),
    (0.5915, -0.4690),
    (-0.5915, -0.4690),
    (-0.5915, 0.4690),
)


def _error_codes(document, context=None):
    report = validate_semantic_map(
        SemanticMap.from_geojson(document), context=context
    )
    return [issue.code for issue in report.issues]


def _feature(document, feature_type):
    return next(
        feature
        for feature in document["features"]
        if feature["properties"]["feature_type"] == feature_type
    )


def _context(clearance=0.0):
    return ValidationContext(
        map_geometry=MapGeometry(resolution=1.0, width=10, height=10),
        navigation_footprint=FOOTPRINT,
        minimum_boundary_clearance=clearance,
    )


def test_valid_example_has_no_structural_errors():
    report = validate_semantic_map(SemanticMap.from_geojson(_valid_document()))
    assert report.valid
    assert report.issues == []


def test_runtime_validation_matches_the_machine_readable_schema():
    schema = yaml.safe_load(
        (PACKAGE_ROOT / "config/semantic_schema.yaml").read_text(encoding="utf-8")
    )
    feature_types = schema["semantic_map"]["feature_types"]

    assert FEATURE_GEOMETRY == {
        name: contract["geometry"] for name, contract in feature_types.items()
    }
    assert REQUIRED_FEATURE_COUNTS == {
        name: contract["minimum_count"]
        for name, contract in feature_types.items()
        if contract["minimum_count"] > 0
    }


def test_duplicate_id_and_wrong_frame_report_stable_object_ids():
    document = _valid_document()
    document["features"][1]["properties"]["id"] = "field_01"
    document["features"][2]["properties"]["frame_id"] = "odom"

    report = validate_semantic_map(SemanticMap.from_geojson(document))
    issues = {(issue.code, issue.object_id) for issue in report.issues}
    assert ("duplicate_feature_id", "field_01") in issues
    assert ("invalid_feature_frame", "entry_01") in issues


def test_required_enabled_features_and_geometry_types_are_checked():
    document = _valid_document()
    for feature in document["features"]:
        if feature["properties"]["feature_type"] == "exclusion_zone":
            feature["properties"]["enabled"] = False
        if feature["properties"]["feature_type"] == "entry_pose":
            feature["geometry"]["type"] = "LineString"

    codes = _error_codes(document)
    assert "missing_feature_type" in codes
    assert "invalid_geometry_type" in codes


def test_polygon_closure_and_work_direction_length_are_checked():
    document = deepcopy(_valid_document())
    document["features"][0]["geometry"]["coordinates"][0][-1] = [1.0, 1.0]
    direction = next(
        feature
        for feature in document["features"]
        if feature["properties"]["feature_type"] == "work_direction"
    )
    direction["geometry"]["coordinates"] = [[1.0, 1.0], [1.0, 1.0]]

    codes = _error_codes(document)
    assert "polygon_not_closed" in codes
    assert "zero_length_work_direction" in codes


def test_polygon_unique_vertices_and_row_point_count_are_checked():
    document = _valid_document()
    field = _feature(document, "field_boundary")
    field["geometry"]["coordinates"] = [
        [[0.0, 0.0], [1.0, 0.0], [1.0, 0.0], [0.0, 0.0]]
    ]
    row = _feature(document, "row_centerline")
    row["geometry"]["coordinates"] = [[1.0, 1.5]]

    codes = _error_codes(document)
    assert "polygon_too_small" in codes
    assert "line_too_short" in codes


def test_self_intersecting_polygon_reports_its_object_id():
    document = _valid_document()
    field = _feature(document, "field_boundary")
    field["geometry"]["coordinates"] = [
        [[0.0, 0.0], [8.0, 6.0], [0.0, 6.0], [8.0, 0.0], [0.0, 0.0]]
    ]

    report = validate_semantic_map(SemanticMap.from_geojson(document))

    assert ("polygon_self_intersection", "field_01") in {
        (issue.code, issue.object_id) for issue in report.issues
    }


def test_exclusion_must_be_contained_by_a_field():
    document = _valid_document()
    exclusion = _feature(document, "exclusion_zone")
    exclusion["geometry"]["coordinates"] = [
        [[7.5, 2.0], [8.5, 2.0], [8.5, 3.0], [7.5, 3.0], [7.5, 2.0]]
    ]

    assert "exclusion_outside_field" in _error_codes(document)


def test_entry_must_be_inside_field_and_outside_exclusion():
    outside = _valid_document()
    _feature(outside, "entry_pose")["geometry"]["coordinates"] = [9.0, 9.0]
    assert "entry_outside_field" in _error_codes(outside)

    blocked = _valid_document()
    _feature(blocked, "entry_pose")["geometry"]["coordinates"] = [3.5, 2.5]
    assert "entry_inside_exclusion" in _error_codes(blocked)


def test_all_feature_coordinates_must_be_inside_rotated_map_extent():
    document = _valid_document()
    row = _feature(document, "row_centerline")
    row["geometry"]["coordinates"][-1] = [11.0, 1.5]

    assert "coordinate_outside_map" in _error_codes(document, _context())

    rotated = ValidationContext(
        map_geometry=MapGeometry(
            resolution=1.0,
            width=10,
            height=10,
            origin_x=10.0,
            origin_y=0.0,
            origin_yaw=1.5707963267948966,
        )
    )
    point_only = deepcopy(document)
    for feature in point_only["features"]:
        feature["properties"]["enabled"] = False
    entry = _feature(point_only, "entry_pose")
    entry["properties"]["enabled"] = True
    entry["geometry"]["coordinates"] = [9.0, 1.0]
    assert "coordinate_outside_map" not in _error_codes(point_only, rotated)


def test_navigation_footprint_must_fit_field_and_avoid_exclusion():
    outside = _valid_document()
    _feature(outside, "entry_pose")["geometry"]["coordinates"] = [0.2, 1.0]
    assert "entry_footprint_outside_field" in _error_codes(outside, _context())

    collision = _valid_document()
    _feature(collision, "entry_pose")["geometry"]["coordinates"] = [2.5, 2.5]
    codes = _error_codes(collision, _context())
    assert "entry_inside_exclusion" not in codes
    assert "entry_footprint_intersects_exclusion" in codes


def test_configured_boundary_clearance_is_measured_from_footprint():
    document = _valid_document()

    assert "insufficient_boundary_clearance" not in _error_codes(
        document, _context(clearance=0.4)
    )
    assert "insufficient_boundary_clearance" in _error_codes(
        document, _context(clearance=0.5)
    )
