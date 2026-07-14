import hashlib
import json
import math
from pathlib import Path
import re

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
SCHEMA_PATH = ROOT / "src/agt_ui_bridge/config/semantic_schema.yaml"
EXAMPLE_ROOT = ROOT / "docs/interfaces/examples/semantic_map"
VALID_MAP_PATH = EXAMPLE_ROOT / "semantic/semantic_map.geojson"
VALID_COVERAGE_PATH = EXAMPLE_ROOT / "semantic/coverage.yaml"
INVALID_ROOT = EXAMPLE_ROOT / "invalid"


def _load_yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _load_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def _validate_semantic_map(document, schema):
    errors = []
    contract = schema["semantic_map"]
    coordinate_contract = schema["coordinate_contract"]
    recognized_versions = schema["load_policy"]["recognized_schema_versions"]

    for member in contract["required_members"]:
        if member not in document:
            errors.append(f"missing_member:{member}")
    if errors:
        return errors

    if document["type"] != contract["type"]:
        errors.append("invalid_collection_type")
    if document["schema_version"] not in recognized_versions:
        errors.append("unsupported_schema_version")
    if document["frame_id"] != coordinate_contract["frame_id"]:
        errors.append("invalid_frame")
    if "crs" in document:
        errors.append("geojson_crs_not_allowed")

    feature_types = contract["feature_types"]
    common = contract["common_feature_properties"]
    id_pattern = re.compile(common["id_pattern"])
    seen_ids = set()
    counts = {feature_type: 0 for feature_type in feature_types}

    for feature in document["features"]:
        properties = feature.get("properties", {})
        feature_id = properties.get("id", "<missing>")
        for member in common["required"]:
            if member not in properties:
                errors.append(f"missing_feature_property:{feature_id}:{member}")

        if feature_id in seen_ids:
            errors.append(f"duplicate_feature_id:{feature_id}")
        seen_ids.add(feature_id)
        if not id_pattern.fullmatch(feature_id):
            errors.append(f"invalid_feature_id:{feature_id}")
        if properties.get("frame_id") != coordinate_contract["frame_id"]:
            errors.append(f"invalid_feature_frame:{feature_id}")
        if not isinstance(properties.get("enabled"), bool):
            errors.append(f"invalid_enabled:{feature_id}")

        feature_type = properties.get("feature_type")
        if feature_type not in feature_types:
            errors.append(f"unknown_feature_type:{feature_id}")
            continue
        counts[feature_type] += 1
        type_contract = feature_types[feature_type]
        geometry = feature.get("geometry", {})
        if geometry.get("type") != type_contract["geometry"]:
            errors.append(f"invalid_geometry_type:{feature_id}")
            continue

        coordinates = geometry.get("coordinates")
        if type_contract["geometry"] == "Point":
            if (
                not isinstance(coordinates, list)
                or len(coordinates) != 2
                or not all(
                    isinstance(value, (int, float)) and math.isfinite(value)
                    for value in coordinates
                )
            ):
                errors.append(f"invalid_point:{feature_id}")
        elif type_contract["geometry"] == "LineString":
            if len(coordinates or []) < type_contract["minimum_points"]:
                errors.append(f"line_too_short:{feature_id}")
        elif type_contract["geometry"] == "Polygon":
            ring = coordinates[0] if coordinates else []
            if not ring or ring[0] != ring[-1]:
                errors.append(f"polygon_not_closed:{feature_id}")
            unique_vertices = {tuple(point) for point in ring[:-1]}
            if len(unique_vertices) < type_contract["minimum_unique_vertices"]:
                errors.append(f"polygon_too_small:{feature_id}")

        for required_property in type_contract.get("required_properties", []):
            value = properties.get(required_property)
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                errors.append(
                    f"invalid_required_property:{feature_id}:{required_property}"
                )

    for feature_type, type_contract in feature_types.items():
        if counts[feature_type] < type_contract["minimum_count"]:
            errors.append(f"missing_feature_type:{feature_type}")
    return errors


def test_schema_defines_all_first_version_feature_types():
    schema = _load_yaml(SCHEMA_PATH)
    assert schema["schema_version"] == "1.0"
    assert schema["coordinate_contract"]["frame_id"] == "map"
    assert set(schema["semantic_map"]["feature_types"]) == {
        "field_boundary",
        "exclusion_zone",
        "entry_pose",
        "work_direction",
        "row_centerline",
        "headland_zone",
        "keepout_zone",
    }
    assert schema["coverage"]["planning_mode_values"] == [
        "polygon",
        "annotated_rows",
    ]


def test_valid_semantic_map_satisfies_the_machine_readable_contract():
    schema = _load_yaml(SCHEMA_PATH)
    semantic_map = _load_json(VALID_MAP_PATH)
    assert _validate_semantic_map(semantic_map, schema) == []


def test_valid_coverage_file_matches_map_profile_and_hash():
    schema = _load_yaml(SCHEMA_PATH)
    semantic_map = _load_json(VALID_MAP_PATH)
    coverage = _load_yaml(VALID_COVERAGE_PATH)

    assert set(schema["coverage"]["required_members"]) <= set(coverage)
    assert coverage["schema_version"] in schema["load_policy"][
        "recognized_schema_versions"
    ]
    assert coverage["map_id"] == semantic_map["map_id"]
    assert coverage["frame_id"] == semantic_map["frame_id"] == "map"
    assert coverage["planning_mode"] in schema["coverage"][
        "planning_mode_values"
    ]
    assert re.fullmatch(
        schema["coverage"]["sha256_pattern"], coverage["base_map_sha256"]
    )

    base_map = (VALID_COVERAGE_PATH.parent / coverage["base_map"]).resolve()
    assert base_map.is_file()
    assert hashlib.sha256(base_map.read_bytes()).hexdigest() == coverage[
        "base_map_sha256"
    ]
    map_yaml = _load_yaml(base_map)
    assert (base_map.parent / map_yaml["image"]).is_file()

    profile_path = ROOT / f"profiles/platforms/{coverage['robot_profile']}.yaml"
    profile = _load_yaml(profile_path)["platform"]
    footprint = profile["geometry"]["navigation_footprint"]
    profile_width = max(point[1] for point in footprint) - min(
        point[1] for point in footprint
    )
    assert coverage["robot_width"] == pytest.approx(profile_width)


@pytest.mark.parametrize(
    ("filename", "expected_error"),
    [
        ("duplicate_feature_id.geojson", "duplicate_feature_id:duplicate_01"),
        ("wrong_frame.geojson", "invalid_frame"),
        ("unsupported_schema.geojson", "unsupported_schema_version"),
    ],
)
def test_invalid_examples_are_rejected_with_stable_errors(
    filename, expected_error
):
    errors = _validate_semantic_map(
        _load_json(INVALID_ROOT / filename), _load_yaml(SCHEMA_PATH)
    )
    assert expected_error in errors


def test_contract_examples_are_versioned_outside_runtime_outputs():
    assert "runtime" not in EXAMPLE_ROOT.parts
    assert len(list(INVALID_ROOT.glob("*.geojson"))) >= 3
