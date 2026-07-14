from copy import deepcopy
from pathlib import Path
import math
import sys
import xml.etree.ElementTree as ET

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPOSITORY_ROOT / "src/agt_ui_bridge"))

from agt_coverage_planning.coverage_adapter import (  # noqa: E402
    CoverageAdapterError,
    GML_NAMESPACE,
    prepare_coverage_request,
)
from agt_ui_bridge.platform_profile import load_platform_profile  # noqa: E402
from agt_ui_bridge.semantic_io import load_semantic_task  # noqa: E402


SEMANTIC_PATH = (
    REPOSITORY_ROOT
    / "docs/interfaces/examples/semantic_map/semantic/semantic_map.geojson"
)
PROFILE_PATH = REPOSITORY_ROOT / "profiles/platforms/bunker.yaml"
ANNOTATED_ROWS_PATH = (
    REPOSITORY_ROOT
    / "docs/interfaces/examples/semantic_map/annotated_rows/semantic_map.geojson"
)


def _task():
    return load_semantic_task(SEMANTIC_PATH)


def _platform():
    return load_platform_profile(PROFILE_PATH)


def test_polygon_mode_maps_boundary_void_direction_and_profile():
    request = prepare_coverage_request(_task(), _platform())

    assert request.planning_mode == "polygon"
    assert request.frame_id == "map"
    assert len(request.polygons) == 2
    assert request.polygons[0][0] == (0.0, 0.0)
    assert request.polygons[1][0] == (3.0, 2.0)
    assert request.swath_angle == pytest.approx(0.0)
    assert request.robot_width == pytest.approx(0.938)
    assert request.operation_width == pytest.approx(0.60)
    assert request.min_turning_radius == pytest.approx(0.0)
    assert request.generate_headland
    assert request.path_mode == "REEDS_SHEPP"
    assert request.swath_mode == "SET_ANGLE"
    assert request.row_swath_mode == "UNKNOWN"


def test_work_direction_is_normalized_as_an_undirected_swath_angle():
    task = _task()
    direction = next(
        feature
        for feature in task.semantic_map.features
        if feature.feature_type == "work_direction"
    )
    direction.coordinates = [[7.0, 1.0], [1.0, 1.0]]

    request = prepare_coverage_request(task, _platform())
    assert request.swath_angle == pytest.approx(0.0)
    assert 0.0 <= request.swath_angle < math.pi


def test_annotated_rows_mode_generates_humble_compatible_gml():
    task = _task()
    task.coverage.planning_mode = "annotated_rows"
    first_row = next(
        feature
        for feature in task.semantic_map.features
        if feature.feature_type == "row_centerline"
    )
    second_row = deepcopy(first_row)
    second_row.id = "row_02"
    second_row.name = "row 2"
    second_row.coordinates = [[1.0, 2.5], [7.0, 2.5]]
    task.semantic_map.features.append(second_row)

    request = prepare_coverage_request(task, _platform())
    root = ET.fromstring(request.gml_text)
    namespace = {"gml": GML_NAMESPACE}

    assert request.planning_mode == "annotated_rows"
    assert request.polygons == ()
    assert request.row_swath_mode == "ROWSARESWATHS"
    assert request.swath_mode == "UNKNOWN"
    assert not request.generate_headland
    assert len(root.findall("Row")) == 2
    assert [element.attrib["id"] for element in root.findall("Row")] == ["1", "2"]
    polygon = root.find("./Field/geometry/gml:Polygon", namespace)
    assert polygon is not None
    assert polygon.attrib["srsName"] == "map"
    assert polygon.find("gml:outerBoundaryIs", namespace) is not None
    assert len(polygon.findall("gml:innerBoundaryIs", namespace)) == 1
    lines = root.findall("./Row/geometry/gml:LineString", namespace)
    assert all(line.attrib["srsName"] == "map" for line in lines)


def test_versioned_annotated_rows_example_is_directly_convertible():
    request = prepare_coverage_request(
        load_semantic_task(ANNOTATED_ROWS_PATH), _platform()
    )

    assert request.planning_mode == "annotated_rows"
    assert request.gml_text.count("<Row id=") == 2


def test_annotated_rows_rejects_single_row_with_stable_reason():
    task = _task()
    task.coverage.planning_mode = "annotated_rows"

    with pytest.raises(CoverageAdapterError) as error:
        prepare_coverage_request(task, _platform())

    assert error.value.code == "insufficient_annotated_rows"
    assert error.value.object_id == "row_centerline"


def test_profile_snapshot_mismatch_blocks_request():
    task = _task()
    task.coverage.robot_width += 0.01

    with pytest.raises(CoverageAdapterError) as error:
        prepare_coverage_request(task, _platform())

    assert error.value.code == "robot_width_profile_mismatch"


def test_multiple_fields_are_rejected_instead_of_silently_dropped():
    task = _task()
    field = next(
        feature
        for feature in task.semantic_map.features
        if feature.feature_type == "field_boundary"
    )
    second_field = deepcopy(field)
    second_field.id = "field_02"
    task.semantic_map.features.append(second_field)

    with pytest.raises(CoverageAdapterError) as error:
        prepare_coverage_request(task, _platform())

    assert error.value.code == "unsupported_field_count"


def test_invalid_semantic_geometry_preserves_validation_error_code():
    task = _task()
    exclusion = next(
        feature
        for feature in task.semantic_map.features
        if feature.feature_type == "exclusion_zone"
    )
    exclusion.coordinates = [
        [[3.0, 2.0], [4.0, 3.0], [3.0, 3.0], [4.0, 2.0], [3.0, 2.0]]
    ]

    with pytest.raises(CoverageAdapterError) as error:
        prepare_coverage_request(task, _platform())

    assert error.value.code == "polygon_self_intersection"
    assert error.value.object_id == "exclusion_01"
