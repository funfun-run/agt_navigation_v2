import json
from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPOSITORY_ROOT / "src/agt_ui_bridge"))

from agt_coverage_planning.path_semantics import (  # noqa: E402
    CONNECTION,
    SWATH,
    PathSemanticsError,
    Pose2D,
    SwathInput,
    TurnInput,
    build_path_semantics,
    parse_path_semantics,
)


def _pose(x, y, yaw=0.0):
    return Pose2D(float(x), float(y), float(yaw))


def _normal_inputs():
    swaths = [
        SwathInput(_pose(0, 0), _pose(4, 0)),
        SwathInput(_pose(4, 1), _pose(0, 1)),
    ]
    turns = [TurnInput((_pose(4, 0), _pose(4.5, 0.5), _pose(4, 1, 3.14)))]
    raw = [
        _pose(0, 0),
        _pose(2, 0),
        _pose(4, 0),
        _pose(4.5, 0.5),
        _pose(4, 1, 3.14),
        _pose(2, 1, 3.14),
        _pose(0, 1, 3.14),
    ]
    return raw, swaths, turns


def test_every_raw_interval_is_classified_and_flat_path_is_reconstructed():
    raw, swaths, turns = _normal_inputs()
    result = build_path_semantics(raw, swaths, turns, swath_sample_step=0.5)

    assert len(result.raw_labels) == len(raw) - 1
    assert {label.component_type for label in result.raw_labels} == {
        SWATH,
        CONNECTION,
    }
    assert result.length_error <= result.length_tolerance
    assert result.reconstructed_poses[0] == raw[0]
    assert result.reconstructed_poses[-1].x == raw[-1].x
    assert len(result.reconstructed_labels) == len(result.reconstructed_poses) - 1
    assert all(
        0 <= component.start_pose_index <= component.end_pose_index
        for component in result.components
    )
    assert result.components[1].start_pose_index == result.components[0].end_pose_index


def test_swath_ids_are_stable_when_route_order_and_direction_change():
    first = SwathInput(_pose(0, 0), _pose(4, 0))
    second = SwathInput(_pose(4, 1), _pose(0, 1))
    raw, _swaths, turns = _normal_inputs()
    baseline = build_path_semantics(raw, [first, second], turns)

    reversed_raw = list(reversed([_pose(p.x, p.y, p.yaw) for p in raw]))
    reversed_turn = TurnInput(tuple(reversed(turns[0].poses)))
    changed = build_path_semantics(
        reversed_raw,
        [SwathInput(second.end, second.start), SwathInput(first.end, first.start)],
        [reversed_turn],
    )

    baseline_by_y = {
        component.poses[0].y: component.swath_id
        for component in baseline.components
        if component.component_type == SWATH
    }
    changed_by_y = {
        component.poses[0].y: component.swath_id
        for component in changed.components
        if component.component_type == SWATH
    }
    assert baseline_by_y == changed_by_y == {0.0: "swath_0001", 1.0: "swath_0002"}


def test_leading_and_trailing_connections_are_ordered_from_raw_endpoints():
    swath = SwathInput(_pose(1, 0), _pose(3, 0))
    leading = TurnInput((_pose(0, 0), _pose(1, 0)))
    trailing = TurnInput((_pose(3, 0), _pose(4, 0)))
    result = build_path_semantics(
        [_pose(0, 0), _pose(1, 0), _pose(3, 0), _pose(4, 0)],
        [swath],
        [leading, trailing],
    )

    assert [component.component_type for component in result.components] == [
        CONNECTION,
        SWATH,
        CONNECTION,
    ]


def test_equal_component_counts_select_leading_or_trailing_by_geometry():
    swath = SwathInput(_pose(1, 0), _pose(3, 0))
    leading = build_path_semantics(
        [_pose(0, 0), _pose(1, 0), _pose(3, 0)],
        [swath],
        [TurnInput((_pose(0, 0), _pose(1, 0)))],
    )
    trailing = build_path_semantics(
        [_pose(1, 0), _pose(3, 0), _pose(4, 0)],
        [swath],
        [TurnInput((_pose(3, 0), _pose(4, 0)))],
    )

    assert leading.components[0].component_type == CONNECTION
    assert trailing.components[-1].component_type == CONNECTION


def test_length_mismatch_and_unordered_components_are_rejected():
    raw, swaths, turns = _normal_inputs()
    with pytest.raises(PathSemanticsError) as mismatch:
        build_path_semantics([_pose(0, 0), _pose(20, 0)], swaths, turns)
    assert mismatch.value.code == "path_reconstruction_length_mismatch"

    with pytest.raises(PathSemanticsError) as unordered:
        build_path_semantics(raw, swaths, turns, swaths_ordered=False)
    assert unordered.value.code == "path_components_unordered_swaths"

    with pytest.raises(PathSemanticsError) as tolerance:
        build_path_semantics(raw, swaths, turns, absolute_length_tolerance=float("nan"))
    assert tolerance.value.code == "invalid_length_tolerance"


def test_document_is_stable_and_round_trips_against_exact_raw_path():
    raw, swaths, turns = _normal_inputs()
    first = build_path_semantics(raw, swaths, turns)
    second = build_path_semantics(raw, swaths, turns)

    assert first.to_json() == second.to_json()
    document = json.loads(first.to_json())
    summary = parse_path_semantics(document, raw)
    assert summary.swath_ids == ("swath_0001", "swath_0002")
    assert len(summary.segment_labels) == len(raw) - 1


def test_document_rejects_fingerprint_mismatch_and_interval_gaps():
    raw, swaths, turns = _normal_inputs()
    document = build_path_semantics(raw, swaths, turns).to_dict()
    changed = list(raw)
    changed[-1] = _pose(0.1, 1, 3.14)
    with pytest.raises(PathSemanticsError) as fingerprint:
        parse_path_semantics(document, changed)
    assert fingerprint.value.code == "path_semantics_fingerprint_mismatch"

    document["raw_segments"] = document["raw_segments"][:-1]
    with pytest.raises(PathSemanticsError) as gap:
        parse_path_semantics(document, raw)
    assert gap.value.code == "unclassified_path_interval"


def test_swath_references_are_available_for_validation_reports():
    raw, swaths, turns = _normal_inputs()
    result = build_path_semantics(raw, swaths, turns)
    summary = parse_path_semantics(result.to_dict(), raw)
    swath_labels = [
        label for label in summary.segment_labels if label.component_type == SWATH
    ]

    assert swath_labels
    assert all(label.swath_id.startswith("swath_") for label in swath_labels)
    assert all(label.component_id == label.swath_id for label in swath_labels)
