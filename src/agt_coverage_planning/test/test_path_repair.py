import json
from pathlib import Path
import sys

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPOSITORY_ROOT / "src/agt_ui_bridge"))

from agt_coverage_planning.path_repair import (  # noqa: E402
    PathRepairError,
    apply_connection_repairs,
    prepare_connection_repairs,
    repair_policy_from_profile,
)
from agt_coverage_planning.path_semantics import (  # noqa: E402
    Pose2D,
    SwathInput,
    TurnInput,
    build_path_semantics,
)


def _pose(x, y, yaw=0.0):
    return Pose2D(float(x), float(y), float(yaw))


def _products():
    swaths = [
        SwathInput(_pose(0, 0), _pose(4, 0)),
        SwathInput(_pose(4, 2), _pose(0, 2)),
    ]
    turns = [TurnInput((_pose(4, 0), _pose(5, 1), _pose(4, 2, 3.14)))]
    raw = [
        _pose(0, 0),
        _pose(4, 0),
        _pose(5, 1),
        _pose(4, 2, 3.14),
        _pose(0, 2, 3.14),
    ]
    semantics = build_path_semantics(raw, swaths, turns, swath_sample_step=1.0)
    return semantics.reconstructed_poses, semantics.to_dict()


def _invalid_connection_report(semantics):
    return {
        "valid": False,
        "path_fingerprint": semantics["path_fingerprint"],
        "invalid_component_ids": ["connection_0001"],
        "invalid_swath_ids": [],
    }


def test_only_invalid_connections_are_selected_for_repair():
    path, semantics = _products()
    preparation = prepare_connection_repairs(
        path, semantics, _invalid_connection_report(semantics)
    )

    assert [target.component_id for target in preparation.targets] == [
        "connection_0001"
    ]
    assert preparation.preserved_swath_ids == ("swath_0001", "swath_0002")


def test_replacement_preserves_every_swath_pose_exactly():
    path, semantics = _products()
    preparation = prepare_connection_repairs(
        path, semantics, _invalid_connection_report(semantics)
    )
    target = preparation.targets[0]
    replacement = [
        path[target.start_pose_index],
        _pose(4.0, 0.8, 1.57),
        _pose(4.0, 1.2, 1.57),
        path[target.end_pose_index],
    ]
    result = apply_connection_repairs(
        path, semantics, preparation, {target.component_id: replacement}
    )

    assert result.swath_coordinates_unchanged
    assert result.repaired_component_ids == ["connection_0001"]
    for component in semantics["components"]:
        if component["component_type"] != "SWATH":
            continue
        expected = path[
            component["start_pose_index"]:component["end_pose_index"] + 1
        ]
        assert any(
            result.poses[index:index + len(expected)] == expected
            for index in range(len(result.poses) - len(expected) + 1)
        )


def test_candidate_endpoints_are_checked_then_forced_to_exact_source_values():
    path, semantics = _products()
    preparation = prepare_connection_repairs(
        path, semantics, _invalid_connection_report(semantics)
    )
    target = preparation.targets[0]
    replacement = [
        _pose(path[target.start_pose_index].x + 0.05, path[target.start_pose_index].y),
        _pose(4.2, 1.0),
        _pose(path[target.end_pose_index].x - 0.05, path[target.end_pose_index].y),
    ]
    result = apply_connection_repairs(
        path, semantics, preparation, {target.component_id: replacement}
    )
    assert path[target.start_pose_index] in result.poses
    assert path[target.end_pose_index] in result.poses

    replacement[0] = _pose(10, 10)
    with pytest.raises(PathRepairError) as mismatch:
        apply_connection_repairs(
            path, semantics, preparation, {target.component_id: replacement}
        )
    assert mismatch.value.code == "repair_endpoint_mismatch"


def test_invalid_swath_and_incomplete_repair_are_rejected_without_mutation():
    path, semantics = _products()
    original = list(path)
    report = {
        "valid": False,
        "path_fingerprint": semantics["path_fingerprint"],
        "invalid_component_ids": ["swath_0001"],
        "invalid_swath_ids": ["swath_0001"],
    }
    with pytest.raises(PathRepairError) as forbidden:
        prepare_connection_repairs(path, semantics, report)
    assert forbidden.value.code == "swath_repair_forbidden"
    assert path == original

    preparation = prepare_connection_repairs(
        path, semantics, _invalid_connection_report(semantics)
    )
    with pytest.raises(PathRepairError) as incomplete:
        apply_connection_repairs(path, semantics, preparation, {})
    assert incomplete.value.code == "incomplete_repair_set"
    assert path == original


def test_semantic_fingerprint_and_unknown_components_are_rejected():
    path, semantics = _products()
    changed = list(path)
    changed[-1] = _pose(-0.1, 2, 3.14)
    with pytest.raises(PathRepairError) as fingerprint:
        prepare_connection_repairs(
            changed, semantics, _invalid_connection_report(semantics)
        )
    assert fingerprint.value.code == "reconstructed_path_fingerprint_mismatch"

    report = _invalid_connection_report(semantics)
    report["path_fingerprint"] = "stale"
    with pytest.raises(PathRepairError) as stale_report:
        prepare_connection_repairs(path, semantics, report)
    assert stale_report.value.code == "validation_report_fingerprint_mismatch"

    report = _invalid_connection_report(semantics)
    report["invalid_component_ids"] = ["connection_9999"]
    with pytest.raises(PathRepairError) as unknown:
        prepare_connection_repairs(path, semantics, report)
    assert unknown.value.code == "unknown_invalid_component"


def test_bunker_policy_is_explicit_and_ackermann_requires_hybrid_contract():
    bunker = yaml.safe_load(
        (REPOSITORY_ROOT / "profiles/platforms/bunker.yaml").read_text(
            encoding="utf-8"
        )
    )
    policy = repair_policy_from_profile(bunker)
    assert policy.kinematics == "tracked_differential"
    assert policy.planner_id == "GridBased"
    assert policy.allow_in_place_rotation

    ackermann = {
        "platform": {
            "name": "test_ackermann",
            "kinematics": "ackermann",
            "geometry": {"min_turning_radius": 0.8},
            "coverage_repair": {
                "enabled": True,
                "planner_id": "CoverageRepairHybrid",
                "planner_family": "hybrid_a_star",
                "allow_in_place_rotation": False,
            },
        }
    }
    assert repair_policy_from_profile(ackermann).planner_id == "CoverageRepairHybrid"
    ackermann["platform"]["coverage_repair"]["planner_family"] = "grid_2d"
    with pytest.raises(PathRepairError) as planner:
        repair_policy_from_profile(ackermann)
    assert planner.value.code == "ackermann_planner_contract_invalid"

    mk_mini = yaml.safe_load(
        (REPOSITORY_ROOT / "profiles/platforms/mk_mini.yaml").read_text(
            encoding="utf-8"
        )
    )
    with pytest.raises(PathRepairError) as disabled:
        repair_policy_from_profile(mk_mini)
    assert disabled.value.code == "coverage_repair_disabled"


def test_valid_path_requires_no_repair_and_report_json_is_stable():
    path, semantics = _products()
    preparation = prepare_connection_repairs(
        path,
        semantics,
        {
            "valid": True,
            "path_fingerprint": semantics["path_fingerprint"],
            "invalid_component_ids": [],
            "invalid_swath_ids": [],
        },
    )
    assert preparation.targets == ()
    assert json.dumps(semantics, sort_keys=True, allow_nan=False)
