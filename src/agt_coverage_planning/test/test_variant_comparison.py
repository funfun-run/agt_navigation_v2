from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPOSITORY_ROOT / "src/agt_ui_bridge"))

from agt_coverage_planning.variant_comparison import (  # noqa: E402
    VariantComparisonError,
    coverage_area_metrics,
    load_variants,
    rank_candidates,
)


def test_default_variants_cover_route_path_and_angle_alternatives():
    variants = load_variants(PACKAGE_ROOT / "config/coverage_variants.yaml")

    assert len(variants) == 6
    assert {variant.route_mode for variant in variants} == {
        "BOUSTROPHEDON",
        "SNAKE",
        "SPIRAL",
    }
    assert {variant.path_mode for variant in variants} == {"DUBIN", "REEDS_SHEPP"}
    assert len({variant.variant_id for variant in variants}) == len(variants)


def test_area_metrics_report_coverage_overlap_and_missed_area():
    metrics = coverage_area_metrics(
        [(0, 0), (10, 0), (10, 4), (0, 4), (0, 0)],
        [],
        [((0, 1), (10, 1)), ((0, 2), (10, 2))],
        operation_width=2.0,
    )

    assert metrics["target_area"] == pytest.approx(40.0)
    assert metrics["covered_area"] == pytest.approx(30.0)
    assert metrics["overlap_area"] == pytest.approx(10.0)
    assert metrics["missed_area"] == pytest.approx(10.0)
    assert metrics["coverage_rate"] == pytest.approx(0.75)
    assert metrics["overlap_rate"] == pytest.approx(0.25)


def test_area_metrics_reject_zero_length_swath():
    with pytest.raises(VariantComparisonError) as error:
        coverage_area_metrics(
            [(0, 0), (4, 0), (4, 4), (0, 4), (0, 0)],
            [],
            [((1, 1), (1, 1))],
            operation_width=1.0,
        )

    assert error.value.code == "zero_length_swath"


def test_geometric_ranking_is_deterministic_and_not_execution_approval():
    candidates = [
        {
            "variant_id": "slow",
            "status": "SUCCEEDED",
            "estimated_motion_time": 12.0,
            "total_path_length": 8.0,
            "eligible_for_execution": False,
        },
        {
            "variant_id": "fast",
            "status": "SUCCEEDED",
            "estimated_motion_time": 10.0,
            "total_path_length": 9.0,
            "eligible_for_execution": False,
        },
    ]

    assert rank_candidates(candidates) == ("fast", "slow")
    assert candidates[1]["geometric_rank"] == 1
    assert not candidates[1]["eligible_for_execution"]
