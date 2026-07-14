from copy import deepcopy
import json
import os
from pathlib import Path
import shutil
import sys

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from agt_ui_bridge import semantic_io  # noqa: E402
from agt_ui_bridge.semantic_io import (  # noqa: E402
    SemanticFileError,
    load_semantic_task,
    save_semantic_task,
    sha256_file,
)


EXAMPLE_ROOT = (
    PACKAGE_ROOT.parents[1] / "docs/interfaces/examples/semantic_map"
)
SEMANTIC_PATH = EXAMPLE_ROOT / "semantic/semantic_map.geojson"
COVERAGE_PATH = EXAMPLE_ROOT / "semantic/coverage.yaml"


def _copy_task(tmp_path):
    shutil.copy2(EXAMPLE_ROOT / "example_map.yaml", tmp_path / "example_map.yaml")
    shutil.copy2(EXAMPLE_ROOT / "example_map.pgm", tmp_path / "example_map.pgm")
    semantic_dir = tmp_path / "semantic"
    semantic_dir.mkdir()
    shutil.copy2(SEMANTIC_PATH, semantic_dir / "semantic_map.geojson")
    coverage = yaml.safe_load(COVERAGE_PATH.read_text(encoding="utf-8"))
    coverage["base_map_sha256"] = sha256_file(tmp_path / "example_map.yaml")
    (semantic_dir / "coverage.yaml").write_text(
        yaml.safe_dump(coverage, sort_keys=False), encoding="utf-8"
    )
    return semantic_dir


def test_valid_task_loads_writable_and_preserves_properties():
    task = load_semantic_task(SEMANTIC_PATH, COVERAGE_PATH)

    assert not task.read_only
    assert task.warnings == []
    assert task.semantic_map.map_id == "example_semantic"
    entry = next(
        feature
        for feature in task.semantic_map.features
        if feature.feature_type == "entry_pose"
    )
    assert entry.properties["yaw"] == 0.0


def test_semantic_task_round_trip_is_lossless(tmp_path):
    source = load_semantic_task(SEMANTIC_PATH, COVERAGE_PATH)
    semantic_dir = _copy_task(tmp_path)

    save_semantic_task(
        source.semantic_map,
        source.coverage,
        semantic_dir / "semantic_map.geojson",
        semantic_dir / "coverage.yaml",
    )
    loaded = load_semantic_task(
        semantic_dir / "semantic_map.geojson",
        semantic_dir / "coverage.yaml",
    )

    assert loaded.semantic_map.to_geojson() == source.semantic_map.to_geojson()
    assert loaded.coverage.to_dict() == source.coverage.to_dict()


def test_hash_mismatch_degrades_to_read_only_without_modifying_files(tmp_path):
    semantic_dir = _copy_task(tmp_path)
    map_path = tmp_path / "example_map.yaml"
    original_semantic = (semantic_dir / "semantic_map.geojson").read_bytes()
    map_path.write_text(map_path.read_text() + "# changed\n", encoding="utf-8")

    loaded = load_semantic_task(semantic_dir / "semantic_map.geojson")

    assert loaded.read_only
    assert "base_map_hash_mismatch" in loaded.warnings
    assert (semantic_dir / "semantic_map.geojson").read_bytes() == original_semantic


def test_atomic_write_failure_preserves_existing_file(tmp_path, monkeypatch):
    destination = tmp_path / "semantic_map.geojson"
    destination.write_text("original", encoding="utf-8")

    def fail_replace(_source, _destination):
        raise OSError("simulated replacement failure")

    monkeypatch.setattr(os, "replace", fail_replace)
    with pytest.raises(OSError, match="simulated replacement failure"):
        semantic_io._atomic_write_text(destination, "replacement")

    assert destination.read_text(encoding="utf-8") == "original"
    assert list(tmp_path.glob("*.tmp")) == []
    assert list(tmp_path.glob(".*.tmp")) == []


def test_save_rejects_invalid_semantic_map(tmp_path):
    task = load_semantic_task(SEMANTIC_PATH, COVERAGE_PATH)
    invalid_map = deepcopy(task.semantic_map)
    invalid_map.frame_id = "odom"

    with pytest.raises(SemanticFileError, match="invalid_frame"):
        save_semantic_task(
            invalid_map,
            task.coverage,
            tmp_path / "semantic_map.geojson",
            tmp_path / "coverage.yaml",
        )
    assert not (tmp_path / "semantic_map.geojson").exists()


def test_model_json_output_remains_standard_geojson():
    task = load_semantic_task(SEMANTIC_PATH, COVERAGE_PATH)
    encoded = json.dumps(task.semantic_map.to_geojson())
    document = json.loads(encoded)
    assert document["type"] == "FeatureCollection"
