"""Atomic semantic task file I/O and base-map consistency checks."""

from dataclasses import dataclass, field
import hashlib
import json
import os
from pathlib import Path
import tempfile

import yaml

from .map_transform import MapGeometry
from .semantic_model import CoverageParameters, SemanticMap
from .semantic_validation import ValidationContext, validate_task


@dataclass
class LoadedSemanticTask:
    semantic_map: SemanticMap
    coverage: CoverageParameters
    semantic_path: Path
    coverage_path: Path
    base_map_path: Path
    read_only: bool = False
    warnings: list = field(default_factory=list)


class SemanticFileError(ValueError):
    pass


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_semantic_map(path):
    path = Path(path)
    document = json.loads(path.read_text(encoding="utf-8"))
    return SemanticMap.from_geojson(document)


def load_coverage(path):
    path = Path(path)
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise SemanticFileError("coverage file must contain a YAML mapping")
    return CoverageParameters.from_dict(data)


def load_semantic_task(semantic_path, coverage_path=None):
    semantic_path = Path(semantic_path).resolve()
    coverage_path = (
        Path(coverage_path).resolve()
        if coverage_path is not None
        else semantic_path.with_name("coverage.yaml")
    )
    semantic_map = load_semantic_map(semantic_path)
    coverage = load_coverage(coverage_path)
    base_map_path = (coverage_path.parent / coverage.base_map).resolve()
    warnings = []
    read_only = False

    map_geometry = None
    if not base_map_path.is_file():
        read_only = True
        warnings.append("base_map_missing")
    else:
        try:
            map_geometry = MapGeometry.from_nav2_yaml(base_map_path)
        except (KeyError, OSError, TypeError, ValueError):
            read_only = True
            warnings.append("base_map_image_missing")

    report = validate_task(
        semantic_map,
        coverage,
        context=ValidationContext(
            map_geometry=map_geometry,
            base_map_path=base_map_path,
        ),
    )
    if not report.valid:
        read_only = True
        warnings.extend(issue.code for issue in report.issues)

    return LoadedSemanticTask(
        semantic_map=semantic_map,
        coverage=coverage,
        semantic_path=semantic_path,
        coverage_path=coverage_path,
        base_map_path=base_map_path,
        read_only=read_only,
        warnings=list(dict.fromkeys(warnings)),
    )


def save_semantic_task(
    semantic_map,
    coverage,
    semantic_path,
    coverage_path=None,
    validation_context=None,
):
    report = validate_task(semantic_map, coverage, context=validation_context)
    if not report.valid:
        codes = ", ".join(issue.code for issue in report.issues)
        raise SemanticFileError(f"semantic task validation failed: {codes}")

    semantic_path = Path(semantic_path)
    coverage_path = (
        Path(coverage_path)
        if coverage_path is not None
        else semantic_path.with_name("coverage.yaml")
    )
    semantic_text = json.dumps(
        semantic_map.to_geojson(), ensure_ascii=False, indent=2
    ) + "\n"
    coverage_text = yaml.safe_dump(
        coverage.to_dict(), sort_keys=False, allow_unicode=True
    )

    _atomic_write_text(semantic_path, semantic_text)
    _atomic_write_text(coverage_path, coverage_text)
    return semantic_path, coverage_path


def _atomic_write_text(path, text):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary_path = Path(stream.name)
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()
