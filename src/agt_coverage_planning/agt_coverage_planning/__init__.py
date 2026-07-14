"""Semantic-map adapters for agricultural coverage planning."""

from .coverage_adapter import (
    CoverageAdapterError,
    CoverageRequestSpec,
    prepare_coverage_request,
)

__all__ = [
    "CoverageAdapterError",
    "CoverageRequestSpec",
    "prepare_coverage_request",
]
