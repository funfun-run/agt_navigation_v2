"""Pure geometry helpers for semantic-editor coverage previews."""

from copy import deepcopy
import math

from .semantic_model import SemanticFeature


class CoveragePreviewError(ValueError):
    pass


def derive_inter_row_aisles(semantic_map):
    """Return a copied map whose crop-row lines are replaced by aisle midlines."""
    rows = [
        feature
        for feature in semantic_map.features
        if feature.enabled and feature.feature_type == "row_centerline"
    ]
    if len(rows) < 2:
        raise CoveragePreviewError("作物行间道路至少需要两条启用的作物行")

    direction = _work_direction(semantic_map)
    normal = (-direction[1], direction[0])
    normalized = [_normalized_endpoints(row, direction) for row in rows]
    normalized.sort(
        key=lambda item: _dot(_midpoint(item[1], item[2]), normal), reverse=True
    )

    aisles = []
    for index, (upper, lower) in enumerate(zip(normalized, normalized[1:]), start=1):
        start = _midpoint(upper[1], lower[1])
        end = _midpoint(upper[2], lower[2])
        if math.dist(start, end) <= 1e-6:
            raise CoveragePreviewError(f"第 {index} 条行间道路长度为零")
        aisles.append(
            SemanticFeature(
                id=f"preview_aisle_{index:03d}",
                feature_type="row_centerline",
                name=f"行间道路 {index}",
                geometry_type="LineString",
                coordinates=[start, end],
            )
        )

    output = deepcopy(semantic_map)
    output.features = [
        feature for feature in output.features if feature.feature_type != "row_centerline"
    ] + aisles
    return output


def _work_direction(semantic_map):
    feature = next(
        (
            item
            for item in semantic_map.features
            if item.enabled and item.feature_type == "work_direction"
        ),
        None,
    )
    if feature is None or len(feature.coordinates) < 2:
        raise CoveragePreviewError("缺少有效作业方向")
    start, end = feature.coordinates[0], feature.coordinates[-1]
    delta = (end[0] - start[0], end[1] - start[1])
    length = math.hypot(*delta)
    if length <= 1e-6:
        raise CoveragePreviewError("作业方向长度为零")
    return delta[0] / length, delta[1] / length


def _normalized_endpoints(feature, direction):
    if len(feature.coordinates) < 2:
        raise CoveragePreviewError(f"{feature.id} 至少需要两个点")
    start = [float(value) for value in feature.coordinates[0]]
    end = [float(value) for value in feature.coordinates[-1]]
    if _dot((end[0] - start[0], end[1] - start[1]), direction) < 0.0:
        start, end = end, start
    return feature.id, start, end


def _midpoint(first, second):
    return [(first[0] + second[0]) * 0.5, (first[1] + second[1]) * 0.5]


def _dot(first, second):
    return first[0] * second[0] + first[1] * second[1]
