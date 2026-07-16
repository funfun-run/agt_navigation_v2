"""Serializable semantic map data models."""

from copy import deepcopy
from dataclasses import dataclass, field


@dataclass
class SemanticFeature:
    id: str
    feature_type: str
    name: str
    geometry_type: str
    coordinates: object
    enabled: bool = True
    frame_id: str = "map"
    properties: dict = field(default_factory=dict)

    @classmethod
    def from_geojson(cls, feature):
        properties = deepcopy(feature.get("properties", {}))
        known = {
            key: properties.pop(key)
            for key in ("id", "feature_type", "name")
            if key in properties
        }
        enabled = properties.pop("enabled", True)
        frame_id = properties.pop("frame_id", "map")
        geometry = feature.get("geometry", {})
        return cls(
            id=known.get("id", ""),
            feature_type=known.get("feature_type", ""),
            name=known.get("name", ""),
            geometry_type=geometry.get("type", ""),
            coordinates=deepcopy(geometry.get("coordinates")),
            enabled=enabled,
            frame_id=frame_id,
            properties=properties,
        )

    def to_geojson(self):
        properties = deepcopy(self.properties)
        properties.update(
            {
                "id": self.id,
                "feature_type": self.feature_type,
                "name": self.name,
                "enabled": self.enabled,
                "frame_id": self.frame_id,
            }
        )
        return {
            "type": "Feature",
            "geometry": {
                "type": self.geometry_type,
                "coordinates": deepcopy(self.coordinates),
            },
            "properties": properties,
        }


@dataclass
class SemanticMap:
    map_id: str
    features: list = field(default_factory=list)
    schema_version: str = "1.0"
    frame_id: str = "map"

    @classmethod
    def from_geojson(cls, document):
        if document.get("type") != "FeatureCollection":
            raise ValueError("semantic map must be a GeoJSON FeatureCollection")
        return cls(
            schema_version=str(document.get("schema_version", "")),
            map_id=str(document.get("map_id", "")),
            frame_id=str(document.get("frame_id", "")),
            features=[
                SemanticFeature.from_geojson(feature)
                for feature in document.get("features", [])
            ],
        )

    def to_geojson(self):
        return {
            "type": "FeatureCollection",
            "schema_version": self.schema_version,
            "map_id": self.map_id,
            "frame_id": self.frame_id,
            "features": [feature.to_geojson() for feature in self.features],
        }


@dataclass
class CoverageParameters:
    map_id: str
    base_map: str
    base_map_sha256: str
    robot_profile: str
    planning_mode: str
    robot_width: float
    operation_width: float
    min_turning_radius: float
    headland_width: float
    allow_reverse: bool
    preferred_swath_angle: float
    row_interpretation: str = "direct_swaths"
    schema_version: str = "1.0"
    frame_id: str = "map"
    extra: dict = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data):
        known_fields = {
            "schema_version",
            "map_id",
            "frame_id",
            "base_map",
            "base_map_sha256",
            "robot_profile",
            "planning_mode",
            "robot_width",
            "operation_width",
            "min_turning_radius",
            "headland_width",
            "allow_reverse",
            "preferred_swath_angle",
            "row_interpretation",
        }
        missing = known_fields - {"row_interpretation"} - set(data)
        if missing:
            raise ValueError(
                "coverage parameters missing: " + ", ".join(sorted(missing))
            )
        return cls(
            schema_version=str(data["schema_version"]),
            map_id=str(data["map_id"]),
            frame_id=str(data["frame_id"]),
            base_map=str(data["base_map"]),
            base_map_sha256=str(data["base_map_sha256"]),
            robot_profile=str(data["robot_profile"]),
            planning_mode=str(data["planning_mode"]),
            robot_width=float(data["robot_width"]),
            operation_width=float(data["operation_width"]),
            min_turning_radius=float(data["min_turning_radius"]),
            headland_width=float(data["headland_width"]),
            allow_reverse=data["allow_reverse"],
            preferred_swath_angle=float(data["preferred_swath_angle"]),
            row_interpretation=str(data.get("row_interpretation", "direct_swaths")),
            extra={key: deepcopy(value) for key, value in data.items() if key not in known_fields},
        )

    def to_dict(self):
        result = deepcopy(self.extra)
        result.update(
            {
                "schema_version": self.schema_version,
                "map_id": self.map_id,
                "frame_id": self.frame_id,
                "base_map": self.base_map,
                "base_map_sha256": self.base_map_sha256,
                "robot_profile": self.robot_profile,
                "planning_mode": self.planning_mode,
                "robot_width": self.robot_width,
                "operation_width": self.operation_width,
                "min_turning_radius": self.min_turning_radius,
                "headland_width": self.headland_width,
                "allow_reverse": self.allow_reverse,
                "preferred_swath_angle": self.preferred_swath_angle,
                "row_interpretation": self.row_interpretation,
            }
        )
        return result
