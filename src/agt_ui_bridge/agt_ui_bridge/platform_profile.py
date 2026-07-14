"""Read canonical platform geometry without depending on Qt or ROS."""

from pathlib import Path

import yaml


def resolve_platform_profile(profile_name, explicit_path=None, search_from=None):
    if explicit_path:
        path = Path(explicit_path).expanduser().resolve()
        if path.is_file():
            return path
        raise FileNotFoundError(path)

    anchor = Path(search_from or __file__).resolve()
    for parent in anchor.parents:
        candidate = parent / "profiles" / "platforms" / f"{profile_name}.yaml"
        if candidate.is_file():
            return candidate
    return None


def load_platform_profile(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))["platform"]
    geometry = data["geometry"]
    footprint = geometry.get("navigation_footprint", geometry.get("footprint"))
    if not footprint:
        raise ValueError("platform profile has no footprint")
    normalized = [[float(value) for value in point] for point in footprint]
    if any(len(point) != 2 for point in normalized):
        raise ValueError("platform footprint points must contain x and y")
    width = max(point[1] for point in normalized) - min(
        point[1] for point in normalized
    )
    return {
        "name": str(data["name"]),
        "footprint": normalized,
        "robot_width": float(width),
        "min_turning_radius": float(geometry.get("min_turning_radius", 0.0)),
    }
