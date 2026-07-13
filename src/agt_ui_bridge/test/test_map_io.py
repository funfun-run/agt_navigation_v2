import importlib.util
from pathlib import Path

import numpy as np
from nav_msgs.msg import OccupancyGrid


SCRIPT = Path(__file__).parents[1] / "scripts" / "map_io_bridge.py"
SPEC = importlib.util.spec_from_file_location("map_io_bridge", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_nav2_map_round_trip(tmp_path):
    source = OccupancyGrid()
    source.header.frame_id = "map"
    source.info.resolution = 0.1
    source.info.width = 3
    source.info.height = 2
    source.info.origin.position.x = -1.5
    source.info.origin.position.y = 2.0
    source.info.origin.orientation.w = 1.0
    source.data = [0, 100, -1, 100, 0, -1]

    yaml_path = MODULE.save_occupancy_map(source, str(tmp_path / "edited"), "pgm", 0.25, 0.65)
    loaded = MODULE.load_occupancy_map(yaml_path, "map")

    assert loaded.info.width == source.info.width
    assert loaded.info.height == source.info.height
    assert np.array_equal(loaded.data, source.data)
    assert loaded.info.origin.position.x == source.info.origin.position.x
