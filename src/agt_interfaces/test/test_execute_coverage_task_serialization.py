from rclpy.serialization import deserialize_message, serialize_message

from agt_interfaces.action import ExecuteCoverageTask


def _round_trip(message):
    return deserialize_message(serialize_message(message), type(message))


def test_goal_serialization_round_trip():
    goal = ExecuteCoverageTask.Goal()
    goal.semantic_map_uri = "runtime/maps/greenhouse/semantic/map.geojson"
    goal.field_id = "field_001"
    goal.planning_mode = "annotated_rows"
    goal.controller_id = "FollowPath"
    goal.allow_repair = True

    restored = _round_trip(goal)
    assert restored.semantic_map_uri == goal.semantic_map_uri
    assert restored.field_id == goal.field_id
    assert restored.planning_mode == goal.planning_mode
    assert restored.controller_id == goal.controller_id
    assert restored.allow_repair is True


def test_result_serialization_round_trip():
    result = ExecuteCoverageTask.Result()
    result.success = True
    result.error_code = 0
    result.message = "completed"
    result.coverage_rate = 0.98
    result.overlap_rate = 0.04
    result.executed_length = 123.5
    result.repaired_segment_count = 2

    restored = _round_trip(result)
    assert restored.success is True
    assert restored.error_code == 0
    assert restored.message == "completed"
    assert restored.coverage_rate == result.coverage_rate
    assert restored.overlap_rate == result.overlap_rate
    assert restored.executed_length == result.executed_length
    assert restored.repaired_segment_count == 2


def test_feedback_serialization_round_trip():
    feedback = ExecuteCoverageTask.Feedback()
    feedback.current_stage = "VALIDATING_PATH"
    feedback.current_swath_index = 3
    feedback.total_swaths = 12
    feedback.distance_remaining = 42.25

    restored = _round_trip(feedback)
    assert restored.current_stage == "VALIDATING_PATH"
    assert restored.current_swath_index == 3
    assert restored.total_swaths == 12
    assert restored.distance_remaining == feedback.distance_remaining
