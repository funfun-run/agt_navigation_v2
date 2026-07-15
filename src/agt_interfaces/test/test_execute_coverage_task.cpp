#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "agt_interfaces/action/execute_coverage_task.hpp"

TEST(ExecuteCoverageTaskInterface, GeneratedTypesAreUsable)
{
  agt_interfaces::action::ExecuteCoverageTask::Goal goal;
  goal.semantic_map_uri = "semantic_map.geojson";
  goal.field_id = "field_001";
  goal.planning_mode = "polygon";
  goal.controller_id = "FollowPath";
  goal.allow_repair = true;

  agt_interfaces::action::ExecuteCoverageTask::Result result;
  result.success = true;
  result.error_code = 0U;
  result.repaired_segment_count = 1U;

  agt_interfaces::action::ExecuteCoverageTask::Feedback feedback;
  feedback.current_stage = "READY";
  feedback.current_swath_index = 0U;
  feedback.total_swaths = 8U;

  EXPECT_EQ(goal.field_id, std::string("field_001"));
  EXPECT_TRUE(goal.allow_repair);
  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.error_code, static_cast<std::uint16_t>(0U));
  EXPECT_EQ(result.repaired_segment_count, static_cast<std::uint32_t>(1U));
  EXPECT_EQ(feedback.current_stage, std::string("READY"));
  EXPECT_EQ(feedback.total_swaths, static_cast<std::uint32_t>(8U));
}
