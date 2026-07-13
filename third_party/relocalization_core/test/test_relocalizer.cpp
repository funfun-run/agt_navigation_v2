#include <cmath>
#include <random>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>

#include "relocalization_core/relocalizer.hpp"

using relocalization_core::CloudT;
using relocalization_core::PointT;
using relocalization_core::RegistrationBackendType;
using relocalization_core::RelocalizationRequest;
using relocalization_core::RelocalizationStatusCode;
using relocalization_core::Relocalizer;
using relocalization_core::RelocalizerConfig;

namespace
{

CloudT::Ptr makeStructuredCloud()
{
  auto cloud = std::make_shared<CloudT>();
  for (int x = -4; x <= 4; ++x) {
    for (int y = -4; y <= 4; ++y) {
      for (int z = -1; z <= 1; ++z) {
        PointT point;
        point.x = static_cast<float>(x) * 0.5f;
        point.y = static_cast<float>(y) * 0.4f;
        point.z = static_cast<float>(z) * 0.3f + static_cast<float>(x + y) * 0.01f;
        point.intensity = static_cast<float>((x + 5) * (y + 5));
        cloud->push_back(point);
      }
    }
  }
  return cloud;
}

CloudT::Ptr makeAsymmetricCloud()
{
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> dx(-3.0f, 2.0f);
  std::uniform_real_distribution<float> dy(-1.0f, 4.0f);
  std::uniform_real_distribution<float> dz(-0.3f, 1.7f);

  auto cloud = std::make_shared<CloudT>();
  cloud->reserve(600);
  for (int i = 0; i < 600; ++i) {
    PointT point;
    point.x = dx(rng) + 0.2f * std::sin(0.7f * static_cast<float>(i));
    point.y = dy(rng) + 0.1f * std::cos(0.3f * static_cast<float>(i));
    point.z = dz(rng) + 0.05f * std::sin(0.11f * static_cast<float>(i));
    point.intensity = static_cast<float>(i % 17);
    cloud->push_back(point);
  }
  return cloud;
}

Eigen::Matrix4f makeTransform(float tx, float ty, float tz, float yaw)
{
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  transform(0, 3) = tx;
  transform(1, 3) = ty;
  transform(2, 3) = tz;
  return transform;
}

}  // namespace

TEST(RelocalizerTest, ReturnsMapNotReadyWhenMapMissing)
{
  Relocalizer relocalizer;
  RelocalizationRequest request;
  request.source_cloud = makeStructuredCloud();

  const auto result = relocalizer.relocalize(request);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.status_code, RelocalizationStatusCode::kMapNotReady);
}

TEST(RelocalizerTest, ReturnsScanTooSmallAfterPreprocessing)
{
  RelocalizerConfig config;
  config.min_scan_points = 50;
  Relocalizer relocalizer(config);
  relocalizer.setGlobalMap(makeStructuredCloud());

  auto tiny_scan = std::make_shared<CloudT>();
  for (int i = 0; i < 10; ++i) {
    PointT point;
    point.x = static_cast<float>(i);
    point.y = 0.0f;
    point.z = 0.0f;
    point.intensity = 0.0f;
    tiny_scan->push_back(point);
  }

  RelocalizationRequest request;
  request.source_cloud = tiny_scan;
  request.initial_guess = Eigen::Matrix4f::Identity();

  const auto result = relocalizer.relocalize(request);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.status_code, RelocalizationStatusCode::kScanTooSmall);
}

TEST(RelocalizerTest, SupportsNdtAndIcpBackends)
{
  const auto target = makeAsymmetricCloud();
  const Eigen::Matrix4f expected = makeTransform(0.35f, -0.2f, 0.08f, 0.05f);
  const Eigen::Matrix4f source_from_target = expected.inverse();
  auto source = std::make_shared<CloudT>();
  pcl::transformPointCloud(*target, *source, source_from_target);

  for (const auto backend : {RegistrationBackendType::kNdt, RegistrationBackendType::kIcp}) {
    RelocalizerConfig config;
    config.backend = backend;
    config.map_voxel_leaf_size = 0.0;
    config.scan_voxel_leaf_size = 0.0;
    config.min_scan_points = 20;
    config.crop_box.enabled = false;
    config.fitness_score_threshold = 1.0;
    config.max_iterations = 120;
    config.max_correspondence_distance = 2.0;
    config.ndt.resolution = 1.0;
    config.ndt.step_size = 0.1;
    config.ndt.num_threads = 1;
    Relocalizer relocalizer(config);
    ASSERT_TRUE(relocalizer.setGlobalMap(target));

    RelocalizationRequest request;
    request.source_cloud = source;
    request.initial_guess = makeTransform(0.32f, -0.18f, 0.08f, 0.04f);

    const auto result = relocalizer.relocalize(request);
    EXPECT_TRUE(result.success) << static_cast<int>(backend) << " " << result.status_message;
    EXPECT_TRUE(result.has_converged);
    EXPECT_NEAR(result.estimated_pose(0, 3), expected(0, 3), 0.2);
    EXPECT_NEAR(result.estimated_pose(1, 3), expected(1, 3), 0.2);
    EXPECT_NEAR(result.estimated_pose(2, 3), expected(2, 3), 0.2);
    const float yaw = std::atan2(result.estimated_pose(1, 0), result.estimated_pose(0, 0));
    EXPECT_NEAR(yaw, 0.05f, 0.2f);
  }
}
