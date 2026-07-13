#include "relocalization_core/scan_preprocessor.hpp"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

namespace relocalization_core
{

ScanPreprocessor::ScanPreprocessor(const RelocalizerConfig & config)
: config_(config)
{
}

void ScanPreprocessor::setConfig(const RelocalizerConfig & config)
{
  config_ = config;
}

ScanPreprocessResult ScanPreprocessor::preprocess(const CloudConstPtr & input) const
{
  ScanPreprocessResult result;
  result.raw_size = input ? input->size() : 0U;

  auto filtered = maybeVoxelFilter(input);
  result.filtered_size = filtered ? filtered->size() : 0U;

  auto cropped = maybeCrop(filtered);
  result.cropped_size = cropped ? cropped->size() : 0U;
  result.processed_cloud = cropped;
  return result;
}

CloudPtr ScanPreprocessor::maybeVoxelFilter(const CloudConstPtr & input) const
{
  CloudPtr output(new CloudT());
  if (!input) {
    return output;
  }
  if (config_.scan_voxel_leaf_size <= 0.0) {
    *output = *input;
    return output;
  }

  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(
    config_.scan_voxel_leaf_size, config_.scan_voxel_leaf_size, config_.scan_voxel_leaf_size);
  voxel.setInputCloud(input);
  voxel.filter(*output);
  return output;
}

CloudPtr ScanPreprocessor::maybeCrop(const CloudConstPtr & input) const
{
  CloudPtr output(new CloudT());
  if (!input) {
    return output;
  }

  if (!config_.crop_box.enabled || config_.crop_box.frame_mode == CropBoxFrameMode::kDisabled) {
    *output = *input;
    return output;
  }

  pcl::CropBox<PointT> crop_box;
  crop_box.setMin(
    Eigen::Vector4f(
      static_cast<float>(config_.crop_box.x_min), static_cast<float>(config_.crop_box.y_min),
      static_cast<float>(config_.crop_box.z_min), 1.0f));
  crop_box.setMax(
    Eigen::Vector4f(
      static_cast<float>(config_.crop_box.x_max), static_cast<float>(config_.crop_box.y_max),
      static_cast<float>(config_.crop_box.z_max), 1.0f));
  crop_box.setInputCloud(input);
  crop_box.filter(*output);
  return output;
}

}  // namespace relocalization_core
