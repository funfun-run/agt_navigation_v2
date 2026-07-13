#include "relocalization_core/map_store.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace relocalization_core
{

bool MapStore::setMap(
  const CloudConstPtr & map, double voxel_leaf_size,
  const std::string & frame_id)
{
  if (!map || map->empty()) {
    raw_map_.reset();
    filtered_map_.reset();
    frame_id_.clear();
    return false;
  }

  raw_map_.reset(new CloudT(*map));
  filtered_map_ = maybeVoxelFilter(raw_map_, voxel_leaf_size);
  frame_id_ = frame_id;
  return filtered_map_ && !filtered_map_->empty();
}

bool MapStore::loadFromPcd(
  const std::string & pcd_path, double voxel_leaf_size,
  const std::string & frame_id)
{
  const auto loaded = loadPcdWithFallback(pcd_path);
  return setMap(loaded, voxel_leaf_size, frame_id);
}

bool MapStore::hasMap() const
{
  return filtered_map_ && !filtered_map_->empty();
}

CloudConstPtr MapStore::filteredMap() const
{
  return filtered_map_;
}

std::size_t MapStore::rawSize() const
{
  return raw_map_ ? raw_map_->size() : 0U;
}

std::size_t MapStore::filteredSize() const
{
  return filtered_map_ ? filtered_map_->size() : 0U;
}

const std::string & MapStore::frameId() const
{
  return frame_id_;
}

CloudPtr MapStore::maybeVoxelFilter(const CloudConstPtr & input, double voxel_leaf_size) const
{
  CloudPtr output(new CloudT());
  if (!input) {
    return output;
  }

  if (voxel_leaf_size <= 0.0) {
    *output = *input;
    return output;
  }

  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel.setInputCloud(input);
  voxel.filter(*output);
  return output;
}

CloudPtr MapStore::loadPcdWithFallback(const std::string & pcd_path) const
{
  CloudPtr cloud(new CloudT());
  if (pcl::io::loadPCDFile<PointT>(pcd_path, *cloud) == 0) {
    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, xyz_cloud) == 0) {
    cloud->reserve(xyz_cloud.size());
    for (const auto & point : xyz_cloud.points) {
      PointT converted;
      converted.x = point.x;
      converted.y = point.y;
      converted.z = point.z;
      converted.intensity = 0.0f;
      cloud->push_back(converted);
    }
  }
  return cloud;
}

}  // namespace relocalization_core
