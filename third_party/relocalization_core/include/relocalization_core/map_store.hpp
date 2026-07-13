#ifndef RELOCALIZATION_CORE__MAP_STORE_HPP_
#define RELOCALIZATION_CORE__MAP_STORE_HPP_

#include <string>

#include "relocalization_core/types.hpp"

namespace relocalization_core
{

class MapStore
{
public:
  bool setMap(const CloudConstPtr & map, double voxel_leaf_size, const std::string & frame_id = "");
  bool loadFromPcd(
    const std::string & pcd_path, double voxel_leaf_size,
    const std::string & frame_id = "");

  bool hasMap() const;
  CloudConstPtr filteredMap() const;
  std::size_t rawSize() const;
  std::size_t filteredSize() const;
  const std::string & frameId() const;

private:
  CloudPtr maybeVoxelFilter(const CloudConstPtr & input, double voxel_leaf_size) const;
  CloudPtr loadPcdWithFallback(const std::string & pcd_path) const;

  CloudPtr raw_map_;
  CloudPtr filtered_map_;
  std::string frame_id_;
};

}  // namespace relocalization_core

#endif  // RELOCALIZATION_CORE__MAP_STORE_HPP_
