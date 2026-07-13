#ifndef RELOCALIZATION_CORE__CONFIG_HPP_
#define RELOCALIZATION_CORE__CONFIG_HPP_

#include "relocalization_core/types.hpp"

namespace relocalization_core
{

struct CropBoxConfig
{
  bool enabled{true};
  CropBoxFrameMode frame_mode{CropBoxFrameMode::kScanLocal};
  double x_min{0.0};
  double x_max{30.0};
  double y_min{-15.0};
  double y_max{15.0};
  double z_min{-2.0};
  double z_max{2.0};
};

struct NdtConfig
{
  double resolution{1.0};
  double step_size{0.1};
  int num_threads{0};
  int search_method{2};  // pclomp::DIRECT7
};

struct RelocalizerConfig
{
  RegistrationBackendType backend{RegistrationBackendType::kNdt};
  double map_voxel_leaf_size{0.25};
  double scan_voxel_leaf_size{0.25};
  int min_scan_points{200};
  CropBoxConfig crop_box{};
  double fitness_score_threshold{2.0};
  int max_iterations{100};
  double transform_epsilon{1e-6};
  double euclidean_fitness_epsilon{1e-6};
  double max_correspondence_distance{3.0};
  NdtConfig ndt{};
};

}  // namespace relocalization_core

#endif  // RELOCALIZATION_CORE__CONFIG_HPP_
