#ifndef RELOCALIZATION_CORE__TYPES_HPP_
#define RELOCALIZATION_CORE__TYPES_HPP_

#include <memory>
#include <string>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace relocalization_core
{

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = CloudT::Ptr;
using CloudConstPtr = CloudT::ConstPtr;

enum class RegistrationBackendType
{
  kNdt,
  kIcp
};

enum class CropBoxFrameMode
{
  kDisabled,
  kScanLocal
};

enum class RelocalizationStatusCode
{
  kOk,
  kMapNotReady,
  kScanTooSmall,
  kBackendFailed,
  kFitnessRejected,
  kInvalidInitialGuess
};

inline std::string toString(RegistrationBackendType backend)
{
  switch (backend) {
    case RegistrationBackendType::kNdt:
      return "ndt";
    case RegistrationBackendType::kIcp:
      return "icp";
    default:
      return "unknown";
  }
}

inline std::string toString(CropBoxFrameMode mode)
{
  switch (mode) {
    case CropBoxFrameMode::kDisabled:
      return "disabled";
    case CropBoxFrameMode::kScanLocal:
      return "scan_local";
    default:
      return "unknown";
  }
}

inline std::string toString(RelocalizationStatusCode code)
{
  switch (code) {
    case RelocalizationStatusCode::kOk:
      return "ok";
    case RelocalizationStatusCode::kMapNotReady:
      return "map_not_ready";
    case RelocalizationStatusCode::kScanTooSmall:
      return "scan_too_small";
    case RelocalizationStatusCode::kBackendFailed:
      return "backend_failed";
    case RelocalizationStatusCode::kFitnessRejected:
      return "fitness_rejected";
    case RelocalizationStatusCode::kInvalidInitialGuess:
      return "invalid_initial_guess";
    default:
      return "unknown";
  }
}

struct RelocalizationRequest
{
  CloudConstPtr source_cloud;
  std::string source_frame_id;
  std::string target_frame_id;
  Eigen::Matrix4f initial_guess{Eigen::Matrix4f::Identity()};
  double request_time_sec{0.0};
  bool enable_debug_outputs{false};
};

struct RelocalizationDebugInfo
{
  std::size_t raw_scan_size{0U};
  std::size_t filtered_scan_size{0U};
  std::size_t cropped_scan_size{0U};
  std::size_t map_size{0U};
  std::size_t filtered_map_size{0U};
  Eigen::Matrix4f used_initial_guess{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f final_transform{Eigen::Matrix4f::Identity()};
  double backend_runtime_ms{0.0};
};

struct RelocalizationResult
{
  bool success{false};
  bool has_converged{false};
  Eigen::Matrix4f estimated_pose{Eigen::Matrix4f::Identity()};
  double fitness_score{0.0};
  RegistrationBackendType backend{RegistrationBackendType::kNdt};
  int iterations{0};
  RelocalizationStatusCode status_code{RelocalizationStatusCode::kBackendFailed};
  std::string status_message;
  CloudPtr aligned_cloud;
};

}  // namespace relocalization_core

#endif  // RELOCALIZATION_CORE__TYPES_HPP_
