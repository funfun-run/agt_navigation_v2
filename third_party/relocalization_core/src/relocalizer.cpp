#include "relocalization_core/relocalizer.hpp"

#include "relocalization_core/backends/icp_backend.hpp"
#include "relocalization_core/backends/ndt_backend.hpp"

namespace relocalization_core
{

Relocalizer::Relocalizer()
: Relocalizer(RelocalizerConfig{})
{
}

Relocalizer::Relocalizer(const RelocalizerConfig & config)
: config_(config), scan_preprocessor_(config)
{
  rebuildBackend();
}

bool Relocalizer::setGlobalMap(const CloudConstPtr & map, const std::string & frame_id)
{
  const bool ok = map_store_.setMap(map, config_.map_voxel_leaf_size, frame_id);
  if (ok && backend_) {
    backend_->setTargetMap(map_store_.filteredMap());
  }
  return ok;
}

bool Relocalizer::setGlobalMapFromPcd(const std::string & pcd_path, const std::string & frame_id)
{
  const bool ok = map_store_.loadFromPcd(pcd_path, config_.map_voxel_leaf_size, frame_id);
  if (ok && backend_) {
    backend_->setTargetMap(map_store_.filteredMap());
  }
  return ok;
}

void Relocalizer::setConfig(const RelocalizerConfig & config)
{
  config_ = config;
  scan_preprocessor_.setConfig(config_);
  rebuildBackend();
  if (map_store_.hasMap() && backend_) {
    backend_->setTargetMap(map_store_.filteredMap());
  }
}

const RelocalizerConfig & Relocalizer::config() const
{
  return config_;
}

RelocalizationResult Relocalizer::relocalize(const RelocalizationRequest & request)
{
  RelocalizationResult result;
  result.backend = config_.backend;
  result.status_code = RelocalizationStatusCode::kBackendFailed;

  if (!map_store_.hasMap()) {
    result.status_code = RelocalizationStatusCode::kMapNotReady;
    result.status_message = "global map is not loaded";
    return result;
  }

  if (!request.source_cloud || request.source_cloud->empty()) {
    result.status_code = RelocalizationStatusCode::kScanTooSmall;
    result.status_message = "source cloud is empty";
    return result;
  }

  if (!request.initial_guess.allFinite()) {
    result.status_code = RelocalizationStatusCode::kInvalidInitialGuess;
    result.status_message = "initial guess contains non-finite values";
    return result;
  }

  const auto preprocessed = scan_preprocessor_.preprocess(request.source_cloud);
  latest_debug_info_.raw_scan_size = preprocessed.raw_size;
  latest_debug_info_.filtered_scan_size = preprocessed.filtered_size;
  latest_debug_info_.cropped_scan_size = preprocessed.cropped_size;
  latest_debug_info_.map_size = map_store_.rawSize();
  latest_debug_info_.filtered_map_size = map_store_.filteredSize();
  latest_debug_info_.used_initial_guess = request.initial_guess;

  if (!preprocessed.processed_cloud ||
    static_cast<int>(preprocessed.processed_cloud->size()) < config_.min_scan_points)
  {
    result.status_code = RelocalizationStatusCode::kScanTooSmall;
    result.status_message = "preprocessed scan has too few points";
    return result;
  }

  BackendAlignRequest backend_request;
  backend_request.source_cloud = preprocessed.processed_cloud;
  backend_request.initial_guess = request.initial_guess;
  backend_request.enable_debug_outputs = request.enable_debug_outputs;
  const auto backend_result = backend_->align(backend_request);

  latest_debug_info_.final_transform = backend_result.final_transformation;
  latest_debug_info_.backend_runtime_ms = backend_result.runtime_ms;

  result.has_converged = backend_result.has_converged;
  result.estimated_pose = backend_result.final_transformation;
  result.fitness_score = backend_result.fitness_score;
  result.iterations = backend_result.iterations;
  result.aligned_cloud = backend_result.aligned_cloud;

  if (!backend_result.has_converged) {
    result.status_code = RelocalizationStatusCode::kBackendFailed;
    result.status_message =
      backend_result.message.empty() ? "backend did not converge" : backend_result.message;
    return result;
  }

  if (backend_result.fitness_score > config_.fitness_score_threshold) {
    result.status_code = RelocalizationStatusCode::kFitnessRejected;
    result.status_message = "fitness score exceeds threshold";
    return result;
  }

  result.success = true;
  result.status_code = RelocalizationStatusCode::kOk;
  result.status_message = "ok";
  return result;
}

bool Relocalizer::hasMap() const
{
  return map_store_.hasMap();
}

RelocalizationDebugInfo Relocalizer::latestDebugInfo() const
{
  return latest_debug_info_;
}

void Relocalizer::rebuildBackend()
{
  if (config_.backend == RegistrationBackendType::kIcp) {
    backend_ = std::make_unique<IcpBackend>(config_);
  } else {
    backend_ = std::make_unique<NdtOmpBackend>(config_);
  }
}

}  // namespace relocalization_core
