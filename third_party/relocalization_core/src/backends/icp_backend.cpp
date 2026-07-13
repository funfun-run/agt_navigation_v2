#include "relocalization_core/backends/icp_backend.hpp"

#include <chrono>

namespace relocalization_core
{

IcpBackend::IcpBackend(const RelocalizerConfig & config)
: config_(config)
{
  registration_.setMaxCorrespondenceDistance(config_.max_correspondence_distance);
  registration_.setTransformationEpsilon(config_.transform_epsilon);
  registration_.setEuclideanFitnessEpsilon(config_.euclidean_fitness_epsilon);
  registration_.setMaximumIterations(config_.max_iterations);
}

RegistrationBackendType IcpBackend::type() const
{
  return RegistrationBackendType::kIcp;
}

void IcpBackend::setTargetMap(const CloudConstPtr & target_map)
{
  target_map_ = target_map;
  registration_.setInputTarget(target_map_);
}

BackendRunResult IcpBackend::align(const BackendAlignRequest & request)
{
  BackendRunResult result;
  registration_.setInputSource(request.source_cloud);

  CloudT aligned;
  const auto start = std::chrono::steady_clock::now();
  registration_.align(aligned, request.initial_guess);
  const auto finish = std::chrono::steady_clock::now();

  result.has_converged = registration_.hasConverged();
  result.final_transformation = registration_.getFinalTransformation();
  result.fitness_score = registration_.getFitnessScore();
  result.iterations = registration_.getMaximumIterations();
  result.runtime_ms =
    std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(finish - start).count();
  result.message = result.has_converged ? "ok" : "icp did not converge";
  if (request.enable_debug_outputs) {
    result.aligned_cloud.reset(new CloudT(aligned));
  }
  return result;
}

}  // namespace relocalization_core
