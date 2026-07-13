#include "relocalization_core/backends/ndt_backend.hpp"

#include <chrono>

namespace relocalization_core
{

NdtOmpBackend::NdtOmpBackend(const RelocalizerConfig & config)
: config_(config)
{
  registration_.setResolution(static_cast<float>(config_.ndt.resolution));
  registration_.setStepSize(config_.ndt.step_size);
  registration_.setTransformationEpsilon(config_.transform_epsilon);
  registration_.setEuclideanFitnessEpsilon(config_.euclidean_fitness_epsilon);
  registration_.setMaximumIterations(config_.max_iterations);
  registration_.setNumThreads(config_.ndt.num_threads);
  registration_.setNeighborhoodSearchMethod(resolveSearchMethod());
}

RegistrationBackendType NdtOmpBackend::type() const
{
  return RegistrationBackendType::kNdt;
}

void NdtOmpBackend::setTargetMap(const CloudConstPtr & target_map)
{
  target_map_ = target_map;
  registration_.setInputTarget(target_map_);
}

BackendRunResult NdtOmpBackend::align(const BackendAlignRequest & request)
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
  result.message = result.has_converged ? "ok" : "ndt did not converge";
  if (request.enable_debug_outputs) {
    result.aligned_cloud.reset(new CloudT(aligned));
  }
  return result;
}

pclomp::NeighborSearchMethod NdtOmpBackend::resolveSearchMethod() const
{
  switch (config_.ndt.search_method) {
    case 0:
      return pclomp::KDTREE;
    case 1:
      return pclomp::DIRECT26;
    case 2:
      return pclomp::DIRECT7;
    case 3:
      return pclomp::DIRECT1;
    default:
      return pclomp::DIRECT7;
  }
}

}  // namespace relocalization_core
