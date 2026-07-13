#ifndef RELOCALIZATION_CORE__REGISTRATION_BACKEND_HPP_
#define RELOCALIZATION_CORE__REGISTRATION_BACKEND_HPP_

#include <string>

#include <Eigen/Core>

#include "relocalization_core/types.hpp"

namespace relocalization_core
{

struct BackendAlignRequest
{
  CloudConstPtr source_cloud;
  Eigen::Matrix4f initial_guess{Eigen::Matrix4f::Identity()};
  bool enable_debug_outputs{false};
};

struct BackendRunResult
{
  bool has_converged{false};
  Eigen::Matrix4f final_transformation{Eigen::Matrix4f::Identity()};
  double fitness_score{0.0};
  int iterations{0};
  double runtime_ms{0.0};
  std::string message;
  CloudPtr aligned_cloud;
};

class RegistrationBackend
{
public:
  virtual ~RegistrationBackend() = default;

  virtual RegistrationBackendType type() const = 0;
  virtual void setTargetMap(const CloudConstPtr & target_map) = 0;
  virtual BackendRunResult align(const BackendAlignRequest & request) = 0;
};

}  // namespace relocalization_core

#endif  // RELOCALIZATION_CORE__REGISTRATION_BACKEND_HPP_
