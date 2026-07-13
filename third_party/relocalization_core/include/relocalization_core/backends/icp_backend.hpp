#ifndef RELOCALIZATION_CORE__BACKENDS__ICP_BACKEND_HPP_
#define RELOCALIZATION_CORE__BACKENDS__ICP_BACKEND_HPP_

#include <pcl/registration/icp.h>

#include "relocalization_core/config.hpp"
#include "relocalization_core/registration_backend.hpp"

namespace relocalization_core
{

class IcpBackend : public RegistrationBackend
{
public:
  explicit IcpBackend(const RelocalizerConfig & config);

  RegistrationBackendType type() const override;
  void setTargetMap(const CloudConstPtr & target_map) override;
  BackendRunResult align(const BackendAlignRequest & request) override;

private:
  RelocalizerConfig config_;
  pcl::IterativeClosestPoint<PointT, PointT> registration_;
  CloudConstPtr target_map_;
};

}  // namespace relocalization_core

#endif  // RELOCALIZATION_CORE__BACKENDS__ICP_BACKEND_HPP_
