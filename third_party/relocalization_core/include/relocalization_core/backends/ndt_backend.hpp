#ifndef RELOCALIZATION_CORE__BACKENDS__NDT_BACKEND_HPP_
#define RELOCALIZATION_CORE__BACKENDS__NDT_BACKEND_HPP_

#include <pclomp/ndt_omp.h>

#include "relocalization_core/config.hpp"
#include "relocalization_core/registration_backend.hpp"

namespace relocalization_core
{

class NdtOmpBackend : public RegistrationBackend
{
public:
  explicit NdtOmpBackend(const RelocalizerConfig & config);

  RegistrationBackendType type() const override;
  void setTargetMap(const CloudConstPtr & target_map) override;
  BackendRunResult align(const BackendAlignRequest & request) override;

private:
  pclomp::NeighborSearchMethod resolveSearchMethod() const;

  RelocalizerConfig config_;
  pclomp::NormalDistributionsTransform<PointT, PointT> registration_;
  CloudConstPtr target_map_;
};

}  // namespace relocalization_core

#endif  // RELOCALIZATION_CORE__BACKENDS__NDT_BACKEND_HPP_
