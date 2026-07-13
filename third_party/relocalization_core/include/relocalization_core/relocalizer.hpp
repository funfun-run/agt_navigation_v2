#ifndef RELOCALIZATION_CORE__RELOCALIZER_HPP_
#define RELOCALIZATION_CORE__RELOCALIZER_HPP_

#include <memory>

#include "relocalization_core/config.hpp"
#include "relocalization_core/map_store.hpp"
#include "relocalization_core/registration_backend.hpp"
#include "relocalization_core/scan_preprocessor.hpp"

namespace relocalization_core
{

class Relocalizer
{
public:
  Relocalizer();
  explicit Relocalizer(const RelocalizerConfig & config);

  bool setGlobalMap(const CloudConstPtr & map, const std::string & frame_id = "");
  bool setGlobalMapFromPcd(const std::string & pcd_path, const std::string & frame_id = "");

  void setConfig(const RelocalizerConfig & config);
  const RelocalizerConfig & config() const;

  RelocalizationResult relocalize(const RelocalizationRequest & request);
  bool hasMap() const;
  RelocalizationDebugInfo latestDebugInfo() const;

private:
  void rebuildBackend();

  RelocalizerConfig config_;
  MapStore map_store_;
  ScanPreprocessor scan_preprocessor_;
  std::unique_ptr<RegistrationBackend> backend_;
  RelocalizationDebugInfo latest_debug_info_{};
};

}  // namespace relocalization_core

#endif  // RELOCALIZATION_CORE__RELOCALIZER_HPP_
