#ifndef RELOCALIZATION_CORE__SCAN_PREPROCESSOR_HPP_
#define RELOCALIZATION_CORE__SCAN_PREPROCESSOR_HPP_

#include "relocalization_core/config.hpp"

namespace relocalization_core
{

struct ScanPreprocessResult
{
  CloudPtr processed_cloud;
  std::size_t raw_size{0U};
  std::size_t filtered_size{0U};
  std::size_t cropped_size{0U};
};

class ScanPreprocessor
{
public:
  explicit ScanPreprocessor(const RelocalizerConfig & config);

  void setConfig(const RelocalizerConfig & config);
  ScanPreprocessResult preprocess(const CloudConstPtr & input) const;

private:
  CloudPtr maybeVoxelFilter(const CloudConstPtr & input) const;
  CloudPtr maybeCrop(const CloudConstPtr & input) const;

  RelocalizerConfig config_;
};

}  // namespace relocalization_core

#endif  // RELOCALIZATION_CORE__SCAN_PREPROCESSOR_HPP_
