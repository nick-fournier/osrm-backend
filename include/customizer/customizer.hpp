#ifndef OSRM_CUSTOMIZE_CUSTOMIZER_HPP
#define OSRM_CUSTOMIZE_CUSTOMIZER_HPP

#include "customizer/customizer_config.hpp"

#include <string>
#include <vector>

namespace osrm::customizer
{

class Customizer
{
  public:
    // Customize cell metrics. If period_speed_files is non-empty, each entry is
    // (period_index, speed_csv_path) and metrics are written under period-indexed
    // TAR paths. If empty, uses config.updater_config.segment_speed_lookup_paths
    // as a single period (legacy behavior, writes legacy TAR paths).
    int Run(const CustomizationConfig &config,
            const std::vector<std::pair<std::size_t, std::string>> &period_speed_files = {});
};

} // namespace osrm::customizer

#endif // OSRM_CUSTOMIZE_CUSTOMIZER_HPP
