#include "osrm/customizer.hpp"
#include "customizer/customizer.hpp"
#include "osrm/customizer_config.hpp"

namespace osrm
{

// Pimpl-like facade

void customize(const CustomizationConfig &config,
               const std::vector<std::pair<std::size_t, std::string>> &period_speed_files)
{
    customizer::Customizer().Run(config, period_speed_files);
}

} // namespace osrm
