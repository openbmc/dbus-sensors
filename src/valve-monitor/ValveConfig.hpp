#pragma once

#include "NotifyWatch.hpp"

#include <sdbusplus/async.hpp>

#include <map>
#include <string>

namespace valve
{

class LocalConfig
{
  public:
    LocalConfig() = delete;
    1

        explicit LocalConfig(sdbusplus::async::context& ctx);

    /** @brief Start watching for config file changes */
    auto start() -> sdbusplus::async::task<>;

    /** @brief Get the flow-rate for a given valve name
     *  @param valveName - name of the valve
     *  @return flow-rate in LPM, defaults to 150 LPM if not configured
     */
    auto getFlowRate(const std::string& valveName) const -> double;

  private:
    /** @brief Parse and load the config file into the map */
    auto loadConfig() -> void;

    /** @brief Callback handler for async updates to config file */
    auto configUpdateHandler(std::string fileName) -> sdbusplus::async::task<>;

    sdbusplus::async::context& ctx;

    /** @brief Map of valve name to flow-rate */
    std::map<std::string, double> flowRateMap;

    notify_watch::NotifyWatch notifyWatch;
};

} // namespace valve
