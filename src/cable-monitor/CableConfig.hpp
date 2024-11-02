#pragma once

#include <sdbusplus/async/task.hpp>

#include <set>
#include <string>

namespace cable
{

class Config
{
  public:
    static constexpr auto configFileDir = "/var/lib/cablemonitor.d";
    static constexpr auto configFileName = "cable-config.json";
    explicit Config() = default;

    using Cables = std::set<std::string>;

    /** Process the configuration file */
    static sdbusplus::async::task<Cables> processConfig(std::string configFile);
};

} // namespace cable
