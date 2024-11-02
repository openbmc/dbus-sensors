#pragma once

#include <sdbusplus/async.hpp>

#include <set>
#include <string>

namespace cable
{

class Config
{
  public:
    static constexpr auto configFileDir = "/var/lib/cablemonitor";
    static constexpr auto configFileName = "cable-config.json";
    explicit Config() = default;

    using Cables = std::set<std::string>;

    /** Process the configuration file */
    static auto processConfig(std::string configFile)
        -> sdbusplus::async::task<Cables>;
};

} // namespace cable
