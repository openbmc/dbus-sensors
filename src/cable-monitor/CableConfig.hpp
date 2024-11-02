#pragma once

#include <sdbusplus/async/task.hpp>

#include <set>
#include <string>

namespace phosphor::cable::config
{

static constexpr auto configFileDir = "/var/lib/cablemonitor.d";
static constexpr auto configFileName = "cable-config.json";

class Config
{
  public:
    explicit Config() = default;

    using Cables = std::set<std::string>;

    /** Process the configuration file */
    static auto processConfig(std::string configFile)
        -> sdbusplus::async::task<Cables>;
};

} // namespace phosphor::cable::config
