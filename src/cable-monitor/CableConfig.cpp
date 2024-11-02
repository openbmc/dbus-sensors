#include "CableConfig.hpp"

#include <nlohmann/json.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async/task.hpp>

#include <algorithm>
#include <fstream>
#include <string>

namespace phosphor::cable::config
{

PHOSPHOR_LOG2_USING;

// NOLINTNEXTLINE(readability-static-accessed-through-instance)
auto Config::parseConfigFile(std::string configFile)
    -> sdbusplus::async::task<json>
{
    std::ifstream jsonFile(configFile);
    if (!jsonFile.is_open())
    {
        error("Config file not found: {PATH}", "PATH", configFile);
        co_return {};
    }

    try
    {
        co_return json::parse(jsonFile, nullptr, true);
    }
    catch (const json::parse_error& e)
    {
        error("Failed to parse config file {PATH}: {ERROR}", "PATH", configFile,
              "ERROR", e);
    }

    co_return {};
}

// NOLINTNEXTLINE(readability-static-accessed-through-instance)
auto Config::processConfig(std::string configFile)
    -> sdbusplus::async::task<Cables>
{
    Cables cables{};
    auto jsonConfig = co_await Config::parseConfigFile(configFile);
    if (jsonConfig.empty())
    {
        co_return cables;
    }

    static constexpr auto connectedCablesProperty = "ConnectedCables";
    try
    {
        for (auto& entry : jsonConfig)
        {
            entry.at(connectedCablesProperty).get_to(cables);
        }
    }
    catch (const json::out_of_range& e)
    {
        error("Failed to find {PROPERTY} in config, {ERROR}", "PROPERTY",
              connectedCablesProperty, "ERROR", e.what());
        co_return cables;
    }

    Cables cablesTemp{};

    for (auto cable : cables)
    {
        std::replace(cable.begin(), cable.end(), ' ', '_');
        info("Config: Cable {NAME}", "NAME", cable);
        cablesTemp.insert(cable);
    }

    co_return cablesTemp;
}

} // namespace phosphor::cable::config
