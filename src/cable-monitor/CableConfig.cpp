#include "CableConfig.hpp"

#include <nlohmann/json.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>

#include <algorithm>
#include <exception>
#include <fstream>
#include <string>

namespace cable
{

PHOSPHOR_LOG2_USING;

using json = nlohmann::json;

static auto parseConfigFile(std::string configFile) -> json
{
    std::ifstream jsonFile(configFile);
    if (!jsonFile.is_open())
    {
        error("Config file not found: {PATH}", "PATH", configFile);
        return {};
    }

    auto jsData = json::parse(jsonFile, nullptr, false);
    if (jsData.is_discarded())
    {
        error("Failed to parse config file {PATH}", "PATH", configFile);
        return {};
    }

    return jsData;
}

auto Config::processConfig(std::string configFile)
    -> sdbusplus::async::task<Config::Cables>
{
    Cables cables{};
    auto jsonConfig = parseConfigFile(configFile);
    if (jsonConfig.empty())
    {
        co_return cables;
    }

    static constexpr auto connectedCablesProperty = "ConnectedCables";
    try
    {
        jsonConfig.at(connectedCablesProperty).get_to(cables);
    }
    catch (const std::exception& e)
    {
        error("Failed to find {PROPERTY} in config, {ERROR}", "PROPERTY",
              connectedCablesProperty, "ERROR", e);
        co_return cables;
    }

    Cables cablesTemp{};

    for (auto cable : cables)
    {
        std::replace(cable.begin(), cable.end(), ' ', '_');
        debug("Config: Cable {NAME}", "NAME", cable);
        cablesTemp.insert(cable);
    }

    co_return cablesTemp;
}

} // namespace cable
