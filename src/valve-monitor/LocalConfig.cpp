#include "LocalConfig.hpp"

#include <nlohmann/json.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>

#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <string>
#include <utility>

namespace valve
{

PHOSPHOR_LOG2_USING;

static constexpr auto configFileDir = "/var/lib/valvemonitor";
static constexpr auto configFileName = "valve-config.json";

namespace fs = std::filesystem;
using json = nlohmann::json;

LocalConfig::LocalConfig(sdbusplus::async::context& ctx) :
    ctx(ctx),
    notifyWatch(ctx, configFileDir,
                std::bind_front(&LocalConfig::configUpdateHandler, this))
{}

auto LocalConfig::start() -> sdbusplus::async::task<>
{
    ctx.spawn(notifyWatch.readNotifyAsync());

    auto configFilePath = std::string(configFileDir) + "/" + configFileName;
    if (fs::exists(configFilePath))
    {
        loadConfig();
    }

    co_return;
}

auto LocalConfig::getFlowRate(const std::string& valveName) const -> double
{
    static constexpr double defaultFlowRateLPM = 150.0;

    auto it = flowRateMap.find(valveName);
    if (it != flowRateMap.end())
    {
        return it->second;
    }
    return defaultFlowRateLPM;
}

auto LocalConfig::loadConfig() -> void
{
    auto configFilePath = std::string(configFileDir) + "/" + configFileName;

    std::ifstream jsonFile(configFilePath);
    if (!jsonFile.is_open())
    {
        error("Valve config file not found: {PATH}", "PATH", configFilePath);
        return;
    }

    // JSON_NOEXCEPTION is enabled; report invalid input instead of aborting.
    const auto jsonConfig = json::parse(jsonFile, nullptr, false);
    if (jsonConfig.is_discarded())
    {
        error("Failed to parse valve config {PATH}", "PATH", configFilePath);
        return;
    }

    if (jsonConfig.empty())
    {
        warning("Empty valve config file: {PATH}", "PATH", configFilePath);
        return;
    }

    static constexpr auto valvesProperty = "Valves";
    if (!jsonConfig.contains(valvesProperty) ||
        !jsonConfig.at(valvesProperty).is_array())
    {
        error("Invalid Valves in valve config {PATH}", "PATH", configFilePath);
        return;
    }

    std::map<std::string, double> newMap;
    for (const auto& valve : jsonConfig.at(valvesProperty))
    {
        // Check keys and types before at()/get(), which otherwise abort.
        if (!valve.contains("Name") || !valve.at("Name").is_string() ||
            !valve.contains("FlowRate") || !valve.at("FlowRate").is_number())
        {
            error("Invalid Name or FlowRate in valve config {PATH}", "PATH",
                  configFilePath);
            return;
        }

        auto name = valve.at("Name").get<std::string>();
        auto flowRate = valve.at("FlowRate").get<double>();
        newMap[name] = flowRate;
        debug("Valve config: {NAME} flow-rate={RATE}", "NAME", name, "RATE",
              flowRate);
    }

    flowRateMap = std::move(newMap);
    info("Valve config loaded with {COUNT} entries", "COUNT",
         flowRateMap.size());
}

auto LocalConfig::configUpdateHandler(std::string fileName)
    -> sdbusplus::async::task<>
{
    if (strcmp(configFileName, fileName.c_str()) != 0)
    {
        error("Update config file name {NAME} is not expected", "NAME",
              fileName);
        co_return;
    }

    auto configFilePath = std::string(configFileDir) + "/" + fileName;
    if (!fs::exists(configFilePath))
    {
        error("Config file {NAME} does not exist", "NAME", configFilePath);
        co_return;
    }

    loadConfig();
}

} // namespace valve
