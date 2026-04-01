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

    json jsonConfig;
    try
    {
        jsonConfig = json::parse(jsonFile, nullptr, true);
    }
    catch (const json::parse_error& e)
    {
        error("Failed to parse valve config {PATH}: {ERROR}", "PATH",
              configFilePath, "ERROR", e);
        return;
    }

    if (jsonConfig.empty())
    {
        warning("Empty valve config file: {PATH}", "PATH", configFilePath);
        return;
    }

    static constexpr auto valvesProperty = "Valves";
    std::map<std::string, double> newMap;

    try
    {
        for (const auto& valve : jsonConfig.at(valvesProperty))
        {
            auto name = valve.at("Name").get<std::string>();
            auto flowRate = valve.at("FlowRate").get<double>();
            newMap[name] = flowRate;
            debug("Valve config: {NAME} flow-rate={RATE}", "NAME", name, "RATE",
                  flowRate);
        }
    }
    catch (const std::exception& e)
    {
        error("Failed to parse valve config: {ERROR}", "ERROR", e);
        return;
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
