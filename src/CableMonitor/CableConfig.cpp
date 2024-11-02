#include "CableConfig.hpp"

#include <nlohmann/json.hpp>
#include <phosphor-logging/lg2.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace phosphor::cable::config
{

PHOSPHOR_LOG2_USING;

namespace fs = std::filesystem;
using json = nlohmann::json;

static constexpr auto configFileDir = "/var/lib/cable-monitor";
static constexpr auto configFileName = "cable-config.json";

auto Config::setupConfigNotify() -> bool
{
    std::error_code ec;

    fs::path configDirPath(configFileDir);
    if (!fs::is_directory(configDirPath, ec))
    {
        fs::create_directories(configFileDir, ec);
    }

    fd = inotify_init1(IN_NONBLOCK);
    if (-1 == fd)
    {
        error("inotify_init1 failed, {ERROR}", "ERROR", errno);
        return false;
    }

    wd = inotify_add_watch(fd, configFileDir, IN_CLOSE_WRITE);
    if (-1 == wd)
    {
        close(fd);
        error("inotify_add_watch failed, {ERROR}", "ERROR", errno);
        return false;
    }

    return true;
}

auto Config::stopConfigNotify() -> void
{
    if (-1 != fd)
    {
        if (-1 != wd)
        {
            inotify_rm_watch(fd, wd);
        }
        close(fd);
    }
}

auto parseConfigFile(std::string& configFile) -> sdbusplus::async::task<json>
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

auto Config::processConfig(std::string& configFile)
    -> sdbusplus::async::task<Cables>
{
    Cables cables{};
    auto jsonConfig = co_await parseConfigFile(configFile);
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
        debug("Config: Cable {NAME}", "NAME", cable);
        cablesTemp.insert(cable);
    }

    co_return cablesTemp;
}

auto Config::getExpectedCableConfig() -> sdbusplus::async::task<Cables>
{
    co_await sdbusplus::async::async_fdio(ctx, fd);

    constexpr auto maxBytes = 1024;
    uint8_t buffer[maxBytes];
    Cables cables{};

    auto bytes = read(fd, buffer, maxBytes);
    if (0 > bytes)
    {
        error("Failed to read inotify event, {ERROR}", "ERROR", errno);
        co_return cables;
    }
    auto offset = 0;
    while (offset < bytes)
    {
        auto event = reinterpret_cast<inotify_event*>(&buffer[offset]);
        if ((event->mask & IN_CLOSE_WRITE) && !(event->mask & IN_ISDIR))
        {
            if (!strcmp(event->name, configFileName))
            {
                auto configFile =
                    std::string(configFileDir) + "/" + configFileName;
                co_return co_await processConfig(configFile);
            }
        }

        offset += offsetof(inotify_event, name) + event->len;
    }

    co_return cables;
}

} // namespace phosphor::cable::config
