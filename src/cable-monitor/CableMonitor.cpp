#include "CableMonitor.hpp"

#include "CableConfig.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <sdbusplus/server/manager.hpp>

#include <cstring>
#include <filesystem>
#include <functional>
#include <string>

PHOSPHOR_LOG2_USING;

namespace cable
{

Monitor::Monitor(sdbusplus::async::context& ctx) :
    ctx(ctx), cableEvents(ctx),
    entityManager(ctx, {CableInventoryIntf::interface},
                  std::bind_front(&Monitor::processInventoryAdded, this),
                  std::bind_front(&Monitor::processInventoryRemoved, this)),
    notifyWatch(ctx, Config::configFileDir,
                std::bind_front(&Monitor::processConfigUpdate, this))
{
    ctx.spawn(start());
}

void Monitor::processInventoryAdded(
    const sdbusplus::message::object_path& objectPath,
    const std::string& /*unused*/)
{
    info("Received cable added for {NAME}", "NAME", objectPath.filename());
    ctx.spawn(processCableAddedAsync(objectPath));
}

void Monitor::processInventoryRemoved(
    const sdbusplus::message::object_path& objectPath,
    const std::string& /*unused*/)
{
    info("Received cable removed for {NAME}", "NAME", objectPath.filename());
    ctx.spawn(processCableRemovedAsync(objectPath));
}

sdbusplus::async::task<> Monitor::processConfigUpdate(
    std::string configFileName)
{
    // NOLINTNEXTLINE(clang-analyzer-core.uninitialized.Branch)
    if (strcmp(Config::configFileName, configFileName.c_str()) != 0)
    {
        error("Update config file name {NAME} is not expected", "NAME",
              configFileName);
        co_return;
    }
    auto configFilePath =
        std::string(Config::configFileDir) + "/" + configFileName;
    if (!std::filesystem::exists(configFilePath))
    {
        error("Config file {NAME} does not exist", "NAME", configFilePath);
        co_return;
    }
    expectedCables = co_await Config::processConfig(configFilePath);
    if (expectedCables.empty())
    {
        error("No expected cables found in config file {NAME}", "NAME",
              configFileName);
        co_return;
    }
    co_await entityManager.handleInventoryGet();
}

sdbusplus::async::task<> Monitor::start()
{
    info("Start cable monitor");

    ctx.spawn(notifyWatch.readNotifyAsync());
    ctx.spawn(entityManager.handleInventoryAdded());
    ctx.spawn(entityManager.handleInventoryRemoved());

    co_return;
}

sdbusplus::async::task<> Monitor::processCableAddedAsync(
    sdbusplus::message::object_path objectPath)
{
    auto cableName = objectPath.filename();

    debug("Received cable added for {NAME}", "NAME", cableName);

    if (connectedCables.contains(cableName))
    {
        debug("Cable {NAME} is already connected, so skip it", "NAME",
              cableName);
        co_return;
    }
    else if (expectedCables.empty())
    {
        debug("No expected cables yet, so skip cable add for {NAME}", "NAME",
              cableName);
        co_return;
    }
    else if (!expectedCables.contains(cableName))
    {
        debug(
            "Cable {NAME} is not in expected cables, skip connected event generation",
            "NAME", cableName);
        co_return;
    }

    connectedCables.insert(cableName);
    co_await cableEvents.generateCableEvent(Events::Type::connected, cableName);
    debug("New cable {NAME} added", "NAME", cableName);

    co_return;
}

sdbusplus::async::task<> Monitor::processCableRemovedAsync(
    sdbusplus::message::object_path objectPath)
{
    auto cableName = objectPath.filename();

    debug("Received cable removed for {NAME}", "NAME", cableName);

    if (expectedCables.empty())
    {
        debug("No expected cables yet, so skip cable add for {NAME}", "NAME",
              cableName);
        co_return;
    }
    else if (!expectedCables.contains(cableName))
    {
        debug(
            "Cable {NAME} is not in expected cables, so skip disconnected event generation",
            "NAME", cableName);
        co_return;
    }
    else if (!connectedCables.contains(cableName))
    {
        debug(
            "Cable {NAME} is not connected, so skip disconnected event generation",
            "NAME", cableName);
        co_return;
    }

    connectedCables.erase(cableName);
    co_await cableEvents.generateCableEvent(Events::Type::disconnected,
                                            cableName);
    debug("Removed cable {NAME}", "NAME", cableName);

    co_return;
}

} // namespace cable

int main()
{
    constexpr auto path = "/xyz/openbmc_project/cable_monitor";
    constexpr auto serviceName = "xyz.openbmc_project.cablemonitor";
    sdbusplus::async::context ctx;
    sdbusplus::server::manager_t manager{ctx, path};

    info("Creating cable monitor");
    cable::Monitor cableMonitor{ctx};
    ctx.request_name(serviceName);

    ctx.run();
    return 0;
}
