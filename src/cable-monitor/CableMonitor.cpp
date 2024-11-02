#include "CableMonitor.hpp"

#include "CableConfig.hpp"
#include "Utils.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <sdbusplus/server/manager.hpp>

#include <algorithm>
#include <cstring>
#include <string>

PHOSPHOR_LOG2_USING;

namespace phosphor::cable::monitor
{

auto CableMonitor::processInventoryAdded(
    const sdbusplus::message::object_path& objectPath) -> void
{
    ctx.spawn(processCableAddedAsync(objectPath));
}

auto CableMonitor::processInventoryRemoved(
    const sdbusplus::message::object_path& objectPath) -> void
{
    ctx.spawn(processCableRemovedAsync(objectPath));
}

// NOLINTNEXTLINE(readability-static-accessed-through-instance)
auto CableMonitor::processConfigUpdate(std::string configFileName)
    -> sdbusplus::async::task<>
{
    // NOLINTNEXTLINE(clang-analyzer-core.uninitialized.Branch)
    if (strcmp(config::configFileName, configFileName.c_str()) != 0)
    {
        error("Update config file name {NAME} is not expected", "NAME",
              configFileName);
        co_return;
    }
    auto configFilePath =
        std::string(config::configFileDir) + "/" + configFileName;
    expectedCables = co_await config::Config::processConfig(configFilePath);
    if (expectedCables.empty())
    {
        error("No expected cables found in config file {NAME}", "NAME",
              configFileName);
        co_return;
    }
    co_await handleInventoryGet();
}

// NOLINTNEXTLINE(readability-static-accessed-through-instance)
auto CableMonitor::start() -> sdbusplus::async::task<>
{
    info("Start cable monitor");

    ctx.spawn(readNotifyAsync());
    ctx.spawn(handleInventoryAdded());
    ctx.spawn(handleInventoryRemoved());

    co_return;
}

// NOLINTNEXTLINE(readability-static-accessed-through-instance)
auto CableMonitor::processCableAddedAsync(
    sdbusplus::message::object_path objectPath) -> sdbusplus::async::task<>
{
    auto cableName = objectPath.filename();

    debug("Received cable added for {NAME}", "NAME", cableName);

    if (connectedCables.find(cableName) != connectedCables.end())
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
    else if (expectedCables.find(cableName) == expectedCables.end())
    {
        debug(
            "Cable {NAME} is not in expected cables, skip connected event generation",
            "NAME", cableName);
        co_return;
    }

    connectedCables.insert(cableName);
    co_await cableEvent.generateCableEvent(event::Event::Type::connected,
                                           cableName);
    debug("New cable {NAME} added", "NAME", cableName);

    co_return;
}

// NOLINTNEXTLINE(readability-static-accessed-through-instance)
auto CableMonitor::processCableRemovedAsync(
    sdbusplus::message::object_path objectPath) -> sdbusplus::async::task<>
{
    auto cableName = objectPath.filename();

    debug("Received cable removed for {NAME}", "NAME", cableName);

    if (expectedCables.empty())
    {
        debug("No expected cables yet, so skip cable add for {NAME}", "NAME",
              cableName);
        co_return;
    }
    else if (expectedCables.find(cableName) == expectedCables.end())
    {
        debug(
            "Cable {NAME} is not in expected cables, so skip disconnected event generation",
            "NAME", cableName);
        co_return;
    }
    else if (connectedCables.find(cableName) == connectedCables.end())
    {
        debug(
            "Cable {NAME} is not connected, so skip disconnected event generation",
            "NAME", cableName);
        co_return;
    }

    connectedCables.erase(cableName);
    co_await cableEvent.generateCableEvent(event::Event::Type::disconnected,
                                           cableName);
    debug("Removed cable {NAME}", "NAME", cableName);

    co_return;
}

} // namespace phosphor::cable::monitor

int main()
{
    constexpr auto path = "/xyz/openbmc_project/cable_monitor";
    constexpr auto serviceName = "xyz.openbmc_project.CableMonitor";
    sdbusplus::async::context ctx;
    sdbusplus::server::manager_t manager{ctx, path};

    info("Creating cable monitor");
    using CableMonitorIntf = phosphor::cable::monitor::CableMonitor;
    CableMonitorIntf cableMonitor{ctx};
    ctx.request_name(serviceName);

    ctx.run();
    return 0;
}
