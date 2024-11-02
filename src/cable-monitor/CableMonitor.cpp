#include "CableMonitor.hpp"

#include "Utils.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <sdbusplus/server/manager.hpp>
#include <xyz/openbmc_project/Inventory/Item/Cable/client.hpp>

#include <algorithm>

PHOSPHOR_LOG2_USING;

namespace phosphor::cable::monitor
{

using CableInventoryIntf =
    sdbusplus::client::xyz::openbmc_project::inventory::item::Cable<>;

auto CableMonitor::processInventoryAdded(
    const sdbusplus::message::object_path& objectPath,
    const SensorData& inventoryData) -> void
{
    auto cableData = inventoryData.find(CableInventoryIntf::interface);
    if (cableData == inventoryData.end())
    {
        return;
    }
    ctx.spawn(processCableAddedAsync(objectPath));
}

auto CableMonitor::processInventoryRemoved(
    const sdbusplus::message::object_path& objectPath,
    const interface_list_t& interfaces) -> void
{
    if (std::find(interfaces.begin(), interfaces.end(),
                  CableInventoryIntf::interface) == interfaces.end())
    {
        return;
    }
    ctx.spawn(processCableRemovedAsync(objectPath));
}

// NOLINTNEXTLINE(readability-static-accessed-through-instance)
auto CableMonitor::start() -> sdbusplus::async::task<>
{
    info("Start cable monitor");

    ctx.spawn(handleConfigUpdate());
    ctx.spawn(handleInventoryAdded());
    ctx.spawn(handleInventoryRemoved());

    co_return;
}

// NOLINTNEXTLINE(readability-static-accessed-through-instance)
auto CableMonitor::handleConfigUpdate() -> sdbusplus::async::task<>
{
    while (!ctx.stop_requested())
    {
        expectedCables = co_await cableConfig.getExpectedCableConfig();
        co_await handleInventoryGet();
    }

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
    co_await cableEvent.generateCableEvent(EventIntf::Event::Type::connected,
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
    co_await cableEvent.generateCableEvent(EventIntf::Event::Type::disconnected,
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
