#include "CableMonitor.hpp"

#include <phosphor-logging/lg2.hpp>

PHOSPHOR_LOG2_USING;

namespace phosphor::cable::monitor
{

constexpr auto CableInventoryInterface =
    "xyz.openbmc_project.Inventory.Item.Cable";

auto CableMonitor::start() -> sdbusplus::async::task<>
{
    info("Start cable monitor");

    ctx.spawn(handleConfigUpdate());
    ctx.spawn(handleInventoryAdded());
    ctx.spawn(handleInventoryRemoved());

    co_return;
}

auto CableMonitor::handleConfigUpdate() -> sdbusplus::async::task<>
{
    while (!ctx.stop_requested())
    {
        expectedCables = co_await cableConfig->getExpectedCableConfig();
        co_await handleInventoryGet();
    }

    co_return;
}

/*
auto getCableInventoryName(const SensorBaseConfigMap& properties)
    -> sdbusplus::async::task<std::optional<std::string>>
{
    static constexpr auto propertyName = "Name";
    try
    {
        co_return loadVariant<std::string>(properties, propertyName);
    }
    catch (const std::exception& e)
    {
        warning("Failed to find cable name in properties: {ERROR}", "ERROR",
                e.what());
    }
    co_return std::nullopt;
}
*/

auto CableMonitor::processInventoryAdded(
    const sdbusplus::message::object_path& objectPath,
    const SensorData& inventoryData) -> void
{
    auto cableData = inventoryData.find(CableInventoryInterface);
    if (cableData == inventoryData.end())
    {
        error("Cable {PATH} does not have cable interface", "PATH", objectPath);
        return;
    }
    ctx.spawn(processCableAddedAsync(objectPath));
}

auto CableMonitor::processCableAddedAsync(
    const sdbusplus::message::object_path& objectPath)
    -> sdbusplus::async::task<>
{
    info("Cable {PATH} has cable interface", "PATH", objectPath);
    auto cableName = objectPath.filename();

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
    cableEvent->generateCableEvent(EventIntf::Event::Type::connected,
                                   cableName);
    debug("New cable {NAME} added", "NAME", cableName);

    co_return;
}

auto CableMonitor::processInventoryRemoved(
    const sdbusplus::message::object_path& objectPath,
    const interface_list_t& interfaces) -> void
{
    if (std::find(interfaces.begin(), interfaces.end(),
                  CableInventoryInterface) == interfaces.end())
    {
        return;
    }
    ctx.spawn(processCableRemovedAsync(objectPath));
}

auto CableMonitor::processCableRemovedAsync(
    const sdbusplus::message::object_path& objectPath)
    -> sdbusplus::async::task<>
{
    auto cableName = objectPath.filename();

    debug("Received cable removed signal for {NAME}", "NAME", cableName);

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
    cableEvent->generateCableEvent(EventIntf::Event::Type::disconnected,
                                   cableName);
    debug("Removed cable {NAME}", "NAME", cableName);

    co_return;
}

} // namespace phosphor::cable::monitor

using namespace phosphor::cable::monitor;

int main()
{
    constexpr auto path = "/xyz/openbmc_project/cable_monitor";
    constexpr auto serviceName = "xyz.openbmc_project.CableMonitor";
    sdbusplus::async::context ctx;
    sdbusplus::server::manager_t manager{ctx, path};

    info("Creating cable monitor");
    CableMonitor cableMonitor{ctx};
    ctx.request_name(serviceName);

    ctx.run();
    return 0;
}
