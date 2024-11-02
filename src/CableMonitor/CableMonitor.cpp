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
    ctx.spawn(handleCableAdded());
    ctx.spawn(handleCableRemoved());

    co_return;
}

auto CableMonitor::processCableGet(
    const sdbusplus::message::object_path& objectPath,
    const SensorData& inventoryData) -> sdbusplus::async::task<>
{
    auto cableData = inventoryData.find(CableInventoryInterface);
    if (cableData == inventoryData.end())
    {
        co_return;
    }

    debug("Cable {PATH} has cable interface", "PATH", objectPath);
    auto cableName = objectPath.filename();

    if (expectedCables.find(cableName) == expectedCables.end())
    {
        debug(
            "Cable {NAME} is not in expected cables, so skip connected event generation",
            "NAME", cableName);
        co_return;
    }
    else if (connectedCables.find(cableName) != connectedCables.end())
    {
        debug("Cable {NAME} is already connected, so skip it", "NAME",
              cableName);
        co_return;
    }

    connectedCables.insert(cableName);
    cableEvent->generateCableEvent(EventIntf::Event::Type::connected,
                                   cableName);
    debug("New cable {NAME} added", "NAME", cableName);

    co_return;
}

auto CableMonitor::handleCableGet() -> sdbusplus::async::task<>
{
    constexpr auto entityManager =
        sdbusplus::async::proxy()
            .service(EntityManagerServiceName)
            .path("/xyz/openbmc_project/inventory")
            .interface("org.freedesktop.DBus.ObjectManager");

    for (const auto& [objectPath, detectorConfig] :
         co_await entityManager.call<ManagedObjectType>(ctx,
                                                        "GetManagedObjects"))
    {
        co_await processCableGet(objectPath, detectorConfig);
    }

    co_return;
}

auto CableMonitor::handleConfigUpdate() -> sdbusplus::async::task<>
{
    while (!ctx.stop_requested())
    {
        expectedCables = co_await cableConfig->getExpectedCableConfig();
        co_await handleCableGet();
    }

    co_return;
}

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

auto CableMonitor::processCableAdded(
    const sdbusplus::message::object_path& objectPath,
    const SensorData& inventoryData) -> sdbusplus::async::task<>
{
    auto cableData = inventoryData.find(CableInventoryInterface);
    if (cableData == inventoryData.end())
    {
        error("Cable {PATH} does not have cable interface", "PATH", objectPath);
        co_return;
    }

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

auto CableMonitor::handleCableAdded() -> sdbusplus::async::task<>
{
    while (!ctx.stop_requested())
    {
        auto [objectPath, inventoryData] =
            co_await cableAddedMatch
                ->next<sdbusplus::message::object_path, SensorData>();
        co_await processCableAdded(objectPath, inventoryData);
    }

    co_return;
}

auto CableMonitor::processCableRemoved(
    const sdbusplus::message::object_path& objectPath,
    const interface_list_t& interfaces) -> sdbusplus::async::task<>
{
    if (interfaces.find(CableInventoryInterface) == interfaces.end())
    {
        co_return;
    }
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

auto CableMonitor::handleCableRemoved() -> sdbusplus::async::task<>
{
    while (!ctx.stop_requested())
    {
        auto [objectPath, interfaces] =
            co_await cableRemovedMatch
                ->next<sdbusplus::message::object_path, interface_list_t>();
        co_await processCableRemoved(objectPath, interfaces);
    }

    co_return;
}

} // namespace phosphor::cable::monitor

using namespace phosphor::cable::monitor;

int main()
{
    constexpr auto path = CableMonitorIntf::CableMonitor::instance_path;
    constexpr auto serviceName =
        CableMonitorIntf::CableMonitor::default_service;
    sdbusplus::async::context ctx;
    sdbusplus::server::manager_t manager{ctx, path};

    info("Creating cable monitor at {PATH}", "PATH", path);
    CableMonitor cableMonitor{ctx, path};
    ctx.request_name(serviceName);

    ctx.run();
    return 0;
}
