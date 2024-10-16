#include "EntityManager.hpp"

#include "Utils.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/match.hpp>
#include <sdbusplus/async/proxy.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace entity_manager
{

PHOSPHOR_LOG2_USING;

using interface_list_t = std::vector<std::string>;

EntityManager::EntityManager(
    sdbusplus::async::context& ctx,
    const std::vector<std::string>& interfaceNames,
    ProcessInventoryCallback_t processInventoryAddedCallback,
    ProcessInventoryCallback_t processInventoryRemovedCallback) :
    ctx(ctx), interfaceNames(interfaceNames),
    processInventoryAddedCallback(processInventoryAddedCallback),
    processInventoryRemovedCallback(processInventoryRemovedCallback),
    inventoryAddedMatch(ctx, RulesIntf::interfacesAdded() +
                                 RulesIntf::sender(serviceName)),
    inventoryRemovedMatch(ctx, RulesIntf::interfacesRemoved() +
                                   RulesIntf::sender(serviceName))
{}

sdbusplus::async::task<> EntityManager::handleInventoryGet()
{
    constexpr auto entityManager =
        sdbusplus::async::proxy()
            .service(serviceName)
            .path("/xyz/openbmc_project/inventory")
            .interface("org.freedesktop.DBus.ObjectManager");

    for (const auto& [objectPath, detectorConfig] :
         co_await entityManager.call<ManagedObjectType>(ctx,
                                                        "GetManagedObjects"))
    {
        for (const auto& interfaceName : interfaceNames)
        {
            if (detectorConfig.find(interfaceName) != detectorConfig.end() &&
                processInventoryAddedCallback)
            {
                processInventoryAddedCallback(objectPath, interfaceName);
            }
        }
    }

    co_return;
}

sdbusplus::async::task<> EntityManager::handleInventoryAdded()
{
    while (!ctx.stop_requested())
    {
        auto [objectPath, inventoryData] =
            co_await inventoryAddedMatch
                .next<sdbusplus::message::object_path, SensorData>();

        for (const auto& interfaceName : interfaceNames)
        {
            if (inventoryData.find(interfaceName) != inventoryData.end() &&
                processInventoryAddedCallback)
            {
                processInventoryAddedCallback(objectPath, interfaceName);
            }
        }
    }

    co_return;
}

sdbusplus::async::task<> EntityManager::handleInventoryRemoved()
{
    while (!ctx.stop_requested())
    {
        auto [objectPath, interfaces] =
            co_await inventoryRemovedMatch
                .next<sdbusplus::message::object_path, interface_list_t>();

        for (const auto& interfaceName : interfaceNames)
        {
            if ((std::find(interfaces.begin(), interfaces.end(),
                           interfaceName) != interfaces.end()) &&
                processInventoryRemovedCallback)
            {
                processInventoryRemovedCallback(objectPath, interfaceName);
            }
        }
    }

    co_return;
}

} // namespace entity_manager
