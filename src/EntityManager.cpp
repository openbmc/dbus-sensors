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
#include <utility>

namespace entity_manager
{

PHOSPHOR_LOG2_USING;

namespace rules_intf = sdbusplus::bus::match::rules;

EntityManager::EntityManager(
    sdbusplus::async::context& ctx, const interface_list_t& interfaceNames,
    ProcessInventoryCallback_t processInventoryAddedCallback,
    ProcessInventoryCallback_t processInventoryRemovedCallback) :
    ctx(ctx), interfaceNames(interfaceNames),
    processInventoryAddedCallback(std::move(processInventoryAddedCallback)),
    processInventoryRemovedCallback(std::move(processInventoryRemovedCallback)),
    inventoryAddedMatch(ctx, rules_intf::interfacesAdded() +
                                 rules_intf::sender(serviceName)),
    inventoryRemovedMatch(ctx, rules_intf::interfacesRemoved() +
                                   rules_intf::sender(serviceName))
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
            if (detectorConfig.contains(interfaceName) &&
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
            if (inventoryData.contains(interfaceName) &&
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
