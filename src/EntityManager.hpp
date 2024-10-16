#pragma once

#include "Utils.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/match.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/ObjectMapper/client.hpp>

#include <algorithm>
#include <exception>
#include <string>
#include <vector>

namespace entity::manager
{
PHOSPHOR_LOG2_USING;
namespace RulesIntf = sdbusplus::bus::match::rules;

template <typename Instance>
class EntityManager
{
  public:
    using interface_list_t = std::vector<std::string>;
    static constexpr auto serviceName = "xyz.openbmc_project.EntityManager";

    EntityManager() = delete;

    explicit EntityManager(sdbusplus::async::context& ctx,
                           const std::string& interfaceName) :
        ctx(ctx), interfaceName(interfaceName),
        inventoryAddedMatch(ctx, RulesIntf::interfacesAdded() +
                                     RulesIntf::sender(serviceName)),
        inventoryRemovedMatch(ctx, RulesIntf::interfacesRemoved() +
                                       RulesIntf::sender(serviceName))
    {}

    /** Get the inventory info from Entity Manager */
    sdbusplus::async::task<> handleInventoryGet()
    {
        constexpr auto entityManager =
            sdbusplus::async::proxy()
                .service(serviceName)
                .path("/xyz/openbmc_project/inventory")
                .interface("org.freedesktop.DBus.ObjectManager");

        for (const auto& [objectPath, detectorConfig] :
             co_await entityManager.call<ManagedObjectType>(
                 ctx, "GetManagedObjects"))
        {
            static_cast<Instance*>(this)->processInventoryAdded(objectPath);
        }

        co_return;
    }

    /** @brief Handle async inventory add from Entity Manager */
    sdbusplus::async::task<> handleInventoryAdded()
    {
        while (!ctx.stop_requested())
        {
            auto [objectPath, inventoryData] =
                co_await inventoryAddedMatch
                    .next<sdbusplus::message::object_path, SensorData>();

            if (inventoryData.find(interfaceName) != inventoryData.end())
            {
                static_cast<Instance*>(this)->processInventoryAdded(objectPath);
            }
        }

        co_return;
    }

    /** @brief Handle async inventory remove from Entity Manager */
    sdbusplus::async::task<> handleInventoryRemoved()
    {
        while (!ctx.stop_requested())
        {
            auto [objectPath, interfaces] =
                co_await inventoryRemovedMatch
                    .next<sdbusplus::message::object_path, interface_list_t>();

            if (std::find(interfaces.begin(), interfaces.end(),
                          interfaceName) != interfaces.end())
            {
                static_cast<Instance*>(this)->processInventoryRemoved(
                    objectPath);
            }
        }

        co_return;
    }

  private:
    sdbusplus::async::context& ctx;
    std::string interfaceName;
    sdbusplus::async::match inventoryAddedMatch;
    sdbusplus::async::match inventoryRemovedMatch;
};

} // namespace entity::manager
