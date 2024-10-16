#pragma once

#include "Utils.hpp"

#include <sdbusplus/async.hpp>

namespace phosphor::entity::manager
{
namespace RulesIntf = sdbusplus::bus::match::rules;

template <typename Instance>
class EntityManager
{
  public:
    using interface_list_t = std::vector<std::string>;

    EntityManager() = delete;

    explicit EntityManager(sdbusplus::async::context& ctx) :
        ctx(ctx), inventoryAddedMatch(ctx, RulesIntf::interfacesAdded() +
                                               RulesIntf::sender(serviceName)),
        inventoryRemovedMatch(ctx, RulesIntf::interfacesRemoved() +
                                       RulesIntf::sender(serviceName))
    {}

    /** Get the inventory info from Entity Manager */
    auto handleInventoryGet() -> sdbusplus::async::task<>
    {
        constexpr auto entityManager =
            sdbusplus::async::proxy()
                .service(serviceName)
                .path("/xyz/openbmc_project/inventory")
                .interface("org.freedesktop.DBus.ObjectManager");

        for (const auto& [objectPath, inventoryData] :
             co_await entityManager.call<ManagedObjectType>(
                 ctx, "GetManagedObjects"))
        {
            static_cast<Instance*>(this)->processInventoryAdded(objectPath,
                                                                inventoryData);
        }

        co_return;
    }

    /** @brief Handle async inventory add from Entity Manager */
    auto handleInventoryAdded() -> sdbusplus::async::task<>
    {
        while (!ctx.stop_requested())
        {
            auto [objectPath, inventoryData] =
                co_await inventoryAddedMatch
                    .next<sdbusplus::message::object_path, SensorData>();

            static_cast<Instance*>(this)->processInventoryAdded(objectPath,
                                                                inventoryData);
        }

        co_return;
    }

    /** @brief Handle async inventory remove from Entity Manager */
    auto handleInventoryRemoved() -> sdbusplus::async::task<>
    {
        while (!ctx.stop_requested())
        {
            auto [objectPath, interfaces] =
                co_await inventoryRemovedMatch
                    .next<sdbusplus::message::object_path, interface_list_t>();

            static_cast<Instance*>(this)->processInventoryRemoved(
                objectPath, interfaces);
        }

        co_return;
    }

  private:
    static constexpr auto serviceName = "xyz.openbmc_project.EntityManager";

    /** @brief D-Bus context */
    sdbusplus::async::context& ctx;
    /** @brief sdbusplus::bus::match_t for inventory added */
    sdbusplus::async::match inventoryAddedMatch;
    /** @brief sdbusplus::bus::match_t for config removed */
    sdbusplus::async::match inventoryRemovedMatch;
};

} // namespace phosphor::entity::manager
