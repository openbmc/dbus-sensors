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

    EntityManager(sdbusplus::async::context& ctx) :
        ctx(ctx), configAddedMatch(ctx, RulesIntf::interfacesAdded() +
                                            RulesIntf::sender(serviceName)),
        configRemovedMatch(ctx, RulesIntf::interfacesRemoved() +
                                    RulesIntf::sender(serviceName))
    {}

    /** Get the initial config from Entity Manager */
    auto handleConfigGet() -> sdbusplus::async::task<>
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
            static_cast<Instance*>(this)->processConfigAdded(objectPath,
                                                             detectorConfig);
        }

        co_return;
    }

    /** @brief Handle async config add from Entity Manager */
    auto handleConfigAdded() -> sdbusplus::async::task<>
    {
        while (!ctx.stop_requested())
        {
            auto [objectPath, detectorConfig] =
                co_await configAddedMatch
                    .next<sdbusplus::message::object_path, SensorData>();

            static_cast<Instance*>(this)->processConfigAdded(objectPath,
                                                             detectorConfig);
        }

        co_return;
    }

    /** @brief Handle async config remove from Entity Manager */
    auto handleConfigRemoved() -> sdbusplus::async::task<>
    {
        while (!ctx.stop_requested())
        {
            auto [objectPath, interfaces] =
                co_await configRemovedMatch
                    .next<sdbusplus::message::object_path, interface_list_t>();

            static_cast<Instance*>(this)->processConfigRemoved(objectPath,
                                                               interfaces);
        }

        co_return;
    }

  private:
    static constexpr auto serviceName = "xyz.openbmc_project.EntityManager";

    /** @brief D-Bus context */
    sdbusplus::async::context& ctx;
    /** @brief sdbusplus::bus::match_t for config added */
    sdbusplus::async::match configAddedMatch;
    /** @brief sdbusplus::bus::match_t for config removed */
    sdbusplus::async::match configRemovedMatch;
};

} // namespace phosphor::entity::manager
