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

namespace phosphor::entity::manager
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
        try
        {
            using ObjectMapper =
                sdbusplus::client::xyz::openbmc_project::ObjectMapper<>;

            auto mapper = ObjectMapper(ctx)
                              .service(ObjectMapper::default_service)
                              .path(ObjectMapper::instance_path);

            std::vector<std::string> interfaces = {interfaceName};
            auto objectPaths = co_await mapper.get_sub_tree_paths(
                "/xyz/openbmc_project/inventory", 0, interfaces);
            for (const auto& objectPath : objectPaths)
            {
                static_cast<Instance*>(this)->processInventoryAdded(objectPath);
            }
        }
        catch (std::exception& e)
        {
            error(
                "Exception occurred for GetSubTreePaths for {INTERFACE}: {ERROR}",
                "INTERFACE", interfaceName, "ERROR", e);
            throw;
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

} // namespace phosphor::entity::manager
