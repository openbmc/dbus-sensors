#pragma once

#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/match.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <functional>
#include <string>
#include <vector>

namespace entity_manager
{

class EntityManager
{
  public:
    using ProcessInventoryCallback_t = std::function<void(
        const sdbusplus::message::object_path&, const std::string&)>;
    using interface_list_t = std::vector<std::string>;
    static constexpr auto serviceName = "xyz.openbmc_project.EntityManager";

    EntityManager() = delete;

    EntityManager(sdbusplus::async::context& ctx,
                  const interface_list_t& interfaceNames,
                  ProcessInventoryCallback_t processInventoryAddedCallback,
                  ProcessInventoryCallback_t processInventoryRemovedCallback);

    /** Get the inventory info from Entity Manager */
    sdbusplus::async::task<> handleInventoryGet();

    /** @brief Handle async inventory add from Entity Manager */
    sdbusplus::async::task<> handleInventoryAdded();

    /** @brief Handle async inventory remove from Entity Manager */
    sdbusplus::async::task<> handleInventoryRemoved();

  private:
    sdbusplus::async::context& ctx;
    interface_list_t interfaceNames;
    ProcessInventoryCallback_t processInventoryAddedCallback;
    ProcessInventoryCallback_t processInventoryRemovedCallback;
    sdbusplus::async::match inventoryAddedMatch;
    sdbusplus::async::match inventoryRemovedMatch;
};

} // namespace entity_manager
