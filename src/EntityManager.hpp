#pragma once

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

namespace RulesIntf = sdbusplus::bus::match::rules;

class EntityManager
{
  public:
    using ProcessInventoryCallback_t = std::function<void(
        const sdbusplus::message::object_path&, const std::string&)>;
    static constexpr auto serviceName = "xyz.openbmc_project.EntityManager";

    EntityManager() = delete;

    EntityManager(sdbusplus::async::context& ctx,
                  const std::vector<std::string>& interfaceNames,
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
    std::vector<std::string> interfaceNames;
    ProcessInventoryCallback_t processInventoryAddedCallback;
    ProcessInventoryCallback_t processInventoryRemovedCallback;
    sdbusplus::async::match inventoryAddedMatch;
    sdbusplus::async::match inventoryRemovedMatch;
};

} // namespace entity_manager
