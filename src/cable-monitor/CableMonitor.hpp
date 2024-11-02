#pragma once

#include "CableConfig.hpp"
#include "CableEvents.hpp"
#include "EntityManagerInterface.hpp"
#include "NotifyWatch.hpp"

#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Inventory/Item/Cable/client.hpp>

#include <string>

namespace cable
{

class Monitor;

using CableInventoryIntf =
    sdbusplus::client::xyz::openbmc_project::inventory::item::Cable<>;

class Monitor
{
  public:
    Monitor() = delete;

    explicit Monitor(sdbusplus::async::context& ctx);

  private:
    /** @brief  Callback handler for new interfaces added to inventory */
    void inventoryAddedHandler(
        const sdbusplus::message::object_path& objectPath,
        const std::string& interfaceName);

    /** @brief Callback handler for interfaces removed from inventory */
    void inventoryRemovedHandler(
        const sdbusplus::message::object_path& objectPath,
        const std::string& interfaceName);

    /** @brief Callback handler for async updates to cable JSON configuration */
    sdbusplus::async::task<> configUpdateHandler(std::string configFileName);

    /** @brief Start the Cable Monitor */
    sdbusplus::async::task<> start();

    /** @brief Asynchronously process cable inventory added */
    sdbusplus::async::task<> processCableAddedAsync(
        sdbusplus::message::object_path objectPath);

    /** @brief Asynchronously process cable inventory removed */
    sdbusplus::async::task<> processCableRemovedAsync(
        sdbusplus::message::object_path objectPath);

    /** @brief Reconcile connected and expected cable data */
    void reconcileCableData();

    sdbusplus::async::context& ctx;
    Config::Cables connectedCables;
    Config::Cables expectedCables;
    Config cableConfig;
    Events cableEvents;
    entity_manager::EntityManagerInterface entityManager;
    notify_watch::NotifyWatch notifyWatch;
};

} // namespace cable
