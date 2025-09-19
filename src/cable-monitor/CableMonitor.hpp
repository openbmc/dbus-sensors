#pragma once

#include "CableConfig.hpp"
#include "CableEvents.hpp"
#include "async_task/EntityManagerInterface.hpp"
#include "async_task/NotifyWatch.hpp"

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
    auto inventoryAddedHandler(
        const sdbusplus::message::object_path& objectPath,
        const std::string& interfaceName) -> void;

    /** @brief Callback handler for interfaces removed from inventory */
    auto inventoryRemovedHandler(
        const sdbusplus::message::object_path& objectPath,
        const std::string& interfaceName) -> void;

    /** @brief Callback handler for async updates to cable JSON configuration */
    auto configUpdateHandler(std::string configFileName)
        -> sdbusplus::async::task<>;

    /** @brief Start the Cable Monitor */
    auto start() -> sdbusplus::async::task<>;

    /** @brief Asynchronously process cable inventory added */
    auto processCableAddedAsync(sdbusplus::message::object_path objectPath)
        -> sdbusplus::async::task<>;

    /** @brief Asynchronously process cable inventory removed */
    auto processCableRemovedAsync(sdbusplus::message::object_path objectPath)
        -> sdbusplus::async::task<>;

    /** @brief Reconcile connected and expected cable data */
    auto reconcileCableData() -> void;

    sdbusplus::async::context& ctx;
    Config::Cables connectedCables;
    Config::Cables expectedCables;
    Events cableEvents;
    entity_manager::EntityManagerInterface entityManager;
    notify_watch::NotifyWatch notifyWatch;
};

} // namespace cable
