#pragma once

#include "CableConfig.hpp"
#include "CableEvents.hpp"
#include "EntityManager.hpp"
#include "NotifyWatch.hpp"

#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Inventory/Item/Cable/client.hpp>

namespace cable
{

class Monitor;

using EMConfigIntf = entity_manager::EntityManager<Monitor>;
using NotifyWatchIntf = phosphor::notify::watch::NotifyWatch<Monitor>;
using CableInventoryIntf =
    sdbusplus::client::xyz::openbmc_project::inventory::item::Cable<>;

class Monitor : public EMConfigIntf, public NotifyWatchIntf
{
  public:
    Monitor() = delete;

    explicit Monitor(sdbusplus::async::context& ctx) :
        EMConfigIntf(ctx, {CableInventoryIntf::interface}),
        NotifyWatchIntf(ctx, Config::configFileDir), ctx(ctx), cableEvents(ctx)
    {
        ctx.spawn(start());
    }

    /** @brief  Process new interfaces added to inventory */
    void processInventoryAdded(
        const sdbusplus::message::object_path& objectPath,
        const std::string& interfaceName);

    /** @brief Process interfaces removed from inventory */
    void processInventoryRemoved(
        const sdbusplus::message::object_path& objectPath,
        const std::string& interfaceName);

    /** @brief Process async changes to cable configuration */
    sdbusplus::async::task<> processConfigUpdate(std::string configFileName);

  private:
    /** @brief Start the Cable Monitor */
    sdbusplus::async::task<> start();

    /** @brief Handle async updates for cable JSON configuration */
    sdbusplus::async::task<> handleConfigUpdate();

    /** @brief Process async cable added */
    sdbusplus::async::task<> processCableAddedAsync(
        sdbusplus::message::object_path objectPath);

    /** @brief Process async cable removed */
    sdbusplus::async::task<> processCableRemovedAsync(
        sdbusplus::message::object_path objectPath);

    /** @brief D-Bus context */
    sdbusplus::async::context& ctx;
    /** @brief Set of connected cables */
    Config::Cables connectedCables;
    /** @brief Expected cables */
    Config::Cables expectedCables;
    /** @brief Cable Config object  */
    Config cableConfig;
    /** @brief Cable Event object */
    Events cableEvents;
};

} // namespace cable
