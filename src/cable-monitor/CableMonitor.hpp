#pragma once

#include "CableConfig.hpp"
#include "CableEvent.hpp"
#include "EntityManager.hpp"
#include "NotifyWatch.hpp"

#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Inventory/Item/Cable/client.hpp>

namespace phosphor::cable::monitor
{

class CableMonitor;

using EMConfigIntf = phosphor::entity::manager::EntityManager<CableMonitor>;
using NotifyWatchIntf = phosphor::notify::watch::NotifyWatch<CableMonitor>;
using CableInventoryIntf =
    sdbusplus::client::xyz::openbmc_project::inventory::item::Cable<>;

class CableMonitor : public EMConfigIntf, public NotifyWatchIntf
{
  public:
    CableMonitor() = delete;

    explicit CableMonitor(sdbusplus::async::context& ctx) :
        EMConfigIntf(ctx, CableInventoryIntf::interface),
        NotifyWatchIntf(ctx, config::configFileDir), ctx(ctx), cableEvent(ctx)
    {
        ctx.spawn(start());
    }

    /** @brief  Process new interfaces added to inventory */
    auto
        processInventoryAdded(const sdbusplus::message::object_path& objectPath)
            -> void;

    /** @brief Process interfaces removed from inventory */
    auto processInventoryRemoved(
        const sdbusplus::message::object_path& objectPath) -> void;

    /** @brief Process async changes to cable configuration */
    auto processConfigUpdate(std::string configFileName)
        -> sdbusplus::async::task<>;

  private:
    /** @brief Start the Cable Monitor */
    auto start() -> sdbusplus::async::task<>;

    /** @brief Handle async updates for cable JSON configuration */
    auto handleConfigUpdate() -> sdbusplus::async::task<>;

    /** @brief Process async cable added */
    auto processCableAddedAsync(sdbusplus::message::object_path objectPath)
        -> sdbusplus::async::task<>;

    /** @brief Process async cable removed */
    auto processCableRemovedAsync(sdbusplus::message::object_path objectPath)
        -> sdbusplus::async::task<>;

    /** @brief D-Bus context */
    sdbusplus::async::context& ctx;
    /** @brief Set of connected cables */
    config::Config::Cables connectedCables;
    /** @brief Expected cables */
    config::Config::Cables expectedCables;
    /** @brief Cable Config object  */
    config::Config cableConfig;
    /** @brief Cable Event object */
    event::Event cableEvent;
};

} // namespace phosphor::cable::monitor
