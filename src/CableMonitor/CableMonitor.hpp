#pragma once

#include "CableConfig.hpp"
#include "CableEvent.hpp"
#include "EntityManager.hpp"

#include <sdbusplus/async.hpp>
#include <sdbusplus/async/server.hpp>

namespace phosphor::cable::monitor
{

class CableMonitor;

using EMConfigIntf = phosphor::entity::manager::EntityManager<CableMonitor>;

namespace ConfigIntf = phosphor::cable::config;
namespace EventIntf = phosphor::cable::event;

class CableMonitor : public EMConfigIntf
{
  public:
    CableMonitor() = delete;

    explicit CableMonitor(sdbusplus::async::context& ctx) :
        EMConfigIntf(ctx), ctx(ctx), cableConfig(ctx), cableEvent(ctx)
    {
        ctx.spawn(start());
    }

    /** @brief  Process new interfaces added to inventory */
    auto
        processInventoryAdded(const sdbusplus::message::object_path& objectPath,
                              const SensorData& inventoryData) -> void;

    /** @brief Process interfaces removed from inventory */
    auto processInventoryRemoved(
        const sdbusplus::message::object_path& objectPath,
        const interface_list_t& interfaces) -> void;

  private:
    /** @brief Start the Cable Monitor */
    auto start() -> sdbusplus::async::task<>;

    /** @brief Handle async updates for cable JSON configuration */
    auto handleConfigUpdate() -> sdbusplus::async::task<>;

    /** @brief Process async cable added */
    auto processCableAddedAsync(const sdbusplus::message::object_path&
                                    objectPath) -> sdbusplus::async::task<>;

    /** @brief Process async cable removed */
    auto processCableRemovedAsync(const sdbusplus::message::object_path&
                                      objectPath) -> sdbusplus::async::task<>;

    /** @brief D-Bus context */
    sdbusplus::async::context& ctx;
    /** @brief Set of connected cables */
    ConfigIntf::Config::Cables connectedCables;
    /** @brief Expected cables */
    ConfigIntf::Config::Cables expectedCables;
    /** @brief Cable Config object  */
    ConfigIntf::Config cableConfig;
    /** @brief Cable Event object */
    EventIntf::Event cableEvent;
};

} // namespace phosphor::cable::monitor
