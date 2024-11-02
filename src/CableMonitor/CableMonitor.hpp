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
namespace RulesIntf = sdbusplus::bus::match::rules;

constexpr auto EntityManagerServiceName = "xyz.openbmc_project.EntityManager";

class CableMonitor : public EMConfigIntf
{
  public:
    CableMonitor() = delete;

    explicit CableMonitor(sdbusplus::async::context& ctx) :
        EMConfigIntf(ctx), ctx(ctx)
    {
        cableConfig = std::make_unique<ConfigIntf::Config>(ctx);
        cableEvent = std::make_unique<EventIntf::Event>(ctx);
        cableAddedMatch = std::make_unique<sdbusplus::async::match>(
            ctx, RulesIntf::interfacesAdded() +
                     RulesIntf::sender(EntityManagerServiceName));
        cableRemovedMatch = std::make_unique<sdbusplus::async::match>(
            ctx, RulesIntf::interfacesRemoved() +
                     RulesIntf::sender(EntityManagerServiceName));
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
    // using interface_list_t = std::set<std::string>;

    /** @brief Start the Cable Monitor */
    auto start() -> sdbusplus::async::task<>;

    /** @brief Handle async updates for cable configuration */
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
    std::unique_ptr<ConfigIntf::Config> cableConfig;
    /** @brief Cable Event object */
    std::unique_ptr<EventIntf::Event> cableEvent;
    /** @brief D-Bus matcher for cable added */
    std::unique_ptr<sdbusplus::async::match> cableAddedMatch;
    /** @brief D-Bus matcher for cable removed */
    std::unique_ptr<sdbusplus::async::match> cableRemovedMatch;
};

} // namespace phosphor::cable::monitor
