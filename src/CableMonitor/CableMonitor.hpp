#pragma once

#include "../Utils.hpp"
#include "CableConfig.hpp"
#include "CableEvent.hpp"

#include <sdbusplus/async.hpp>
#include <sdbusplus/async/server.hpp>
#include <xyz/openbmc_project/Cable/CableMonitor/aserver.hpp>

namespace phosphor::cable::monitor
{

class CableMonitor;

namespace ConfigIntf = phosphor::cable::config;
namespace EventIntf = phosphor::cable::event;
namespace RulesIntf = sdbusplus::bus::match::rules;

using CableMonitorIntf =
    sdbusplus::aserver::xyz::openbmc_project::cable::CableMonitor<CableMonitor>;

constexpr auto EntityManagerServiceName = "xyz.openbmc_project.EntityManager";

class CableMonitor : public CableMonitorIntf
{
  public:
    CableMonitor() = delete;

    explicit CableMonitor(sdbusplus::async::context& ctx, const char* path) :
        CableMonitorIntf(ctx, path), ctx(ctx)
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

  private:
    using interface_list_t = std::set<std::string>;

    /** @brief Start the Cable Monitor */
    auto start() -> sdbusplus::async::task<>;
    /** @brief Handle async updates for cable configuration */
    auto handleConfigUpdate() -> sdbusplus::async::task<>;
    /** @brief Handle cable get */
    auto handleCableGet() -> sdbusplus::async::task<>;
    /** @brief Process cable get */
    auto processCableGet(
        const sdbusplus::message::object_path& objectPath,
        const SensorData& inventoryData) -> sdbusplus::async::task<>;
    /** @brief Handle async cable added */
    auto handleCableAdded() -> sdbusplus::async::task<>;
    /** @brief Process async cable added */
    auto processCableAdded(
        const sdbusplus::message::object_path& objectPath,
        const SensorData& inventoryData) -> sdbusplus::async::task<>;
    /** @brief Handle async cable removed */
    auto handleCableRemoved() -> sdbusplus::async::task<>;
    /** @brief Process async cable removed */
    auto processCableRemoved(
        const sdbusplus::message::object_path& objectPath,
        const interface_list_t& interfaces) -> sdbusplus::async::task<>;

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
