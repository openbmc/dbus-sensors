#pragma once

#include "LeakEvents.hpp"
#include "Systemd.hpp"
#include "Utils.hpp"

#include <gpiod.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/async/server.hpp>
#include <xyz/openbmc_project/Association/Definitions/aserver.hpp>
#include <xyz/openbmc_project/Configuration/Common/client.hpp>
#include <xyz/openbmc_project/Configuration/GPIOLeakDetector/client.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/aserver.hpp>

PHOSPHOR_LOG2_USING;

constexpr auto serviceName = "xyz.openbmc_project.LeakDetector";

using LeakEventsIntf = phosphor::leak::events::LeakEvents;
using SystemdIntf = phosphor::systemd::Systemd;

namespace phosphor::leak::detector
{

class LeakDetector;

using AssociationIntf =
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions<
        LeakDetector>;
using DetectorIntf =
    sdbusplus::aserver::xyz::openbmc_project::state::leak::Detector<
        LeakDetector>;
using DetectorConfigIntf =
    sdbusplus::client::xyz::openbmc_project::configuration::GPIOLeakDetector<>;
using CommonConfigIntf =
    sdbusplus::client::xyz::openbmc_project::configuration::Common<>;

namespace config
{

/** @brief Leak detector configuration */
struct DetectorConfig
{
    /** @brief Detector name */
    std::string name = defaults::name;
    /** @brief Type */
    DetectorIntf::DetectorType type = DetectorIntf::DetectorType::Unknown;
    /** @brief Pin name */
    std::string pinName = defaults::pinName;
    /** @brief Polarity */
    CommonConfigIntf::PinPolarity polarity =
        CommonConfigIntf::PinPolarity::ActiveLow;
    /** @brief Level */
    DetectorConfigIntf::DetectorLevel level =
        DetectorConfigIntf::DetectorLevel::Critical;
    /** @brief Default Action */
    DetectorConfigIntf::DetectorDefaultAction action =
        DetectorConfigIntf::DetectorDefaultAction::Unknown;

    /** @brief Default values */
    struct defaults
    {
        static constexpr auto name = "unknown";
        static constexpr auto pinName = "unknown";
    };
};

/** @brief Get the detector configuration from the Entity Manager
 */
auto processConfig(sdbusplus::async::context& ctx,
                   const sdbusplus::message::object_path& objectPath)
    -> sdbusplus::async::task<DetectorConfig>;

}; // namespace config

class LeakDetector : public DetectorIntf, public AssociationIntf
{
  public:
    explicit LeakDetector(sdbusplus::async::context& ctx,
                          LeakEventsIntf& leakEvents, SystemdIntf& systemd,
                          const config::DetectorConfig& config) :
        DetectorIntf(ctx, getObjectPath(config.name).str.c_str()),
        AssociationIntf(ctx, getObjectPath(config.name).str.c_str()), ctx(ctx),
        leakEvents(leakEvents), systemd(systemd), config(config)
    {
        try
        {
            createLeakDetector();
        }
        catch (const std::exception& e)
        {
            throw;
        }
    }

  private:
    /** @brief Create the leak detector */
    auto createLeakDetector() -> void;
    /**  @brief Get the detector object path for the given detector name */
    auto getObjectPath(std::string detectorName)
        -> sdbusplus::message::object_path;
    /** @brief Read the leak detector state. Returns true if read succeded
     * otherwise false */
    auto readState() -> bool;
    /** @brief Setup Async handler for the leak detector state change. Returns
     * true if setup succeded otherwise false */
    auto setupAsyncEvent() -> bool;
    /** @brief Read the  leak detector state asynchronously */
    auto readStateAsync() -> sdbusplus::async::task<>;
    /** @brief Update the leak detector state given the GPIO state */
    auto updateState(bool gpioState) -> sdbusplus::async::task<>;
    /** @brief D-Bus context */
    sdbusplus::async::context& ctx;
    /** @brief Leak events instance */
    LeakEventsIntf& leakEvents;
    /** @brief Systemd instance */
    SystemdIntf& systemd;
    /** @brief Detector configuration  */
    config::DetectorConfig config;
    /** @brief gpio line */
    gpiod::line line;
    /** @brief gpio line file descriptor */
    int lineFd = -1;
    /** File descriptor based async event handler */
    std::unique_ptr<sdbusplus::async::fdio> fdioInstance;
};

} // namespace phosphor::leak::detector
