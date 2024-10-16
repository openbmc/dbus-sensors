#pragma once

#include "../Utils.hpp"
#include "LeakUtils.hpp"

#include <gpiod.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/async/server.hpp>
#include <xyz/openbmc_project/Association/Definitions/aserver.hpp>
#include <xyz/openbmc_project/Configuration/GPIOLeakDetector/client.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/aserver.hpp>

PHOSPHOR_LOG2_USING;

namespace phosphor::leak::detector
{

class LeakDetector;

namespace UtilsIntf = phosphor::leak::utils;
using AssociationIntf =
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions<
        LeakDetector>;
using DetectorIntf =
    sdbusplus::aserver::xyz::openbmc_project::state::leak::Detector<
        LeakDetector>;
using DetectorConfigIntf =
    sdbusplus::common::xyz::openbmc_project::configuration::GPIOLeakDetector;
using CommonConfigIntf =
    sdbusplus::common::xyz::openbmc_project::configuration::Common;
using DetectorStateIntf =
    sdbusplus::common::xyz::openbmc_project::state::leak::Detector;
using PathIntf = sdbusplus::common::xyz::openbmc_project::state::leak::
    Detector::namespace_path;

namespace config
{

/** @brief Entity Manager configuration interface for GPIO leak detector */
static constexpr auto interface = DetectorConfigIntf::interface;
/** @brief Entity Manager configuration keys for GPIO leak detector */
static constexpr auto propertyName = "Name";
static constexpr auto propertySubType = "SubType";
static constexpr auto propertyPinName = "PinName";
static constexpr auto propertyPolarity = "Polarity";
static constexpr auto propertyLevel = "Level";
static constexpr auto propertyDefaultAction = "DefaultAction";

/** @brief Action name to systemd target map */
static const auto actionTargets =
    std::unordered_map<DetectorConfigIntf::DetectorDefaultAction, std::string>{
        {DetectorConfigIntf::DetectorDefaultAction::ChassisPowerOff,
         "xyz.openbmc_project.State.Chassis.PowerOff"},
        {DetectorConfigIntf::DetectorDefaultAction::RackPowerOff,
         "xyz.openbmc_project.State.Rack.PowerOff"}};

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
        CommonConfigIntf::PinPolarity::Unknown;
    /** @brief Level */
    DetectorConfigIntf::DetectorLevel level =
        DetectorConfigIntf::DetectorLevel::Critical;
    /** @brief Action */
    DetectorConfigIntf::DetectorDefaultAction action =
        DetectorConfigIntf::DetectorDefaultAction::Unknown;

    /** @brief Default values */
    struct defaults
    {
        static constexpr auto name = "unknown";
        static constexpr auto pinName = "unknown";
    };
};

/** @brief Get the detector configuration from the Entity Manager configuration
 */
auto processConfig(const SensorBaseConfigMap& properties)
    -> std::optional<DetectorConfig>;

}; // namespace config

class LeakDetector : public DetectorIntf, public AssociationIntf
{
  public:
    explicit LeakDetector(sdbusplus::async::context& ctx,
                          const config::DetectorConfig& config) :
        DetectorIntf(ctx, getPath(config.name).c_str()),
        AssociationIntf(ctx, getPath(config.name).c_str()), ctx(ctx),
        config(config)
    {
        info("Creating leak detector {NAME}", "NAME", config.name);
        DetectorIntf::pretty_name_ = config.name;
        DetectorIntf::type_ = config.type;
        if (!readState())
        {
            error("Failed to read leak detector state for {NAME}", "NAME",
                  config.name);
            return;
        }
        if (!setupAsyncEvent())
        {
            error("Failed to setup async event for {NAME}", "NAME",
                  config.name);
            return;
        }
        debug("Created leak detector {NAME}", "NAME", config.name);
    }

    auto get_property(pretty_name_t) const
    {
        return pretty_name_;
    }

    auto get_property(state_t) const
    {
        return state_;
    }

  private:
    /**  @brief Get the path for the given name */
    auto getPath(std::string name) -> std::string;
    /** @brief Read the leak detector state */
    auto readState() -> bool;
    /** @brief Setup Async handler for the leak detector state change */
    auto setupAsyncEvent() -> bool;
    /** @brief Read the  leak detector state asynchronously */
    auto readStateAsync() -> sdbusplus::async::task<>;
    /** @brief Update the leak detector state */
    auto updateState(bool state) -> void;
    /** @brief D-Bus context */
    sdbusplus::async::context& ctx;
    /** @brief Detector configuration  */
    const config::DetectorConfig config;
    /** @brief gpio line */
    gpiod::line line;
    /** @brief gpio line file descriptor */
    int lineFd = -1;
};

} // namespace phosphor::leak::detector
