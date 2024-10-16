#pragma once

#include "../Utils.hpp"
#include "LeakUtils.hpp"

class LeakDetector;

// #include <xyz/openbmc_project/Association/Definitions/server.hpp>
#include <gpiod.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/async/server.hpp>
#include <xyz/openbmc_project/Association/Definitions/aserver.hpp>
#include <xyz/openbmc_project/LeakDetection/Detector/aserver.hpp>

namespace phosphor::leak::detector
{

namespace UtilsIntf = phosphor::leak::utils;
using AssociationIntf =
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions<
        LeakDetector>;
using LeakDetectorIntf =
    sdbusplus::aserver::xyz::openbmc_project::leak_detection::Detector<
        LeakDetector>;
using PathIntf = sdbusplus::common::xyz::openbmc_project::leak_detection::
    Detector::namespace_path;

namespace config
{

/** @brief Entity Manager configuration interface for GPIO leak detector */
static constexpr auto interface =
    "xyz.openbmc_project.Configuration.GpioLeakDetector";
/** @brief Entity Manager configuration keys for GPIO leak detector */
static constexpr auto propertyName = "Name";
static constexpr auto propertyPinName = "PinName";
static constexpr auto propertyPolarity = "Polarity";
static constexpr auto propertyLevel = "Level";
static constexpr auto propertyDefaultAction = "DefaultAction";

/** @brief GPIO polarity */
enum class Polarity
{
    activeLow,
    activeHigh,
    unknown
};

/** @brief Polarity name to enum map */
static const auto validPolarity = std::unordered_map<std::string, Polarity>{
    {"ActiveLow", Polarity::activeLow}, {"ActiveHigh", Polarity::activeHigh}};

/** @brief Leak detector actions */
enum class Action
{
    ChassisPowerOff,
    unknown
};

/** @brief Action name to enum map */
static const auto validActions = std::unordered_map<std::string, Action>{
    {"ChassisPowerOff", Action::ChassisPowerOff}};

/** @brief Action name to systemd target map */
static const auto actionTargets = std::unordered_map<Action, std::string>{
    {Action::ChassisPowerOff, "xyz.openbmc_project.State.Chassis.PowerOff"}};

/** @brief Leak detector level name to enum map */
static const auto validLevels =
    std::unordered_map<std::string, UtilsIntf::Level>{
        {"Warning", UtilsIntf::Level::warning},
        {"Critical", UtilsIntf::Level::critical}};

/** @brief Leak detector configuration */
struct DetectorConfig
{
    /** @brief Detector name */
    std::string name = defaults::name;
    /** @brief Pin name */
    std::string pinName = defaults::pinName;
    /** @brief Polarity */
    Polarity polarity = Polarity::unknown;
    /** @brief Level */
    UtilsIntf::Level level = UtilsIntf::Level::unknown;
    /** @brief Action */
    Action action = Action::unknown;

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

class LeakDetector : public LeakDetectorIntf, public AssociationIntf
{
  public:
    explicit LeakDetector(sdbusplus::async::context& ctx,
                          const config::DetectorConfig& config) :
        LeakDetectorIntf(ctx, getPath(config.name).c_str()),
        AssociationIntf(ctx, getPath(config.name).c_str()), ctx(ctx),
        config(config)
    {
        info("Creating leak detector {NAME}", "NAME", config.name);
        LeakDetectorIntf::pretty_name_ = config.name;
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
        info("Created leak detector {NAME}", "NAME", config.name);
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
