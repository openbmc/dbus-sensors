#pragma once

#include "../Utils.hpp"
#include "LeakUtils.hpp"

#include <gpiod.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/async/server.hpp>
#include <xyz/openbmc_project/Association/Definitions/aserver.hpp>
#include <xyz/openbmc_project/Configuration/Common/client.hpp>
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
    sdbusplus::client::xyz::openbmc_project::configuration::GPIOLeakDetector<>;
using CommonConfigIntf =
    sdbusplus::client::xyz::openbmc_project::configuration::Common<>;

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
        DetectorIntf(ctx, getObjectPath(config.name).str.c_str()),
        AssociationIntf(ctx, getObjectPath(config.name).str.c_str()), ctx(ctx),
        config(config)
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
