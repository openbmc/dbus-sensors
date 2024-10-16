#pragma once

#include "GPIOInterface.hpp"
#include "LeakEvents.hpp"
#include "Systemd.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/server.hpp>
#include <sdbusplus/async/task.hpp>
#include <xyz/openbmc_project/Association/Definitions/aserver.hpp>
#include <xyz/openbmc_project/Configuration/GPIOLeakDetector/client.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/aserver.hpp>

#include <exception>
#include <string>
#include <unordered_map>

PHOSPHOR_LOG2_USING;

using SystemdIntf = systemd::Systemd;

namespace leak::detector
{

class LeakDetector;

using DetectorConfigIntf =
    sdbusplus::client::xyz::openbmc_project::configuration::GPIOLeakDetector<>;

using DetectorIntf = sdbusplus::async::server_t<
    LeakDetector,
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions,
    sdbusplus::aserver::xyz::openbmc_project::state::leak::Detector>;

namespace config
{

/** @brief Detector type to enum map */
static const auto validDetectorTypes =
    std::unordered_map<std::string, DetectorIntf::DetectorType>{
        {"LeakSensingCable", DetectorIntf::DetectorType::LeakSensingCable},
        {"Unknown", DetectorIntf::DetectorType::Unknown}};

/** @brief GPIO polarity */
enum class PinPolarity
{
    activeLow,
    activeHigh
};

/** @brief Polarity name to enum map */
static const auto validPinPolarity =
    std::unordered_map<std::string, PinPolarity>{
        {"ActiveLow", PinPolarity::activeLow},
        {"ActiveHigh", PinPolarity::activeHigh}};

/** @brief Detector level */
enum class DetectorLevel
{
    critical,
    warning
};

/** @brief Leak detector level name to enum map */
static const auto validDetectorLevel =
    std::unordered_map<std::string, DetectorLevel>{
        {"Warning", DetectorLevel::warning},
        {"Critical", DetectorLevel::critical}};

/** @brief Leak detector configuration */
struct DetectorConfig
{
    std::string name = Defaults::name;
    DetectorIntf::DetectorType type = DetectorIntf::DetectorType::Unknown;
    std::string pinName = Defaults::pinName;
    PinPolarity polarity = PinPolarity::activeLow;
    DetectorLevel level = DetectorLevel::critical;

    struct Defaults
    {
        static constexpr auto name = "unknown";
        static constexpr auto pinName = "unknown";
    };
};

}; // namespace config

using GPIOIntf = gpio::GPIOInterface<LeakDetector>;

class LeakDetector : public DetectorIntf, public GPIOIntf
{
  public:
    explicit LeakDetector(sdbusplus::async::context& ctx,
                          events::LeakEvents& leakEvents, SystemdIntf& systemd,
                          const config::DetectorConfig& config) :
        DetectorIntf(ctx, getObjectPath(config.name).str.c_str()),
        GPIOIntf(ctx, config.name, config.pinName,
                 (config.polarity == config::PinPolarity::activeLow)),
        ctx(ctx), leakEvents(leakEvents), systemd(systemd), config(config)
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

    void updateGPIOState(bool gpioState);

    sdbusplus::async::task<> updateGPIOStateAsync(bool gpioState);

  private:
    void createLeakDetector();

    static sdbusplus::message::object_path getObjectPath(
        const std::string& detectorName);

    sdbusplus::async::context& ctx;
    events::LeakEvents& leakEvents;
    SystemdIntf& systemd;
    config::DetectorConfig config;
    bool emitSignal = false;
};

} // namespace leak::detector
