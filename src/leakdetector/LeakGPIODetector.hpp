#pragma once

#include "GPIOInterface.hpp"
#include "LeakEvents.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/server.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Association/Definitions/aserver.hpp>
#include <xyz/openbmc_project/Configuration/GPIOLeakDetector/client.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/aserver.hpp>

#include <array>
#include <string>
#include <string_view>
#include <unordered_map>

PHOSPHOR_LOG2_USING;

namespace leak
{

class GPIODetector;

using DetectorConfigIntf =
    sdbusplus::client::xyz::openbmc_project::configuration::GPIOLeakDetector<>;

using DetectorIntf = sdbusplus::async::server_t<
    GPIODetector,
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions,
    sdbusplus::aserver::xyz::openbmc_project::state::leak::Detector>;

namespace config
{

static constexpr std::array<std::string_view, 2> detectorTypes = {
    "LeakSensingCable", "Unknown"};

/** @brief Detector type to enum map */
static const auto validDetectorTypes =
    std::unordered_map<std::string_view, DetectorIntf::DetectorType>{
        {detectorTypes[0], DetectorIntf::DetectorType::LeakSensingCable},
        {detectorTypes[1], DetectorIntf::DetectorType::Unknown}};

/** @brief GPIO polarity */
enum class PinPolarity
{
    activeLow,
    activeHigh
};

static constexpr std::array<std::string_view, 2> pinPolarity = {
    "ActiveLow", "ActiveHigh"};

/** @brief Polarity name to enum map */
static const auto validPinPolarity =
    std::unordered_map<std::string_view, PinPolarity>{
        {pinPolarity[0], PinPolarity::activeLow},
        {pinPolarity[1], PinPolarity::activeHigh}};

/** @brief Detector level */
enum class DetectorLevel
{
    critical,
    warning
};

static constexpr std::array<std::string_view, 2> detectorLevel = {
    "Warning", "Critical"};

/** @brief Leak detector level name to enum map */
static const auto validDetectorLevel =
    std::unordered_map<std::string_view, DetectorLevel>{
        {detectorLevel[0], DetectorLevel::warning},
        {detectorLevel[1], DetectorLevel::critical}};

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

using GPIOIntf = gpio::GPIOInterface<GPIODetector>;

class GPIODetector : public DetectorIntf, public GPIOIntf
{
  public:
    explicit GPIODetector(sdbusplus::async::context& ctx, Events& leakEvents,
                          const config::DetectorConfig& config);

    void updateGPIOState(bool gpioState);

    sdbusplus::async::task<> updateGPIOStateAsync(bool gpioState);

  private:
    static sdbusplus::message::object_path getObjectPath(
        const std::string& detectorName);

    sdbusplus::async::context& ctx;
    Events& leakEvents;
    config::DetectorConfig config;
    bool emitSignal = false;
};

} // namespace leak
