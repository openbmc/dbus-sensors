#pragma once

#include "GPIInterface.hpp"
#include "LeakEvents.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/async/server.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Association/Definitions/aserver.hpp>
#include <xyz/openbmc_project/Configuration/GPIOLeakDetector/client.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/aserver.hpp>

#include <array>
#include <string>
#include <string_view>
#include <utility>

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

/** @brief Detector type to enum map */
static constexpr std::array<
    std::pair<std::string_view, DetectorIntf::DetectorType>, 2>
    validDetectorTypes = {
        {{"LeakSensingCable", DetectorIntf::DetectorType::LeakSensingCable},
         {"Unknown", DetectorIntf::DetectorType::Unknown}}};

/** @brief GPIO polarity */
enum class PinPolarity
{
    activeLow,
    activeHigh,
    unknown
};

/** @brief Polarity name to enum map */
static constexpr std::array<std::pair<std::string_view, PinPolarity>, 2>
    validPinPolarity = {
        {{"Low", PinPolarity::activeLow}, {"High", PinPolarity::activeHigh}}};

/** @brief Detector level */
enum class DetectorLevel
{
    critical,
    warning,
    unknown
};

/** @brief Leak detector level name to enum map */
static constexpr std::array<std::pair<std::string_view, DetectorLevel>, 2>
    validDetectorLevel = {{{"Warning", DetectorLevel::warning},
                           {"Critical", DetectorLevel::critical}}};

/** @brief Leak detector configuration */
struct DetectorConfig
{
    std::string name = Defaults::name;
    DetectorIntf::DetectorType type = DetectorIntf::DetectorType::Unknown;
    std::string pinName = Defaults::pinName;
    PinPolarity polarity = PinPolarity::unknown;
    DetectorLevel level = DetectorLevel::unknown;
    sdbusplus::message::object_path parentInventoryPath;

    struct Defaults
    {
        static constexpr auto name = "unknown";
        static constexpr auto pinName = "unknown";
    };
};

}; // namespace config

class GPIODetector : public DetectorIntf
{
  public:
    explicit GPIODetector(sdbusplus::async::context& ctx, Events& leakEvents,
                          const config::DetectorConfig& config);

    auto createAssociations() -> void;

  private:
    auto updateGPIOStateAsync(bool gpioState) -> sdbusplus::async::task<>;

    sdbusplus::async::context& ctx;
    Events& leakEvents;
    config::DetectorConfig config;
    gpio::GPIInterface inputInterface;
};

} // namespace leak
