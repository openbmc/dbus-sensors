#pragma once

#include "GPIInterface.hpp"
#include "GPOInterface.hpp"
#include "ValveEvents.hpp"

#include <sdbusplus/async.hpp>
#include <sdbusplus/async/server.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Association/Definitions/aserver.hpp>
#include <xyz/openbmc_project/Configuration/GPIOValve/client.hpp>
#include <xyz/openbmc_project/Control/Valve/aserver.hpp>
#include <xyz/openbmc_project/Sensor/Value/aserver.hpp>
#include <xyz/openbmc_project/State/Decorator/Availability/aserver.hpp>
#include <xyz/openbmc_project/State/Decorator/OperationalStatus/aserver.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace valve
{

using ValveConfigIntf =
    sdbusplus::client::xyz::openbmc_project::configuration::GPIOValve<>;

class GPIOValve;

using ValveIntf = sdbusplus::async::server_t<
    GPIOValve, sdbusplus::aserver::xyz::openbmc_project::sensor::Value,
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions,
    sdbusplus::aserver::xyz::openbmc_project::state::decorator::Availability,
    sdbusplus::aserver::xyz::openbmc_project::state::decorator::
        OperationalStatus>;

using ValveControlIntf = sdbusplus::async::server_t<
    GPIOValve, sdbusplus::aserver::xyz::openbmc_project::control::Valve,
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions>;

namespace config
{

enum class PinPolarity
{
    activeLow,
    activeHigh,
    unknown
};

struct ValveConfig
{
    std::string name = Defaults::name;
    std::string openPinName = Defaults::pinName;
    PinPolarity openPolarity = PinPolarity::unknown;
    std::string openControlPinName = Defaults::pinName;
    bool openControlValue = false;
    uint8_t targetOpen = 100; // For a GPIO valve, 100% is the target open
                              // value as valve can only be open or closed.

    struct Defaults
    {
        static constexpr auto name = "unknown";
        static constexpr auto pinName = "unknown";
    };
};

/** @brief Get the valve configuration from the Entity Manager */
auto getConfig(sdbusplus::async::context& ctx,
               sdbusplus::message::object_path objectPath)
    -> sdbusplus::async::task<std::optional<ValveConfig>>;

} // namespace config

class GPIOValve : public ValveIntf, public ValveControlIntf
{
  public:
    explicit GPIOValve(sdbusplus::async::context& ctx,
                       sdbusplus::message::object_path& objectPath,
                       Events& events, const config::ValveConfig& config);

    ~GPIOValve();

    auto createAssociations() -> sdbusplus::async::task<>;

    // NOLINTNEXTLINE(readability-identifier-naming)
    auto get_property(state_t /*unused*/) const -> State;

    // NOLINTNEXTLINE(readability-identifier-naming)
    auto set_property(state_t /*unused*/, auto state) -> bool;

  private:
    auto updateGPIOStateAsync(bool gpioState) -> sdbusplus::async::task<>;

    auto createSensorAssociations() -> sdbusplus::async::task<>;

    auto createControlAssociations() -> void;

    sdbusplus::async::context& ctx;
    sdbusplus::message::object_path inventoryPath;
    Events& events;
    config::ValveConfig config;
    gpio::GPIInterface inputInterface;
};

} // namespace valve
