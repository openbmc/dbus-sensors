#pragma once

#include "BaseValve.hpp"
#include "GPIInterface.hpp"
#include "GPOInterface.hpp"
#include "ValveEvents.hpp"

#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Configuration/GPIOValve/client.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace valve
{

using GPIOConfigIntf =
    sdbusplus::client::xyz::openbmc_project::configuration::GPIOValve<>;

namespace config
{

enum class PinPolarity
{
    activeLow,
    activeHigh,
    unknown
};

struct GPIOConfig : public BaseConfig
{
    std::string openPinName = Defaults::pinName;
    PinPolarity openPolarity = PinPolarity::unknown;
    std::string openControlPinName = Defaults::pinName;
    bool openControlValue = false;
    uint8_t targetOpen = 100; // For a GPIO valve, 100% is the target open
                              // value as valve can only be open or closed.

    struct Defaults
    {
        static constexpr auto pinName = "unknown";
    };
};

/** @brief Get the valve configuration from the Entity Manager */
auto getConfig(sdbusplus::async::context& ctx,
               sdbusplus::message::object_path objectPath)
    -> sdbusplus::async::task<std::optional<GPIOConfig>>;

} // namespace config

class GPIOValve : public BaseValve
{
  public:
    explicit GPIOValve(sdbusplus::async::context& ctx,
                       sdbusplus::message::object_path& objectPath,
                       Events& events, const LocalConfig& localConfig,
                       const config::GPIOConfig& config);

  protected:
    auto getState() const -> State override;
    auto setState(State state) -> bool override;

  private:
    auto updateGPIOStateAsync(bool gpioState) -> sdbusplus::async::task<>;

    config::GPIOConfig gpioConfig;
    gpio::GPIInterface inputInterface;
};

} // namespace valve
