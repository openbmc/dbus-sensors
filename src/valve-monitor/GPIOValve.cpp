#include "GPIOValve.hpp"

#include "EntityManagerInterface.hpp"
#include "GPOInterface.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <array>
#include <exception>
#include <functional>
#include <optional>
#include <string_view>
#include <utility>

namespace valve
{

PHOSPHOR_LOG2_USING;

namespace config
{

/** @brief Polarity name to enum map */
static constexpr std::array<std::pair<std::string_view, PinPolarity>, 2>
    validPinPolarity = {
        {{"Low", PinPolarity::activeLow}, {"High", PinPolarity::activeHigh}}};

auto getConfig(sdbusplus::async::context& ctx,
               sdbusplus::message::object_path objectPath)
    -> sdbusplus::async::task<std::optional<GPIOConfig>>
{
    GPIOConfig config = {};
    GPIOConfigIntf::properties_t properties = {};

    try
    {
        properties =
            co_await GPIOConfigIntf(ctx)
                .service(entity_manager::EntityManagerInterface::serviceName)
                .path(objectPath.str)
                .properties();
    }
    catch (const std::exception& e)
    {
        error("Failed to get properties for {PATH}: {ERR}", "PATH",
              objectPath.str, "ERR", e);
        co_return std::nullopt;
    }

    config.name = properties.name;
    config.openPinName = properties.open_pin_name;

    for (const auto& [key, value] : config::validPinPolarity)
    {
        if (properties.open_polarity == key)
        {
            config.openPolarity = value;
            break;
        }
    }
    if (config.openPolarity == config::PinPolarity::unknown)
    {
        error("Invalid polarity {POLARITY} for {NAME}", "POLARITY",
              properties.open_polarity, "NAME", config.name);
        co_return std::nullopt;
    }

    config.openControlPinName = properties.open_control_pin_name;
    config.openControlValue = properties.open_control_value;

    debug(
        "Valve config: {NAME} {OPEN_PIN} {OPEN_POLARITY} {OPEN_CONTROL_PIN} {OPEN_CONTROL_VALUE} ",
        "NAME", config.name, "OPEN_PIN", config.openPinName, "OPEN_POLARITY",
        config.openPolarity, "OPEN_CONTROL_PIN", config.openControlPinName,
        "OPEN_CONTROL_VALUE", config.openControlValue);

    co_return config;
}

} // namespace config

GPIOValve::GPIOValve(sdbusplus::async::context& ctx,
                     sdbusplus::message::object_path& objectPath,
                     Events& events, const LocalConfig& localConfig,
                     const config::GPIOConfig& config) :
    BaseValve(ctx, objectPath, events, localConfig, config), gpioConfig(config),
    inputInterface(ctx, config.name, config.openPinName,
                   (config.openPolarity == config::PinPolarity::activeLow),
                   std::bind_front(&GPIOValve::updateGPIOStateAsync, this))
{
    ctx.spawn(inputInterface.start());
}

auto GPIOValve::getState() const -> State
{
    debug("Getting {VALVE} state", "VALVE", baseConfig.name);
    return ((value() != 0) ? State::Open : State::Close);
}

auto GPIOValve::setState(State state) -> bool
{
    debug("Setting {VALVE} to {STATE}", "VALVE", baseConfig.name, "STATE",
          convertStateToString(state));

    if ((value() == 0 && state == State::Close) ||
        (value() != 0 && state == State::Open))
    {
        info("Ignoring, as new state {STATE} matches the current state",
             "STATE", convertStateToString(state));
        return true;
    }

    try
    {
        gpio::GPOInterface controlGPO{baseConfig.name,
                                      gpioConfig.openControlPinName};
        bool controlValue =
            ((state == State::Open) ? gpioConfig.openControlValue
                                    : !gpioConfig.openControlValue);
        auto res = controlGPO.setValue(controlValue);
        if (res)
        {
            info("Successfully set {VALVE} to {STATE}", "VALVE",
                 baseConfig.name, "STATE", convertStateToString(state));
            return res;
        }
    }
    catch (const std::exception& e)
    {
        error("Failed to set control pin {PIN} to {VALUE}: {ERR}", "PIN",
              gpioConfig.openControlPinName, "VALUE",
              gpioConfig.openControlValue, "ERR", e);
    }

    error("Failed to set {VALVE} to {STATE}", "VALVE", baseConfig.name, "STATE",
          convertStateToString(state));

    return false;
}

auto GPIOValve::updateGPIOStateAsync(bool gpioState) -> sdbusplus::async::task<>
{
    auto newValue = gpioState ? 100 : 0;

    if (newValue != value())
    {
        debug("Updating valve {VALVE} to {VALUE}", "VALVE", baseConfig.name,
              "VALUE", newValue);
        value(newValue);

        co_await events.generateValveEvent(inventoryPath, gpioState);
    }

    co_return;
}

} // namespace valve
