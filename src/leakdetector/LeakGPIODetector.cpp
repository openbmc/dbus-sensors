#include "LeakGPIODetector.hpp"

#include "LeakEvents.hpp"
#include "Systemd.hpp"

#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <array>
#include <functional>
#include <string>
#include <string_view>
#include <utility>

namespace leak
{

namespace config
{

/** @brief Leak level to systemd target service map */
static constexpr std::array<std::pair<config::DetectorLevel, std::string_view>,
                            2>
    leakActionTargets = {{{config::DetectorLevel::warning,
                           "xyz.openbmc_project.leakdetector.warning@"},
                          {config::DetectorLevel::critical,
                           "xyz.openbmc_project.leakdetector.critical@"}}};

} // namespace config

static sdbusplus::message::object_path getObjectPath(
    const std::string& detectorName)
{
    return (
        sdbusplus::message::object_path(DetectorIntf::namespace_path::value) /
        DetectorIntf::namespace_path::detector / detectorName);
}

GPIODetector::GPIODetector(sdbusplus::async::context& ctx, Events& leakEvents,
                           const config::DetectorConfig& config) :
    DetectorIntf(ctx, getObjectPath(config.name).str.c_str()), ctx(ctx),
    leakEvents(leakEvents), config(config),
    gpioInterface(ctx, config.name, config.pinName,
                  (config.polarity == config::PinPolarity::activeLow),
                  std::bind_front(&GPIODetector::updateGPIOStateAsync, this))
{
    pretty_name<false>(config.name);
    type<false>(config.type);

    ctx.spawn(gpioInterface.start());

    debug("Created leak detector {NAME}", "NAME", config.name);
}

sdbusplus::async::task<> GPIODetector::updateGPIOStateAsync(bool gpioState)
{
    auto newState = gpioState ? DetectorIntf::DetectorState::Abnormal
                              : DetectorIntf::DetectorState::Normal;

    debug("Updating detector {DETECTOR} state to {STATE}", "DETECTOR",
          config.name, "STATE", newState);

    if (newState != state_)
    {
        if (emitSignal)
        {
            state(newState);
        }
        else
        {
            state<false>(newState);
            emitSignal = true;
        }

        co_await leakEvents.generateLeakEvent(getObjectPath(config.name),
                                              state_, config.level);
        if (state_ != DetectorIntf::DetectorState::Normal)
        {
            for (const auto& [level, serviceSuffix] : config::leakActionTargets)
            {
                if (config.level == level)
                {
                    auto target = std::string(serviceSuffix) + config.name +
                                  ".service";
                    debug("Starting systemd target {TARGET}", "TARGET", target);
                    ctx.spawn(systemd::Systemd::startUnit(ctx, target));
                    break;
                }
            }
        }
    }

    co_return;
}

} // namespace leak
