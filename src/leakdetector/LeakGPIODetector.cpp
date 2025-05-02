#include "LeakGPIODetector.hpp"

#include "LeakEvents.hpp"
#include "Systemd.hpp"

#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <exception>
#include <string>
#include <unordered_map>

namespace leak
{

namespace config
{

/** @brief Map for Leak level to systemd target service */
static const auto leakActionTargets =
    std::unordered_map<config::DetectorLevel, std::string>{
        {config::DetectorLevel::warning,
         "xyz.openbmc_project.leakdetector.warning@"},
        {config::DetectorLevel::critical,
         "xyz.openbmc_project.leakdetector.critical@"}};

} // namespace config

GPIODetector::GPIODetector(sdbusplus::async::context& ctx, Events& leakEvents,
                           const config::DetectorConfig& config) :
    DetectorIntf(ctx, getObjectPath(config.name).str.c_str()),
    GPIOIntf(ctx, config.name, config.pinName,
             (config.polarity == config::PinPolarity::activeLow)),
    ctx(ctx), leakEvents(leakEvents), config(config)
{
    pretty_name<false>(config.name);
    type<false>(config.type);

    debug("Created leak detector {NAME}", "NAME", config.name);
}

auto GPIODetector::updateGPIOState(bool gpioState) -> void
{
    ctx.spawn(updateGPIOStateAsync(gpioState));
}

auto GPIODetector::updateGPIOStateAsync(bool gpioState)
    -> sdbusplus::async::task<>
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
            try
            {
                auto target = config::leakActionTargets.at(config.level) +
                              config.name + ".service";
                debug("Starting systemd target {TARGET}", "TARGET", target);
                ctx.spawn(systemd::Systemd::startUnit(ctx, target));
            }
            catch (const std::exception& e)
            {
                error(
                    "Failed to find leak action target for level {LEVEL} with {ERROR}",
                    "LEVEL", config.level, "ERROR", e.what());
            }
        }
    }

    co_return;
}

auto GPIODetector::getObjectPath(const std::string& detectorName)
    -> sdbusplus::message::object_path
{
    return (
        sdbusplus::message::object_path(DetectorIntf::namespace_path::value) /
        DetectorIntf::namespace_path::detector / detectorName);
}

} // namespace leak
