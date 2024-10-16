#include "LeakDetector.hpp"

#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <exception>
#include <string>
#include <unordered_map>

namespace phosphor::leak::detector
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

auto LeakDetector::updateGPIOState(bool gpioState) -> void
{
    ctx.spawn(updateGPIOStateAsync(gpioState));
}

// NOLINTNEXTLINE(readability-static-accessed-through-instance)
auto LeakDetector::updateGPIOStateAsync(bool gpioState)
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
            if (config.runTarget)
            {
                auto target = config::leakActionTargets.at(config.level) +
                              config.name + ".service";
                debug("Starting systemd target {TARGET}", "TARGET", target);
                systemd.startUnit(target);
            }
        }
    }

    co_return;
}

auto LeakDetector::createLeakDetector() -> void
{
    pretty_name<false>(config.name);
    type<false>(config.type);
    try
    {
        configureGPIO();
    }
    catch (std::exception& e)
    {
        throw;
    }
    debug("Created leak detector {NAME}", "NAME", config.name);
}

auto LeakDetector::getObjectPath(const std::string& detectorName)
    -> sdbusplus::message::object_path
{
    return (
        sdbusplus::message::object_path(DetectorIntf::namespace_path::value) /
        DetectorIntf::namespace_path::detector / detectorName);
}

} // namespace phosphor::leak::detector
