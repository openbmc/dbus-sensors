#include "LeakDetector.hpp"

namespace phosphor::leak::detector
{

namespace config
{

/** @brief Action name to systemd target map */
static const auto actionTargets =
    std::unordered_map<DetectorConfigIntf::DetectorDefaultAction, std::string>{
        {DetectorConfigIntf::DetectorDefaultAction::ChassisPowerOff,
         "xyz.openbmc_project.State.Chassis.PowerOff"},
        {DetectorConfigIntf::DetectorDefaultAction::RackPowerOff,
         "xyz.openbmc_project.State.Rack.PowerOff"}};

auto processConfig(sdbusplus::async::context& ctx,
                   const sdbusplus::message::object_path& objectPath)
    -> sdbusplus::async::task<DetectorConfig>
{
    DetectorConfig config = {};

    constexpr auto EMServiceName = "xyz.openbmc_project.EntityManager";
    auto client =
        DetectorConfigIntf(ctx).service(EMServiceName).path(objectPath.str);

    config.name = co_await client.name();
    config.type = co_await client.sub_type();
    config.pinName = co_await client.pin_name();
    config.polarity = co_await client.polarity();
    config.level = co_await client.level();
    config.action = co_await client.default_action();

    debug("Detector config: {NAME} {PINNAME} {POLARITY} {LEVEL} {ACTION}",
          "NAME", config.name, "PINNAME", config.pinName, "POLARITY",
          config.polarity, "LEVEL", config.level, "ACTION", config.action);

    co_return config;
}

} // namespace config

auto LeakDetector::createLeakDetector() -> void
{
    pretty_name(config.name);
    type(config.type);
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

auto LeakDetector::getObjectPath(std::string detectorName)
    -> sdbusplus::message::object_path
{
    return (
        sdbusplus::message::object_path(DetectorIntf::namespace_path::value) /
        DetectorIntf::namespace_path::detector / detectorName);
}

auto LeakDetector::updateGPIOState(bool gpioState) -> void
{
    ctx.spawn(updateGPIOStateAsync(gpioState));
}

auto LeakDetector::updateGPIOStateAsync(bool gpioState)
    -> sdbusplus::async::task<>
{
    auto newState = gpioState ? DetectorIntf::DetectorState::Abnormal
                              : DetectorIntf::DetectorState::Normal;

    debug("Updating detector {DETECTOR} state to {STATE}", "DETECTOR",
          config.name, "STATE", newState);

    if (newState != state_)
    {
        state(newState);
        co_await leakEvents.generateLeakEvent(getObjectPath(config.name),
                                              state_, config.level);
        if (state_ != DetectorIntf::DetectorState::Normal)
        {
            if (config.action !=
                DetectorConfigIntf::DetectorDefaultAction::Unknown)
            {
                debug("Starting systemd target {TARGET}", "TARGET",
                      config.action);
                systemd.startUnit(config::actionTargets.at(config.action));
            }
        }
    }

    co_return;
}

} // namespace phosphor::leak::detector
