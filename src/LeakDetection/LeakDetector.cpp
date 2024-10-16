#include "LeakDetector.hpp"

namespace phosphor::leak::detector
{

using DetectorStateIntf =
    sdbusplus::common::xyz::openbmc_project::state::leak::Detector;

auto LeakDetector::getPath(std::string name) -> std::string
{
    return std::string(PathIntf::value) + "/" + PathIntf::detector + "/" + name;
}

auto LeakDetector::readState() -> bool
{
    line = gpiod::find_line(config.pinName);
    if (!line)
    {
        error("Failed to find GPIO line {GPIO_LINE}", "GPIO_LINE",
              config.pinName);
        return false;
    }

    line.request({serviceName, gpiod::line_request::DIRECTION_INPUT, 0});
    int lineValue = 0;
    try
    {
        lineValue = line.get_value();
        if (lineValue < 0)
        {
            error("Failed to read GPIO line {GPIO_LINE}", "GPIO_LINE",
                  config.pinName);
            return false;
        }
    }
    catch (std::exception& e)
    {
        error("Failed to read GPIO line {GPIO_LINE}: {ERROR}", "GPIO_LINE",
              config.pinName, "ERROR", e.what());
        return false;
    }

    debug("Read state {STATE} for {PINNAME}", "STATE", lineValue, "PINNAME",
          config.pinName);

    updateState(lineValue == 1);

    line.release();

    return true;
}

auto LeakDetector::setupAsyncEvent() -> bool
{
    try
    {
        line.request({serviceName, gpiod::line_request::EVENT_BOTH_EDGES, {}});
    }
    catch (std::exception& e)
    {
        error("Failed to request GPIO line {GPIO_LINE}: {ERROR}", "GPIO_LINE",
              config.pinName, "ERROR", e.what());
        return false;
    }

    lineFd = line.event_get_fd();
    if (lineFd < 0)
    {
        error("Failed to get event fd for GPIO line {GPIO_LINE}", "GPIO_LINE",
              config.pinName);
        return false;
    }

    ctx.spawn(readStateAsync());

    debug("setupAsyncEvent: Event fd {LINEFD} for {PINNAME}", "LINEFD", lineFd,
          "PINNAME", config.pinName);

    return true;
}

auto LeakDetector::readStateAsync() -> sdbusplus::async::task<>
{
    while (!ctx.stop_requested())
    {
        info("Asynchronously reading state for {PINNAME}, {LINEFD}", "PINNAME",
             config.pinName, "LINEFD", lineFd);
        // Wait for the fd event for the line to change
        co_await sdbusplus::async::async_fdio(ctx, lineFd);
        debug("Received event for {DETECTOR}, {LINEFD}", "DETECTOR",
              config.name, "LINEFD", lineFd);
        auto lineEvent = line.event_read();
        bool state = (lineEvent.event_type == gpiod::line_event::RISING_EDGE);
        updateState(state);
    }
}

auto LeakDetector::updateState(bool state) -> void
{
    auto newState =
        ((config.polarity == CommonConfigIntf::PinPolarity::ActiveLow)
             ? !state
             : state)
            ? DetectorIntf::DetectorState::Abnormal
            : DetectorIntf::DetectorState::Normal;
    if (newState != state_)
    {
        state_ = newState;
        utils::generateLeakEvent(ctx, config.name, state_, config.level);
        if (state_ != DetectorIntf::DetectorState::Normal)
        {
            if (config.action !=
                DetectorConfigIntf::DetectorDefaultAction::Unknown)
            {
                debug("Starting systemd target {TARGET}", "TARGET",
                      config.action);
                startUnit(ctx.get_bus(),
                          config::actionTargets.at(config.action));
            }
            else
            {
                warning("No action configured for detector {DETECTOR}",
                        "DETECTOR", config.name);
            }
        }
    }
}

namespace config
{
auto processConfig(const SensorBaseConfigMap& properties)
    -> std::optional<DetectorConfig>
{
    DetectorConfig config = {};

    config.name = loadVariant<std::string>(properties, propertyName);
    config.type = DetectorStateIntf::convertDetectorTypeFromString(
        loadVariant<std::string>(properties, propertySubType));

    config.pinName = loadVariant<std::string>(properties, propertyPinName);
    config.polarity = CommonConfigIntf::convertPinPolarityFromString(
        loadVariant<std::string>(properties, propertyPolarity));

    try
    {
        config.level = DetectorConfigIntf::convertDetectorLevelFromString(
            loadVariant<std::string>(properties, propertyLevel));
    }
    catch (std::exception& e)
    {
        warning("Level missing for detector {DETECTOR}", "DETECTOR",
                config.name);
    }

    try
    {
        config.action =
            DetectorConfigIntf::convertDetectorDefaultActionFromString(
                loadVariant<std::string>(properties, propertyDefaultAction));
    }
    catch (std::exception& e)
    {
        warning("Default action missing for detector {DETECTOR}", "DETECTOR",
                config.name);
    }

    debug("Detector config: {NAME} {PINNAME} {POLARITY} {LEVEL} {ACTION}",
          "NAME", config.name, "PINNAME", config.pinName, "POLARITY",
          config.polarity, "LEVEL", config.level, "ACTION", config.action);

    return config;
}
} // namespace config

} // namespace phosphor::leak::detector
