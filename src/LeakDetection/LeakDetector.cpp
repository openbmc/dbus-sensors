#include "LeakDetector.hpp"

#include "fdio.hpp"

namespace phosphor::leak::detector
{
auto LeakDetector::getPath(std::string name) -> std::string
{
    return std::string(PathIntf::value) + "/" + PathIntf::leak_detector + "/" +
           name;
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
    bool newState =
        (config.polarity == config::Polarity::activeLow) ? !state : state;
    if (newState != state_)
    {
        state_ = newState;
        utils::generateLeakEvent(ctx, config.name, state_, config.level);
        if (state_)
        {
            if (config.action != config::Action::unknown)
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
    auto type = validDetectorTypes.find(
        loadVariant<std::string>(properties, propertySubType));
    if (type == validDetectorTypes.end())
    {
        warning("Invalid detector type {TYPE} for detector {DETECTOR}", "TYPE",
                type->first, "DETECTOR", config.name);
        config.type = LeakDetectorIntf::DetectorType::Unknown;
    }
    else
    {
        config.type = type->second;
    }
    config.pinName = loadVariant<std::string>(properties, propertyPinName);
    auto polarity = validPolarity.find(
        loadVariant<std::string>(properties, propertyPolarity));
    if (polarity == validPolarity.end())
    {
        error("Invalid polarity {POLARITY} for detector {DETECTOR}", "POLARITY",
              polarity->first, "DETECTOR", config.name);
        return std::nullopt;
    }
    config.polarity = polarity->second;
    try
    {
        auto level = validLevels.find(
            loadVariant<std::string>(properties, propertyLevel));
        if (level == validLevels.end())
        {
            warning("Invalid level {LEVEL} for detector {DETECTOR}", "LEVEL",
                    level->first, "DETECTOR", config.name);
        }
        else
        {
            config.level = level->second;
        }
    }
    catch (std::exception& e)
    {
        warning("Level missing for detector {DETECTOR}", "DETECTOR",
                config.name);
    }

    try
    {
        auto defaultAction = validActions.find(
            loadVariant<std::string>(properties, propertyDefaultAction));
        if (defaultAction == validActions.end())
        {
            warning("Invalid default action {ACTION} for detector {DETECTOR}",
                    "ACTION", defaultAction->first, "DETECTOR", config.name);
        }
        else
        {
            config.action = defaultAction->second;
        }
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
