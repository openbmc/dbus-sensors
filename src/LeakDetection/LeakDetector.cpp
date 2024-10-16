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

    info("Read state {STATE} for {PINNAME}", "STATE", lineValue, "PINNAME",
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

    info("setupAsyncEvent: Event fd {LINEFD} for {PINNAME}", "LINEFD", lineFd,
         "PINNAME", config.pinName);

    ctx.spawn(readStateAsync());

    return true;
}

auto LeakDetector::readStateAsync() -> sdbusplus::async::task<>
{
    while (!ctx.stop_requested())
    {
        info("Asynchronously reading state for {PINNAME}, {LINEFD}", "PINNAME",
             config.pinName, "LINEFD", lineFd);
        // Need to wait on the event fd for the line to change
        co_await sdbusplus::async::async_fdio(ctx, lineFd);
        info("Received event for {PINNAME}, {LINEFD}", "PINNAME",
             config.pinName, "LINEFD", lineFd);
        auto lineEvent = line.event_read();
        bool state = (lineEvent.event_type == gpiod::line_event::RISING_EDGE);
        updateState(state);
    }
    // info("Start Spawn"):
    // ctx.spawn(readStateAsync());
    // info("End Spawn");
    // co_return;
}

auto LeakDetector::updateState(bool state) -> void
{
    bool newState =
        (config.polarity == config::Polarity::activeLow) ? !state : state;
    if (newState != state_)
    {
        state_ = newState;
        if (state_)
        {
            info("Leak detected on {PINNAME}", "PINNAME", config.pinName);
            // Call the leak event handler
            // Start the systemd target
        }
        else
        {
            info("Leak cleared on {PINNAME}", "PINNAME", config.pinName);
            // Call the leak cleared event handler
        }
    }
    else
    {
        info("Leak state unchanged on {PINNAME}", "PINNAME", config.pinName);
    }
}

namespace config
{
auto processConfig(const SensorBaseConfigMap& properties)
    -> std::optional<DetectorConfig>
{
    DetectorConfig config = {};

    config.name = loadVariant<std::string>(properties, propertyName);
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

    info("Detector config: {NAME} {PINNAME} {POLARITY} {LEVEL} {ACTION}",
         "NAME", config.name, "PINNAME", config.pinName, "POLARITY",
         config.polarity, "LEVEL", config.level, "ACTION", config.action);

    return config;
}
} // namespace config

} // namespace phosphor::leak::detector
