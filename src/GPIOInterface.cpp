#include "GPIOInterface.hpp"

#include <gpiod.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>

#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace gpio
{

PHOSPHOR_LOG2_USING;

GPIOInterface::GPIOInterface(sdbusplus::async::context& ctx,
                             const std::string& consumerName,
                             const std::string& pinName, bool activeLow,
                             Callback_t updateStateCallback) :
    ctx(ctx), pinName(pinName),
    updateStateCallback(std::move(updateStateCallback))
{
    if (!this->updateStateCallback)
    {
        throw std::runtime_error("updateStateCallback is not set");
    }
    line = gpiod::find_line(pinName);
    if (!line)
    {
        throw std::runtime_error("Failed to find GPIO line for " + pinName);
    }
    try
    {
        line.request({consumerName, gpiod::line_request::EVENT_BOTH_EDGES,
                      activeLow ? gpiod::line_request::FLAG_ACTIVE_LOW : 0});
    }
    catch (std::exception& e)
    {
        throw std::runtime_error("Failed to request line for " + pinName +
                                 " with error " + e.what());
    }

    int lineFd = line.event_get_fd();
    if (lineFd < 0)
    {
        throw std::runtime_error(
            "Failed to get event fd for GPIO line " + pinName);
    }

    fdioInstance = std::make_unique<sdbusplus::async::fdio>(ctx, lineFd);
}

GPIOInterface::GPIOInterface(sdbusplus::async::context& ctx,
                             const std::string& consumerName,
                             const std::string& pinName) :
    ctx(ctx), pinName(pinName)
{
    line = gpiod::find_line(pinName);
    if (!line)
    {
        throw std::runtime_error("Failed to find GPIO line for " + pinName);
    }
    try
    {
        line.request({consumerName, gpiod::line_request::DIRECTION_OUTPUT, 0});
    }
    catch (std::exception& e)
    {
        throw std::runtime_error("Failed to request line for " + pinName +
                                 " with error " + e.what());
    }

    // Set input mode to false as GPIO is in output mode
    inputMode = false;
}

auto GPIOInterface::start() -> sdbusplus::async::task<>
{
    if (!inputMode)
    {
        debug("GPIOInterface is not in input mode");
        co_return;
    }

    // Start the async read for the GPIO line
    ctx.spawn(readGPIOAsyncEvent());

    // Read the initial GPIO value
    co_await readGPIOAsync();
}

auto GPIOInterface::setValue(bool value) -> bool
{
    if (inputMode)
    {
        error("GPIOInterface is in input mode, so set value is not allowed");
        return false;
    }

    try
    {
        line.set_value(value ? 1 : 0);
    }
    catch (std::exception& e)
    {
        error("Failed to set value for GPIO line {LINENAME} with error {ERROR}",
              "LINENAME", pinName, "ERROR", e);
        return false;
    }

    return true;
}

auto GPIOInterface::readGPIOAsync() -> sdbusplus::async::task<>
{
    auto lineValue = line.get_value();
    if (lineValue < 0)
    {
        error("Failed to read GPIO line {LINENAME}", "LINENAME", pinName);
        co_return;
    }
    co_await updateStateCallback(lineValue == gpiod::line_event::RISING_EDGE);

    co_return;
}

auto GPIOInterface::readGPIOAsyncEvent() -> sdbusplus::async::task<>
{
    while (!ctx.stop_requested())
    {
        // Wait for the fd event for the line to change
        co_await fdioInstance->next();

        line.event_read();
        auto lineValue = line.get_value();

        co_await updateStateCallback(
            lineValue == gpiod::line_event::RISING_EDGE);
    }

    co_return;
}

} // namespace gpio
