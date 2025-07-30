#include "GPIInterface.hpp"

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

GPIInterface::GPIInterface(sdbusplus::async::context& ctx,
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
        throw std::runtime_error("Failed to find GPI line for " + pinName);
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
            "Failed to get event fd for GPI line " + pinName);
    }

    fdioInstance = std::make_unique<sdbusplus::async::fdio>(ctx, lineFd);
}

auto GPIInterface::start() -> sdbusplus::async::task<>
{
    // Start the async read for the GPI line
    ctx.spawn(readGPIAsyncEvent());

    // Read the initial GPI value
    co_await readGPIAsync();
}

auto GPIInterface::readGPIAsync() -> sdbusplus::async::task<>
{
    auto lineValue = line.get_value();
    if (lineValue < 0)
    {
        error("Failed to read GPI line {LINENAME}", "LINENAME", pinName);
        co_return;
    }
    co_await updateStateCallback(lineValue == gpiod::line_event::RISING_EDGE);

    co_return;
}

auto GPIInterface::readGPIAsyncEvent() -> sdbusplus::async::task<>
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
