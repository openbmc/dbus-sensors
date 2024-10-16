#pragma once

#include <gpiod.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/fdio.hpp>
#include <sdbusplus/async/task.hpp>

#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

namespace gpio
{

PHOSPHOR_LOG2_USING;

template <typename Instance>
class GPIOInterface
{
  public:
    GPIOInterface() = delete;

    GPIOInterface(sdbusplus::async::context& ctx,
                  const std::string& consumerName, const std::string& pinName,
                  bool activeLow) :
        ctx(ctx), consumerName(consumerName), pinName(pinName),
        activeLow(activeLow)
    {}

    /** @brief Configure GPIO async operations and read the current value */
    void configureGPIO()
    {
        line = gpiod::find_line(pinName);
        if (!line)
        {
            throw std::runtime_error("Failed to find GPIO line for " + pinName);
        }
        try
        {
            line.request(
                {consumerName, gpiod::line_request::EVENT_BOTH_EDGES,
                 activeLow ? gpiod::line_request::FLAG_ACTIVE_LOW : 0});
        }
        catch (std::exception& e)
        {
            throw std::runtime_error("Failed to request line for " + pinName +
                                     " with error " + e.what());
        }

        lineFd = line.event_get_fd();
        if (lineFd < 0)
        {
            throw std::runtime_error(
                "Failed to get event fd for GPIO line " + pinName);
        }

        fdioInstance = std::make_unique<sdbusplus::async::fdio>(ctx, lineFd);
        if (!fdioInstance)
        {
            throw std::runtime_error(
                "Failed to create fdio for GPIO line " + pinName);
        }

        // Start the async read for the GPIO line
        ctx.spawn(readGPIOAsyncEvent());

        // Read the initial GPIO value
        ctx.spawn(readGPIOAsync());
    }

  private:
    /** @brief Read the gpio state asynchronously */
    sdbusplus::async::task<> readGPIOAsync()
    {
        auto lineValue = line.get_value();
        if (lineValue < 0)
        {
            error("Failed to read GPIO line {LINENAME}", "LINENAME", pinName);
            co_return;
        }
        static_cast<Instance*>(this)->updateGPIOState(
            lineValue == gpiod::line_event::RISING_EDGE);

        co_return;
    }

    /** @brief Read the gpio state asynchronously based on gpio event */
    sdbusplus::async::task<> readGPIOAsyncEvent()
    {
        while (!ctx.stop_requested())
        {
            // Wait for the fd event for the line to change
            co_await fdioInstance->next();

            line.event_read();
            auto lineValue = line.get_value();

            co_await static_cast<Instance*>(this)->updateGPIOStateAsync(
                lineValue == gpiod::line_event::RISING_EDGE);
        }
    }

    sdbusplus::async::context& ctx;
    const std::string& consumerName;
    const std::string& pinName;
    bool activeLow = false;
    gpiod::line line;
    int lineFd = -1;
    /** File descriptor based async event handler */
    std::unique_ptr<sdbusplus::async::fdio> fdioInstance;
};

} // namespace gpio
