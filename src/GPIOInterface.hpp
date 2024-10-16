#pragma once

#include <gpiod.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>

namespace phosphor::gpio
{

PHOSPHOR_LOG2_USING;

template <typename Instance>
class GPIOInterface
{
  public:
    GPIOInterface() = delete;

    GPIOInterface(sdbusplus::async::context& ctx, const std::string serviceName,
                  std::string pinName) :
        ctx(ctx), serviceName(serviceName), pinName(pinName)
    {}

    /** @brief Read the gpio state */
    auto readGPIO() -> void
    {
        auto line = gpiod::find_line(pinName);
        if (!line)
        {
            throw std::runtime_error("Failed to find GPIO line for " + pinName);
        }

        int lineValue = 0;
        try
        {
            line.request(
                {serviceName, gpiod::line_request::DIRECTION_INPUT, 0});
            lineValue = line.get_value();
            if (lineValue < 0)
            {
                throw std::runtime_error(
                    "Invalid value " + std::to_string(lineValue) +
                    "read GPIO line for " + pinName);
            }
        }
        catch (std::exception& e)
        {
            throw std::runtime_error("Failed to read GPIO line for " + pinName +
                                     " with error " + e.what());
        }

        static_cast<Instance*>(this)->updateGPIOState(lineValue == 1);

        line.release();
    }

    /** @brief Configure GPIO for async read */
    auto configureAsyncRead() -> void
    {
        try
        {
            line.request(
                {serviceName, gpiod::line_request::EVENT_BOTH_EDGES, {}});
        }
        catch (std::exception& e)
        {
            throw std::runtime_error("Failed to request GPIO line for " +
                                     pinName + " with error " + e.what());
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

        ctx.spawn(readGPIOAsync());
    }

  private:
    /** @brief Read the gpio state asynchronously */
    auto readGPIOAsync() -> sdbusplus::async::task<>
    {
        while (!ctx.stop_requested())
        {
            info("Asynchronously reading state for {PINNAME}, {LINEFD}",
                 "PINNAME", pinName, "LINEFD", lineFd);
            // Wait for the fd event for the line to change
            co_await fdioInstance->next();
            info("Received event for {GPIO}, {LINEFD}", "GPIO", pinName,
                 "LINEFD", lineFd);
            auto lineEvent = line.event_read();
            bool state =
                (lineEvent.event_type == gpiod::line_event::RISING_EDGE);
            static_cast<Instance*>(this)->updateGPIOState(state);
        }
    }

    /** @brief D-Bus context */
    sdbusplus::async::context& ctx;
    /** @brief D-Bus service name */
    std::string serviceName;
    /** @brief gpio pin name */
    std::string pinName;
    /** @brief gpio line */
    gpiod::line line;
    /** @brief gpio line file descriptor */
    int lineFd = -1;
    /** File descriptor based async event handler */
    std::unique_ptr<sdbusplus::async::fdio> fdioInstance;
};

} // namespace phosphor::gpio
