#pragma once

#include <gpiod.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/fdio.hpp>
#include <sdbusplus/async/task.hpp>

#include <functional>
#include <memory>
#include <string>

namespace gpio
{

class GPIOInterface
{
  public:
    using Callback_t = std::function<sdbusplus::async::task<>(bool)>;

    GPIOInterface() = delete;

    GPIOInterface(sdbusplus::async::context& ctx,
                  const std::string& consumerName, const std::string& pinName,
                  bool activeLow, Callback_t updateStateCallback);

    /** @brief Start the GPIO Interface */
    sdbusplus::async::task<> start();

  private:
    /** @brief Read the gpio state asynchronously */
    sdbusplus::async::task<> readGPIOAsync();

    /** @brief Read the gpio state asynchronously based on gpio event */
    sdbusplus::async::task<> readGPIOAsyncEvent();

    sdbusplus::async::context& ctx;
    const std::string& pinName;
    Callback_t updateStateCallback;
    gpiod::line line;
    /** File descriptor based async event handler */
    std::unique_ptr<sdbusplus::async::fdio> fdioInstance;
};

} // namespace gpio
