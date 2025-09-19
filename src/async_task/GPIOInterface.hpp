#pragma once

#include <gpiod.hpp>
#include <sdbusplus/async.hpp>

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
    auto start() -> sdbusplus::async::task<>;

  private:
    /** @brief Read the gpio state asynchronously */
    auto readGPIOAsync() -> sdbusplus::async::task<>;

    /** @brief Read the gpio state asynchronously based on gpio event */
    auto readGPIOAsyncEvent() -> sdbusplus::async::task<>;

    sdbusplus::async::context& ctx;
    const std::string& pinName;
    Callback_t updateStateCallback;
    gpiod::line line;
    /** File descriptor based async event handler */
    std::unique_ptr<sdbusplus::async::fdio> fdioInstance;
};

} // namespace gpio
