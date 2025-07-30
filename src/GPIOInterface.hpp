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

    /** @brief Constructor to configure the GPIO interface in read event mode */
    GPIOInterface(sdbusplus::async::context& ctx,
                  const std::string& consumerName, const std::string& pinName,
                  bool activeLow, Callback_t updateStateCallback);

    /** @brief Constructor to configure the GPIO interface in set/write mode */
    GPIOInterface(sdbusplus::async::context& ctx,
                  const std::string& consumerName, const std::string& pinName);

    /** @brief Start the GPIO Interface */
    auto start() -> sdbusplus::async::task<>;

    /** @brief Set the GPIO value. Only valid in set/write mode */
    auto setValue(bool value) -> bool;

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
    /** Is GPIOInterface in Input Direction. True for read event mode, false
     * otherwise  */
    bool inputMode = true;
};

} // namespace gpio
