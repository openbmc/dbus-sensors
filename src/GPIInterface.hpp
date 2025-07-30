#pragma once

#include <gpiod.hpp>
#include <sdbusplus/async.hpp>

#include <functional>
#include <memory>
#include <string>

namespace gpio
{

/** @brief Class to handle General Purpose Input interface */
class GPIInterface
{
  public:
    using Callback_t = std::function<sdbusplus::async::task<>(bool)>;

    GPIInterface() = delete;

    /** @brief Constructor to configure the GPI interface in read event mode */
    GPIInterface(sdbusplus::async::context& ctx,
                 const std::string& consumerName, const std::string& pinName,
                 bool activeLow, Callback_t updateStateCallback);

    /** @brief Start the GPI Interface */
    auto start() -> sdbusplus::async::task<>;

  private:
    /** @brief Read the gpi state asynchronously */
    auto readGPIAsync() -> sdbusplus::async::task<>;

    /** @brief Read the gpi state asynchronously based on gpi event */
    auto readGPIAsyncEvent() -> sdbusplus::async::task<>;

    sdbusplus::async::context& ctx;
    const std::string& pinName;
    Callback_t updateStateCallback;
    gpiod::line line;
    /** File descriptor based async event handler */
    std::unique_ptr<sdbusplus::async::fdio> fdioInstance;
};

} // namespace gpio
