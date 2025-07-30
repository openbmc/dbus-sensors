#pragma once

#include <gpiod.hpp>
#include <sdbusplus/async.hpp>

#include <functional>
#include <memory>
#include <string>

namespace gpio
{

/** @brief Class to handle General Purpose Output interface */
class GPOInterface
{
  public:
    GPOInterface() = delete;

    GPOInterface(sdbusplus::async::context& ctx,
                 const std::string& consumerName, const std::string& pinName);

    /** @brief Set the GPO value */
    auto setValue(bool value) -> bool;

  private:
    sdbusplus::async::context& ctx;
    const std::string& pinName;
    gpiod::line line;
};

} // namespace gpio
