#include "GPOInterface.hpp"

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

GPOInterface::GPOInterface(sdbusplus::async::context& ctx,
                           const std::string& consumerName,
                           const std::string& pinName) :
    ctx(ctx), pinName(pinName)
{
    line = gpiod::find_line(pinName);
    if (!line)
    {
        throw std::runtime_error("Failed to find output line for " + pinName);
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
}

auto GPOInterface::setValue(bool value) -> bool
{
    try
    {
        line.set_value(value ? 1 : 0);
    }
    catch (std::exception& e)
    {
        error(
            "Failed to set value for output line {LINENAME} with error {ERROR}",
            "LINENAME", pinName, "ERROR", e);
        return false;
    }

    return true;
}

} // namespace gpio
