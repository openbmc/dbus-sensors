#pragma once

#include <sdbusplus/async/context.hpp>
#include <sdbusplus/bus.hpp>

#include <string>

namespace systemd
{

class Systemd
{
  public:
    /** @brief Start a systemd unit.
     * Returns true on success, otherwise false.
     */
    sdbusplus::async::task<bool> startUnit(sdbusplus::async::context& ctx,
                                           const std::string& sysdUnit);
};

} // namespace systemd
