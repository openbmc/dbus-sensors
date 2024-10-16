#pragma once

#include <sdbusplus/async.hpp>

#include <string>

namespace systemd
{

class Systemd
{
  public:
    /** @brief Start a systemd unit.
     * Returns true on success, otherwise false.
     */
    static sdbusplus::async::task<> startUnit(sdbusplus::async::context& ctx,
                                              std::string sysdUnit);
};

} // namespace systemd
