#pragma once

#include <sdbusplus/async.hpp>

#include <string>

namespace systemd
{

class SystemdInterface
{
  public:
    /** @brief Start a systemd unit.
     * Returns true on success, otherwise false.
     */
    static auto startUnit(sdbusplus::async::context& ctx, std::string sysdUnit)
        -> sdbusplus::async::task<>;
};

} // namespace systemd
