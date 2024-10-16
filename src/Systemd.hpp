#pragma once

#include <sdbusplus/bus.hpp>

#include <string>

namespace phosphor::systemd
{

class Systemd
{
  public:
    Systemd() = delete;
    explicit Systemd(sdbusplus::bus_t& bus) : bus(bus) {}

    /** @brief Start a systemd unit.
     * Returns true on success, otherwise false.
     */
    bool startUnit(const std::string& sysdUnit);

  private:
    sdbusplus::bus_t& bus;
};

} // namespace phosphor::systemd
