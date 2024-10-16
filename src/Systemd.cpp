#include "Systemd.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/message.hpp>

#include <exception>
#include <string>

PHOSPHOR_LOG2_USING;

namespace phosphor::systemd
{

auto Systemd::startUnit(const std::string& sysdUnit) -> bool
{
    if (sysdUnit.empty())
    {
        error("sysdUnit is empty");
        return false;
    }
    sdbusplus::message_t msg = bus.new_method_call(
        "org.freedesktop.systemd1", "/org/freedesktop/systemd1",
        "org.freedesktop.systemd1.Manager", "StartUnit");
    msg.append(sysdUnit, "replace");
    try
    {
        bus.call_noreply(msg);
    }
    catch (const std::exception& e)
    {
        warning("Failed to start {UNIT}: {ERROR}", "UNIT", sysdUnit, "ERROR",
                e.what());
        return false;
    }
    return true;
}

} // namespace phosphor::systemd
