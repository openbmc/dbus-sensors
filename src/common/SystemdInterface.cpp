#include "SystemdInterface.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <exception>
#include <string>

PHOSPHOR_LOG2_USING;

namespace systemd
{

auto SystemdInterface::startUnit(sdbusplus::async::context& ctx,
                                 std::string sysdUnit)
    -> sdbusplus::async::task<>
{
    if (sysdUnit.empty())
    {
        error("sysdUnit is empty");
        co_return;
    }

    try
    {
        constexpr auto systemd =
            sdbusplus::async::proxy()
                .service("org.freedesktop.systemd1")
                .path("/org/freedesktop/systemd1")
                .interface("org.freedesktop.systemd1.Manager");

        sdbusplus::message::object_path jobObjectPath =
            co_await systemd.call<sdbusplus::message::object_path>(
                ctx, "StartUnit", sysdUnit, "replace");

        debug("Started {UNIT} with {JOBID}", "UNIT", sysdUnit, "JOBID",
              jobObjectPath.str);
    }
    catch (const std::exception& e)
    {
        warning("Failed to start {UNIT}: {ERROR}", "UNIT", sysdUnit, "ERROR",
                e);
        co_return;
    }

    co_return;
}

} // namespace systemd
