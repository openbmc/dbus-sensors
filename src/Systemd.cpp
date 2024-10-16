#include "Systemd.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <exception>
#include <string>
#include <variant>

PHOSPHOR_LOG2_USING;

namespace systemd
{

sdbusplus::async::task<> Systemd::startUnit(sdbusplus::async::context& ctx,
                                            std::string sysdUnit)
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

        std::variant<sdbusplus::message::object_path> jobObjectPath =
            co_await systemd
                .call<std::variant<sdbusplus::message::object_path>>(
                    ctx, "StartUnit", sysdUnit, "replace");

        debug("Started {UNIT} with {JOBID}", "UNIT", sysdUnit, "JOBID",
              std::get<sdbusplus::message::object_path>(jobObjectPath));
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
