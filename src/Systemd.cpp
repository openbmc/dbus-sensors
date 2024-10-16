#include "Systemd.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/proxy.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <exception>
#include <string>
#include <variant>

PHOSPHOR_LOG2_USING;

namespace systemd
{

// NOLINTBEGIN(readability-static-accessed-through-instance,
// cppcoreguidelines-avoid-reference-coroutine-parameters)
sdbusplus::async::task<bool> Systemd::startUnit(sdbusplus::async::context& ctx,
                                                const std::string& sysdUnit)
// NOLINTEND(readability-static-accessed-through-instance,
// cppcoreguidelines-avoid-reference-coroutine-parameters)
{
    if (sysdUnit.empty())
    {
        error("sysdUnit is empty");
        co_return false;
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
              std::get<sdbusplus::message::object_path>(jobObjectPath)
                  .filename());
    }
    catch (const std::exception& e)
    {
        warning("Failed to start {UNIT}: {ERROR}", "UNIT", sysdUnit, "ERROR",
                e.what());
        co_return false;
    }

    co_return true;
}

} // namespace systemd
