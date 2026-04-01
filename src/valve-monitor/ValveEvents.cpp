#include "ValveEvents.hpp"

#include <phosphor-logging/commit.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/State/Valve/event.hpp>

namespace valve
{

PHOSPHOR_LOG2_USING;

auto Events::generateValveEvent(sdbusplus::message::object_path valvePath,
                                bool open) -> sdbusplus::async::task<>
{
    namespace event_intf = sdbusplus::event::xyz::openbmc_project::state::Valve;

    if (open)
    {
        co_await lg2::commit(ctx,
                             event_intf::ValveOpen("VALVE_NAME", valvePath));
    }
    else
    {
        co_await lg2::commit(ctx,
                             event_intf::ValveClose("VALVE_NAME", valvePath));
    }

    debug("Valve event generated for {PATH} with open={STATUS}", "PATH",
          valvePath, "STATUS", open);
}

auto Events::handleValveSetPointWarning(
    sdbusplus::message::object_path valvePath, bool asserted)
    -> sdbusplus::async::task<>
{
    namespace error_intf = sdbusplus::error::xyz::openbmc_project::state::Valve;

    bool currentlyAsserted = pendingEvents.contains(valvePath.str);

    if (asserted && !currentlyAsserted)
    {
        // New warning — commit the event and track it
        auto eventPath = co_await lg2::commit(
            ctx,
            error_intf::ValveUnableToReachSetPoint("VALVE_NAME", valvePath));

        pendingEvents[valvePath.str] = eventPath;
        warning("Valve unable to reach set point for {PATH}", "PATH",
                valvePath);
    }
    else if (!asserted && currentlyAsserted)
    {
        // Warning cleared — resolve the pending event
        co_await lg2::resolve(ctx, pendingEvents[valvePath.str]);
        pendingEvents.erase(valvePath.str);
        debug("Valve set point warning resolved for {PATH}", "PATH", valvePath);
    }
}

} // namespace valve
