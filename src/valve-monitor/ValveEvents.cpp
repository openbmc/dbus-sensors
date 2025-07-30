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

} // namespace valve
