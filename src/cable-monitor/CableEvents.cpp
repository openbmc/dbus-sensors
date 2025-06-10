#include "CableEvents.hpp"

#include <phosphor-logging/commit.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <xyz/openbmc_project/State/Cable/event.hpp>

#include <string>

PHOSPHOR_LOG2_USING;

namespace cable
{

auto Events::generateCableEvent(Type type, std::string name)
    -> sdbusplus::async::task<>
{
    // Added NO_LINT to bypass clang-tidy warning about STDEXEC_ASSERT as clang
    // seems to be confused about context being uninitialized.
    // NOLINTNEXTLINE(clang-analyzer-core.uninitialized.Branch)
    if (type == Type::connected)
    {
        auto pendingEvent = pendingEvents.find(name);
        if (pendingEvent != pendingEvents.end())
        {
            co_await lg2::resolve(ctx, pendingEvent->second);

            using CableConnected = sdbusplus::event::xyz::openbmc_project::
                state::Cable::CableConnected;
            co_await lg2::commit(ctx, CableConnected("PORT_ID", name));
            pendingEvents.erase(pendingEvent);
        }
    }
    else if (type == Type::disconnected)
    {
        using CableDisconnected = sdbusplus::error::xyz::openbmc_project::
            state::Cable::CableDisconnected;
        auto eventPath =
            co_await lg2::commit(ctx, CableDisconnected("PORT_ID", name));
        warning("Generate CableDisconnected for {NAME}", "NAME", name);
        pendingEvents.emplace(name, eventPath);
    }
    else
    {
        error("Unknown cable event type");
    }
    co_return;
}

} // namespace cable
