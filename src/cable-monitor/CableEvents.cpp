#include "CableEvents.hpp"

#include <phosphor-logging/commit.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Logging/Entry/client.hpp>
#include <xyz/openbmc_project/State/Cable/event.hpp>

#include <string>

PHOSPHOR_LOG2_USING;

namespace cable
{

// NOLINTNEXTLINE(readability-static-accessed-through-instance)
sdbusplus::async::task<> Events::resolveCableEvent(
    sdbusplus::message::object_path eventPath)
{
    using LoggingEntry =
        sdbusplus::client::xyz::openbmc_project::logging::Entry<>;

    auto entryClient = LoggingEntry(ctx)
                           .service(LoggingEntry::default_service)
                           .path(eventPath.str);
    co_await entryClient.resolved(true);
}

// NOLINTNEXTLINE(readability-static-accessed-through-instance)
sdbusplus::async::task<> Events::generateCableEvent(Type type, std::string name)
{
    if (type == Type::connected)
    {
        auto pendingEvent = pendingEvents.find(name);
        if (pendingEvent != pendingEvents.end())
        {
            co_await resolveCableEvent(pendingEvent->second);

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
