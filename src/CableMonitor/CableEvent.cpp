#include "CableEvent.hpp"

#include <phosphor-logging/lg2.hpp>

PHOSPHOR_LOG2_USING;

namespace phosphor::cable::event
{

auto Event::generateCableEvent(Type type, std::string& name) -> void
{
    if (type == Type::connected)
    {
        info("Generate CableConnected for {NAME}", "NAME", name);
        // TODO: Resolve CableDisconnected event using lg2 API
        // TODO: Generate CableConnected event using lg2::commit API
    }
    else if (type == Type::disconnected)
    {
        warning("Generate CableDisconnected for {NAME}", "NAME", name);
        // TODO: Generate CableDisconnected event using lg2::commit API
    }
    else
    {
        error("Unknown cable event type");
    }
}

} // namespace phosphor::cable::event
