#include "LeakUtils.hpp"

#include <phosphor-logging/lg2.hpp>

PHOSPHOR_LOG2_USING;

namespace phosphor::leak::utils
{

void resolveLeakEvent(sdbusplus::async::context&, const std::string& name,
                      Level level)
{
    if (level == Level::critical)
    {
        error("Critical leak resolved for {NAME}", "NAME", name);
        // TODO: Resolve LeakDetectedCritical event using lg2::commit API
    }
    else
    {
        warning("Warning leak resolved for {NAME}", "NAME", name);
        // TODO: Resolve LeakDetectedWarning event using lg2::commit API
    }
    // TODO: Generate LeakDetectedNormal event using lg2::commit API
}

void generateLeakEvent(sdbusplus::async::context& ctx, const std::string& name,
                       bool state, Level level)
{
    if (state)
    {
        if (level == Level::critical)
        {
            error("Critical leak detected for {NAME}", "NAME", name);
            // TODO: Generate LeakDetectedCritical event using lg2::commit API
        }
        else
        {
            warning("Warning leak detected for {NAME}", "NAME", name);
            // TODO: Generate LeakDetectedWarning event using lg2::commit API
        }
    }
    else
    {
        resolveLeakEvent(ctx, name, level);
    }
}
} // namespace phosphor::leak::utils
