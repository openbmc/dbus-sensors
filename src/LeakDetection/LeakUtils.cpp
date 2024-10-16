#include "LeakUtils.hpp"

namespace phosphor::leak::utils
{
void generateLeakEvent(std::string& name, Level level)
{
    if (level == Level::critical)
    {
        error("Critical leak detected for {NAME}", "NAME", name);
    }
    else
    {
        warning("Warning leak detected for {NAME}", "NAME", name);
    }
}

void resolveLeakEvent(std::string& name, Level level)
{
    if (level == Level::critical)
    {
        error("Critical leak detected for {NAME}", "NAME", name);
    }
    else
    {
        warning("Warning leak detected for {NAME}", "NAME", name);
    }
}
} // namespace phosphor::leak::utils
