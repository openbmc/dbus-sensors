#include "ValveMonitor.hpp"

#include "ValveFactory.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <sdbusplus/server/manager.hpp>

#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <utility>

PHOSPHOR_LOG2_USING;

namespace valve
{

ValveMonitor::ValveMonitor(sdbusplus::async::context& ctx) :
    ctx(ctx), events(ctx), valveConfig(ctx),
    entityManager(ctx, ValveFactory::getInterfaces(),
                  std::bind_front(&ValveMonitor::processInventoryAdded, this),
                  std::bind_front(&ValveMonitor::processInventoryRemoved, this))
{
    ctx.spawn(valveConfig.start());
    ctx.spawn(entityManager.handleInventoryGet());
}

auto ValveMonitor::processInventoryAdded(
    const sdbusplus::message::object_path& objectPath,
    const std::string& interfaceName) -> void
{
    // Added NO_LINT to bypass clang-tidy warning about STDEXEC_ASSERT as clang
    // seems to be confused about context being uninitialized.
    // NOLINTNEXTLINE(clang-analyzer-core.uninitialized.Branch)
    ctx.spawn(processConfigAddedAsync(objectPath, interfaceName));
}

auto ValveMonitor::processInventoryRemoved(
    const sdbusplus::message::object_path& objectPath,
    const std::string& /*unused*/) -> void
{
    if (!valves.contains(objectPath.str))
    {
        return;
    }
    debug("Removed valve {VALVE}", "VALVE", objectPath);
    valves.erase(objectPath.str);
}

auto ValveMonitor::processConfigAddedAsync(
    sdbusplus::message::object_path objectPath, std::string interfaceName)
    -> sdbusplus::async::task<>
{
    if (valves.contains(objectPath.str))
    {
        warning("Valve at {PATH} already exists", "PATH", objectPath);
        co_return;
    }

    try
    {
        auto valve = co_await ValveFactory::createValve(
            ctx, objectPath, events, valveConfig, interfaceName);
        if (!valve)
        {
            error("Failed to create valve for {OBJECT_PATH}", "OBJECT_PATH",
                  objectPath);
            co_return;
        }
        valves[objectPath.str] = std::move(valve);
    }
    catch (std::exception& e)
    {
        error("Failed to create valve for {OBJECT_PATH}: {ERROR}",
              "OBJECT_PATH", objectPath, "ERROR", e);
    }

    co_return;
}

} // namespace valve

int main()
{
    constexpr auto path = "/xyz/openbmc_project";
    constexpr auto serviceName = "xyz.openbmc_project.valvemonitor";
    sdbusplus::async::context ctx;
    sdbusplus::server::manager_t manager{ctx, path};

    info("Creating valve monitor at {PATH}", "PATH", path);
    valve::ValveMonitor valveMonitor{ctx};

    ctx.request_name(serviceName);

    ctx.run();
    return 0;
}
