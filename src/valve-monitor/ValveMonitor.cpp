#include "ValveMonitor.hpp"

#include "GPIOValve.hpp"

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
    ctx(ctx), events(ctx),
    entityManager(ctx, {ValveConfigIntf::interface},
                  std::bind_front(&ValveMonitor::processInventoryAdded, this),
                  std::bind_front(&ValveMonitor::processInventoryRemoved, this))
{
    ctx.spawn(entityManager.handleInventoryGet());
}

auto ValveMonitor::processInventoryAdded(
    const sdbusplus::message::object_path& objectPath,
    const std::string& /*unused*/) -> void
{
    // Added NO_LINT to bypass clang-tidy warning about STDEXEC_ASSERT as clang
    // seems to be confused about context being uninitialized.
    // NOLINTNEXTLINE(clang-analyzer-core.uninitialized.Branch)
    ctx.spawn(processConfigAddedAsync(objectPath));
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
    sdbusplus::message::object_path objectPath) -> sdbusplus::async::task<>
{
    auto res = co_await config::getConfig(ctx, objectPath);
    if (!res.has_value())
    {
        error("Failed to get config for {OBJECT_PATH}", "OBJECT_PATH",
              objectPath);
        co_return;
    }
    auto config = res.value();

    if (valves.contains(objectPath.str))
    {
        warning("Valve {VALVE} already exist", "VALVE", config.name);
        co_return;
    }

    try
    {
        auto valve =
            std::make_unique<GPIOValve>(ctx, objectPath, events, config);
        co_await valve->createAssociations();
        valves[objectPath.str] = std::move(valve);
    }
    catch (std::exception& e)
    {
        error("Failed to create valve {VALVE}: {ERROR}", "VALVE", config.name,
              "ERROR", e);
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
