#include "ValveMonitor.hpp"

#include "ValveFactory.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <sdbusplus/server/manager.hpp>

#include <chrono>
#include <exception>
#include <functional>
#include <string>
#include <utility>
#include <vector>

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
    ctx.spawn(cleanupRetiringAsync());
}

auto ValveMonitor::processInventoryAdded(
    const sdbusplus::message::object_path& objectPath,
    const std::string& interfaceName) -> void
{
    if (ctx.stop_requested())
    {
        return;
    }

    const auto& key = objectPath.str;
    auto& state = valveStates[key];
    if (state.desiredPresent)
    {
        if (state.desiredInterface != interfaceName)
        {
            warning("Ignoring conflicting valve interface {INTERFACE} at "
                    "{PATH}; {DESIRED} remains desired",
                    "INTERFACE", interfaceName, "PATH", objectPath, "DESIRED",
                    state.desiredInterface);
        }
        return;
    }

    state.desiredInterface = interfaceName;
    state.desiredPresent = true;
    ++state.generation;
    info("Valve interface {INTERFACE} added to inventory at {PATH}",
         "INTERFACE", interfaceName, "PATH", objectPath);
    scheduleCreate(key);
}

auto ValveMonitor::processInventoryRemoved(
    const sdbusplus::message::object_path& objectPath,
    const std::string& interfaceName) -> void
{
    const auto& key = objectPath.str;
    auto stateIt = valveStates.find(key);
    if (stateIt == valveStates.end())
    {
        return;
    }

    auto& state = stateIt->second;
    if (!state.desiredPresent || state.desiredInterface != interfaceName)
    {
        return;
    }

    state.desiredPresent = false;
    ++state.generation;
    info("Valve interface {INTERFACE} removed from inventory at {PATH}",
         "INTERFACE", interfaceName, "PATH", objectPath);

    auto valveIt = valves.find(key);
    if (valveIt != valves.end() && state.activeInterface == interfaceName &&
        !state.retiring)
    {
        state.retiring = true;
        info("Requesting stop for valve interface {INTERFACE} at {PATH}",
             "INTERFACE", interfaceName, "PATH", objectPath);
        try
        {
            valveIt->second->requestStop();
        }
        catch (const std::exception& e)
        {
            error("Failed to request valve stop at {PATH}: {ERROR}", "PATH",
                  objectPath, "ERROR", e);
        }
        catch (...)
        {
            error("Failed to request valve stop at {PATH}: unknown error",
                  "PATH", objectPath);
        }
        return;
    }

    if (!state.creating && !state.retiring && valveIt == valves.end())
    {
        valveStates.erase(stateIt);
    }
}

auto ValveMonitor::scheduleCreate(const std::string& objectPath) -> void
{
    auto stateIt = valveStates.find(objectPath);
    if (ctx.stop_requested() || stateIt == valveStates.end() ||
        !stateIt->second.desiredPresent || stateIt->second.creating ||
        stateIt->second.retiring || valves.contains(objectPath))
    {
        return;
    }

    auto& state = stateIt->second;
    state.creating = true;
    const auto generation = state.generation;
    const auto interfaceName = state.desiredInterface;
    info("Scheduling valve interface {INTERFACE} at {PATH}", "INTERFACE",
         interfaceName, "PATH", objectPath);
    const auto path = sdbusplus::message::object_path(objectPath);
    try
    {
        ctx.spawn(processConfigAddedAsync(path, interfaceName, generation));
    }
    catch (const std::exception& e)
    {
        error("Failed to schedule valve creation for {PATH}: {ERROR}", "PATH",
              objectPath, "ERROR", e);
        std::terminate();
    }
    catch (...)
    {
        error("Failed to schedule valve creation for {PATH}: unknown error",
              "PATH", objectPath);
        std::terminate();
    }
}

auto ValveMonitor::cleanupRetiringAsync() -> sdbusplus::async::task<>
{
    while (!ctx.stop_requested())
    {
        co_await sdbusplus::async::sleep_for(ctx, std::chrono::seconds(1));
        if (ctx.stop_requested())
        {
            co_return;
        }

        std::vector<std::string> paths;
        for (const auto& [path, state] : valveStates)
        {
            if (state.retiring)
            {
                paths.push_back(path);
            }
        }

        for (const auto& path : paths)
        {
            if (ctx.stop_requested())
            {
                co_return;
            }
            auto stateIt = valveStates.find(path);
            if (stateIt == valveStates.end() || !stateIt->second.retiring)
            {
                continue;
            }

            auto valveIt = valves.find(path);
            if (valveIt != valves.end())
            {
                if (!valveIt->second->isStopped())
                {
                    continue;
                }
                // Remove D-Bus registrations before any replacement is made.
                valves.erase(valveIt);
                info("Retired stopped valve at {PATH}", "PATH", path);
            }

            stateIt = valveStates.find(path);
            if (stateIt == valveStates.end())
            {
                continue;
            }
            stateIt->second.retiring = false;
            stateIt->second.activeInterface.clear();
            if (stateIt->second.desiredPresent && !ctx.stop_requested())
            {
                scheduleCreate(path);
            }
            else if (!stateIt->second.creating)
            {
                valveStates.erase(stateIt);
            }
        }
    }
    co_return;
}

auto ValveMonitor::processConfigAddedAsync(
    sdbusplus::message::object_path objectPath, std::string interfaceName,
    std::uint64_t generation) -> sdbusplus::async::task<>
{
    const auto key = objectPath.str;
    if (ctx.stop_requested())
    {
        auto stateIt = valveStates.find(key);
        if (stateIt != valveStates.end())
        {
            stateIt->second.creating = false;
        }
        co_return;
    }

    std::unique_ptr<BaseValve> valve;
    bool creationFailed = false;
    try
    {
        valve = co_await ValveFactory::createValve(ctx, objectPath, events,
                                                   valveConfig, interfaceName);
    }
    catch (const std::exception& e)
    {
        creationFailed = true;
        error("Failed to create valve for {OBJECT_PATH}: {ERROR}",
              "OBJECT_PATH", objectPath, "ERROR", e);
    }
    catch (...)
    {
        creationFailed = true;
        error("Failed to create valve for {OBJECT_PATH}: unknown error",
              "OBJECT_PATH", objectPath);
    }

    // Keep the per-path creation reservation until the factory has completely
    // unwound and any dormant result has been destroyed.
    auto stateIt = valveStates.find(key);
    if (ctx.stop_requested())
    {
        valve.reset();
        if (stateIt != valveStates.end())
        {
            stateIt->second.creating = false;
        }
        co_return;
    }

    if (stateIt == valveStates.end())
    {
        valve.reset();
        co_return;
    }

    auto& state = stateIt->second;
    const bool current = state.desiredPresent &&
                         state.generation == generation &&
                         state.desiredInterface == interfaceName;
    if (!current || creationFailed || !valve)
    {
        const bool missingValve = !valve;
        valve.reset();
        state.creating = false;
        if (!creationFailed && missingValve && current)
        {
            error("Failed to create valve for {OBJECT_PATH}", "OBJECT_PATH",
                  objectPath);
        }
        if (state.desiredPresent && !current)
        {
            scheduleCreate(key);
        }
        else if (!state.desiredPresent && !state.retiring &&
                 !valves.contains(key))
        {
            valveStates.erase(stateIt);
        }
        co_return;
    }

    if (state.retiring || valves.contains(key))
    {
        valve.reset();
        state.creating = false;
        error("Refusing to replace an active valve at {PATH}", "PATH",
              objectPath);
        co_return;
    }

    auto [valveIt, inserted] = valves.emplace(key, std::move(valve));
    if (!inserted)
    {
        state.creating = false;
        error("Refusing to replace an active valve at {PATH}", "PATH",
              objectPath);
        co_return;
    }

    state.activeInterface = interfaceName;
    state.creating = false;
    valveIt->second->startMonitoring();

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
