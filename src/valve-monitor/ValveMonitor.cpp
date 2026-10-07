#include "ValveMonitor.hpp"

#include "AnalogValve.hpp"
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
    const auto& key = objectPath.str;
    if (interfaceName == AnalogConfigIntf::interface)
    {
        auto valveIt = valves.find(key);
        if (valveIt != valves.end() &&
            dynamic_cast<AnalogValve*>(valveIt->second.get()) == nullptr)
        {
            warning(
                "Ignoring AnalogValve add while GPIO valve exists at {PATH}",
                "PATH", objectPath);
            return;
        }

        auto& state = analogStates[key];
        if (!state.desiredPresent)
        {
            state.desiredPresent = true;
            ++state.generation;
        }
        scheduleAnalogCreate(key);
        return;
    }

    if (analogStates.contains(key))
    {
        warning(
            "Ignoring GPIO valve add during AnalogValve lifecycle at {PATH}",
            "PATH", objectPath);
        return;
    }

    // Added NO_LINT to bypass clang-tidy warning about STDEXEC_ASSERT as clang
    // seems to be confused about context being uninitialized.
    // NOLINTNEXTLINE(clang-analyzer-core.uninitialized.Branch)
    ctx.spawn(processGPIOConfigAddedAsync(objectPath, interfaceName));
}

auto ValveMonitor::processInventoryRemoved(
    const sdbusplus::message::object_path& objectPath,
    const std::string& interfaceName) -> void
{
    const auto& key = objectPath.str;
    auto stateIt = analogStates.find(key);
    if (stateIt != analogStates.end())
    {
        if (interfaceName != AnalogConfigIntf::interface)
        {
            return;
        }

        auto& state = stateIt->second;
        if (!state.desiredPresent)
        {
            return;
        }
        state.desiredPresent = false;
        ++state.generation;

        auto valveIt = valves.find(key);
        if (valveIt != valves.end())
        {
            auto* analog = dynamic_cast<AnalogValve*>(valveIt->second.get());
            if (analog != nullptr && !state.retiring)
            {
                state.retiring = true;
                analog->requestStop();
            }
        }
        else if (!state.creating && !state.retiring && !state.desiredPresent)
        {
            analogStates.erase(stateIt);
        }
        return;
    }

    if (interfaceName == AnalogConfigIntf::interface)
    {
        return;
    }
    if (!valves.contains(key))
    {
        return;
    }
    debug("Removed valve {VALVE}", "VALVE", objectPath);
    valves.erase(key);
}

auto ValveMonitor::scheduleAnalogCreate(const std::string& objectPath) -> void
{
    auto stateIt = analogStates.find(objectPath);
    if (ctx.stop_requested() || stateIt == analogStates.end() ||
        !stateIt->second.desiredPresent || stateIt->second.creating ||
        stateIt->second.retiring || valves.contains(objectPath))
    {
        return;
    }

    auto& state = stateIt->second;
    state.creating = true;
    const auto generation = state.generation;
    const auto interfaceName = std::string(AnalogConfigIntf::interface);
    const auto path = sdbusplus::message::object_path(objectPath);
    try
    {
        ctx.spawn(
            processAnalogConfigAddedAsync(path, interfaceName, generation));
    }
    catch (const std::exception& e)
    {
        error("Failed to schedule AnalogValve creation for {PATH}: {ERROR}",
              "PATH", objectPath, "ERROR", e);
        std::terminate();
    }
    catch (...)
    {
        error(
            "Failed to schedule AnalogValve creation for {PATH}: unknown error",
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
        for (const auto& [path, state] : analogStates)
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
            auto stateIt = analogStates.find(path);
            if (stateIt == analogStates.end() || !stateIt->second.retiring)
            {
                continue;
            }

            auto valveIt = valves.find(path);
            if (valveIt != valves.end())
            {
                auto* analog =
                    dynamic_cast<AnalogValve*>(valveIt->second.get());
                if (analog == nullptr || !analog->isStopped())
                {
                    continue;
                }
                // Remove D-Bus registrations before any replacement is made.
                valves.erase(valveIt);
            }

            stateIt = analogStates.find(path);
            if (stateIt == analogStates.end())
            {
                continue;
            }
            stateIt->second.retiring = false;
            if (stateIt->second.desiredPresent && !ctx.stop_requested())
            {
                scheduleAnalogCreate(path);
            }
            else if (!stateIt->second.creating)
            {
                analogStates.erase(stateIt);
            }
        }
    }
    co_return;
}

auto ValveMonitor::processGPIOConfigAddedAsync(
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

auto ValveMonitor::processAnalogConfigAddedAsync(
    sdbusplus::message::object_path objectPath, std::string interfaceName,
    std::uint64_t generation) -> sdbusplus::async::task<>
{
    const auto key = objectPath.str;
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

    if (ctx.stop_requested())
    {
        valve.reset();
        co_return;
    }

    auto stateIt = analogStates.find(key);
    if (stateIt == analogStates.end())
    {
        valve.reset();
        co_return;
    }

    auto& state = stateIt->second;
    const bool current = state.desiredPresent &&
                         state.generation == generation &&
                         interfaceName == AnalogConfigIntf::interface;
    if (!current || creationFailed || !valve)
    {
        const bool missingValve = !valve;
        // Factory results are dormant AnalogValves and can safely be discarded.
        valve.reset();
        state.creating = false;
        if (!creationFailed && missingValve && current)
        {
            error("Failed to create valve for {OBJECT_PATH}", "OBJECT_PATH",
                  objectPath);
        }
        if (!current && state.desiredPresent)
        {
            scheduleAnalogCreate(key);
        }
        else if (!state.desiredPresent && !state.retiring)
        {
            analogStates.erase(stateIt);
        }
        co_return;
    }

    auto* analog = dynamic_cast<AnalogValve*>(valve.get());
    if (analog == nullptr || valves.contains(key))
    {
        valve.reset();
        state.creating = false;
        error("Unexpected existing or non-analog valve at {PATH}", "PATH",
              objectPath);
        co_return;
    }

    valves.emplace(key, std::move(valve));
    state.creating = false;
    analog->startMonitoring();

    co_return;
}

} // namespace valve

int main()
{
    constexpr auto serviceName = "xyz.openbmc_project.valvemonitor";
    sdbusplus::async::context ctx;
    sdbusplus::server::manager_t sensorManager{ctx,
                                               "/xyz/openbmc_project/sensors"};
    sdbusplus::server::manager_t controlManager{ctx,
                                                "/xyz/openbmc_project/control"};

    valve::ValveMonitor valveMonitor{ctx};

    ctx.request_name(serviceName);

    ctx.run();
    return 0;
}
