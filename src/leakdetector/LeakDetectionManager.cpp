#include "LeakDetectionManager.hpp"

#include "LeakDetector.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <sdbusplus/server/manager.hpp>

#include <exception>
#include <memory>
#include <string>

PHOSPHOR_LOG2_USING;

namespace leak::manager
{

void LeakDetectionManager::processInventoryAdded(
    const sdbusplus::message::object_path& objectPath)
{
    ctx.spawn(processConfigAddedAsync(objectPath));
}

void LeakDetectionManager::processInventoryRemoved(
    const sdbusplus::message::object_path& objectPath)
{
    if (detectors.find(objectPath.str) == detectors.end())
    {
        return;
    }
    detectors.erase(objectPath.str);
    debug("Removed detector {DETECTOR}", "DETECTOR", objectPath.filename());
}

// NOLINTNEXTLINE(readability-static-accessed-through-instance)
sdbusplus::async::task<> LeakDetectionManager::startup()
{
    ctx.spawn(handleInventoryAdded());
    ctx.spawn(handleInventoryRemoved());
    ctx.spawn(handleInventoryGet());
    co_return;
}

// NOLINTNEXTLINE(readability-static-accessed-through-instance)
sdbusplus::async::task<> LeakDetectionManager::processConfigAddedAsync(
    sdbusplus::message::object_path objectPath)
{
    auto config = co_await getDetectorConfig(objectPath);

    if (detectors.find(objectPath.str) != detectors.end())
    {
        warning("Detector {DETECTOR} already exist", "DETECTOR", config.name);
        co_return;
    }

    try
    {
        detectors[objectPath.str] = std::make_unique<detector::LeakDetector>(
            ctx, leakEvents, systemd, config);
    }
    catch (std::exception& e)
    {
        error("Failed to create detector {DETECTOR}: {ERROR}", "DETECTOR",
              config.name, "ERROR", e.what());
    }

    co_return;
}

// NOLINTBEGIN(readability-static-accessed-through-instance)
sdbusplus::async::task<detector::config::DetectorConfig>
    LeakDetectionManager::getDetectorConfig(
        sdbusplus::message::object_path objectPath)
// NOLINTEND(readability-static-accessed-through-instance)
{
    detector::config::DetectorConfig config = {};

    auto client =
        GPIODetectorConfigIntf(ctx).service(serviceName).path(objectPath.str);

    config.name = co_await client.name();
    config.type =
        detector::config::validDetectorTypes.find(co_await client.sub_type())
            ->second;
    config.pinName = co_await client.pin_name();
    config.polarity =
        detector::config::validPinPolarity.find(co_await client.polarity())
            ->second;
    config.level =
        detector::config::validDetectorLevel.find(co_await client.level())
            ->second;

    debug("Detector config: {NAME} {PIN_NAME} {POLARITY} {LEVEL}", "NAME",
          config.name, "PIN_NAME", config.pinName, "POLARITY", config.polarity,
          "LEVEL", config.level);

    co_return config;
}

} // namespace leak::manager

int main()
{
    constexpr auto path = leak::detector::DetectorIntf::namespace_path::value;
    constexpr auto serviceName = "xyz.openbmc_project.LeakDetector";
    sdbusplus::async::context ctx;
    sdbusplus::server::manager_t manager{ctx, path};

    info("Creating leak detection manager at {PATH}", "PATH", path);
    using LeakDetectionManagerIntf = leak::manager::LeakDetectionManager;
    LeakDetectionManagerIntf leakDetectionManager{ctx};

    ctx.request_name(serviceName);

    ctx.run();
    return 0;
}
