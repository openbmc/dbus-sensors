#include "LeakDetectionManager.hpp"

#include "LeakDetector.hpp"
#include "Utils.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <sdbusplus/server/manager.hpp>
#include <xyz/openbmc_project/Configuration/GPIOLeakDetector/client.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/aserver.hpp>

#include <algorithm>
#include <exception>
#include <memory>
#include <string>
#include <vector>

PHOSPHOR_LOG2_USING;

namespace phosphor::leak::manager
{

using GPIODetectorConfigIntf =
    sdbusplus::client::xyz::openbmc_project::configuration::GPIOLeakDetector<>;

auto LeakDetectionManager::processInventoryAdded(
    const sdbusplus::message::object_path& objectPath,
    const SensorData& inventoryData) -> void
{
    auto data = inventoryData.find(GPIODetectorConfigIntf::interface);
    if (data == inventoryData.end())
    {
        return;
    }

    ctx.spawn(processConfigAddedAsync(objectPath));
}

auto LeakDetectionManager::processInventoryRemoved(
    const sdbusplus::message::object_path& objectPath,
    const interface_list_t& interfaces) -> void
{
    if (std::find(interfaces.begin(), interfaces.end(),
                  GPIODetectorConfigIntf::interface) == interfaces.end())
    {
        return;
    }
    auto detectorName = objectPath.filename();
    if (detectors.find(detectorName) == detectors.end())
    {
        return;
    }
    detectors.erase(detectorName);
    debug("Removed detector {DETECTOR}", "DETECTOR", detectorName);
}

// NOLINTNEXTLINE(readability-static-accessed-through-instance)
auto LeakDetectionManager::startup() -> sdbusplus::async::task<>
{
    ctx.spawn(handleInventoryAdded());
    ctx.spawn(handleInventoryRemoved());
    ctx.spawn(handleInventoryGet());
    co_return;
}

// NOLINTNEXTLINE(readability-static-accessed-through-instance)
auto LeakDetectionManager::processConfigAddedAsync(
    sdbusplus::message::object_path objectPath) -> sdbusplus::async::task<>
{
    auto config = co_await getDetectorConfig(objectPath);

    auto detectorName = config.name;

    if (detectors.find(detectorName) != detectors.end())
    {
        warning("Detector {DETECTOR} already exist", "DETECTOR", detectorName);
        co_return;
    }

    try
    {
        detectors[detectorName] = std::make_unique<detector::LeakDetector>(
            ctx, leakEvents, systemd, config);
    }
    catch (std::exception& e)
    {
        error("Failed to create detector {DETECTOR}: {ERROR}", "DETECTOR",
              detectorName, "ERROR", e.what());
    }

    co_return;
}

// NOLINTNEXTLINE(readability-static-accessed-through-instance)
auto LeakDetectionManager::getDetectorConfig(
    sdbusplus::message::object_path objectPath)
    -> sdbusplus::async::task<detector::config::DetectorConfig>
{
    detector::config::DetectorConfig config = {};

    constexpr auto serviceName = "xyz.openbmc_project.EntityManager";
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
    config.runTarget = co_await client.run_target();

    debug("Detector config: {NAME} {PIN_NAME} {POLARITY} {LEVEL} {RUN_TARGET}",
          "NAME", config.name, "PIN_NAME", config.pinName, "POLARITY",
          config.polarity, "LEVEL", config.level, "RUN_TARGET",
          config.runTarget);

    co_return config;
}

} // namespace phosphor::leak::manager

int main()
{
    constexpr auto path =
        phosphor::leak::detector::DetectorIntf::namespace_path::value;
    constexpr auto serviceName = "xyz.openbmc_project.LeakDetector";
    sdbusplus::async::context ctx;
    sdbusplus::server::manager_t manager{ctx, path};

    info("Creating leak detection manager at {PATH}", "PATH", path);
    using LeakDetectionManagerIntf =
        phosphor::leak::manager::LeakDetectionManager;
    LeakDetectionManagerIntf leakDetectionManager{ctx};

    ctx.request_name(serviceName);

    ctx.run();
    return 0;
}