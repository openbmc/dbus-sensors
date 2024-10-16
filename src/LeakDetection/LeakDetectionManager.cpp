#include "LeakDetectionManager.hpp"

#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/Configuration/GPIOLeakDetector/client.hpp>

PHOSPHOR_LOG2_USING;

namespace phosphor::leak::manager
{

using GPIODetectorConfigIntf =
    sdbusplus::client::xyz::openbmc_project::configuration::GPIOLeakDetector<>;
namespace DetectorConfigIntf = phosphor::leak::detector::config;

static const std::vector<std::string> configTypes = {"LeakDetector.Group",
                                                     "GpioLeakDetector"};

auto LeakDetectionManager::startup() -> sdbusplus::async::task<>
{
    ctx.spawn(handleConfigAdded());
    ctx.spawn(handleConfigRemoved());
    ctx.spawn(handleConfigGet());
    co_return;
}

auto LeakDetectionManager::processConfigAdded(
    const sdbusplus::message::object_path& objectPath,
    const SensorData& detectorConfig) -> void
{
    auto data = detectorConfig.find(GPIODetectorConfigIntf::interface);
    if (data == detectorConfig.end())
    {
        return;
    }

    ctx.spawn(processConfigAddedAsync(objectPath));
}

auto LeakDetectionManager::processConfigAddedAsync(
    const sdbusplus::message::object_path& objectPath)
    -> sdbusplus::async::task<>
{
    auto config = co_await DetectorConfigIntf::processConfig(ctx, objectPath);

    auto detectorName = config.name;

    if (detectors.find(detectorName) != detectors.end())
    {
        warning("Detector {DETECTOR} already exist", "DETECTOR", detectorName);
        co_return;
    }

    try
    {
        detectors[detectorName] = std::make_unique<DetectorIntf::LeakDetector>(
            ctx, leakEvents, systemd, config);
    }
    catch (std::exception& e)
    {
        error("Failed to create detector {DETECTOR}: {ERROR}", "DETECTOR",
              detectorName, "ERROR", e.what());
    }

    co_return;
}

auto LeakDetectionManager::processConfigRemoved(
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

    return;
}

} // namespace phosphor::leak::manager

int main()
{
    constexpr auto path =
        DetectorIntf::DetectorIntf::Detector::namespace_path::value;
    sdbusplus::async::context ctx;
    sdbusplus::server::manager_t manager{ctx, path};

    LeakEventsIntf leakEvents{ctx};
    SystemdIntf systemd{ctx.get_bus()};

    info("Creating leak detection manager at {PATH}", "PATH", path);
    using LeakDetectionManagerIntf =
        phosphor::leak::manager::LeakDetectionManager;
    LeakDetectionManagerIntf leakDetectionManager{ctx, leakEvents, systemd};

    ctx.request_name(serviceName);

    ctx.run();
    return 0;
}
