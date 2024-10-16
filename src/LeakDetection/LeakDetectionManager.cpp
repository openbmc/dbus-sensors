#include "LeakDetectionManager.hpp"

#include <phosphor-logging/lg2.hpp>

PHOSPHOR_LOG2_USING;

namespace phosphor::leak::manager
{

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
    const SensorData& detectorConfig) -> sdbusplus::async::task<>
{
    auto data = detectorConfig.find(DetectorConfigIntf::interface);
    if (data == detectorConfig.end())
    {
        co_return;
    }
    auto config = DetectorConfigIntf::processConfig(data->second);
    if (!config.has_value())
    {
        warning("Invalid config for {PATH}", "PATH", objectPath);
        co_return;
    }

    auto detectorName = config.value().name;

    if (detectors.find(detectorName) != detectors.end())
    {
        debug("Detector {DETECTOR} already exist", "DETECTOR", detectorName);
        co_return;
    }

    try
    {
        detectors[detectorName] = std::make_unique<DetectorIntf::LeakDetector>(
            ctx, leakEvents, systemd, config.value());
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
    const interface_list_t& interfaces) -> sdbusplus::async::task<>
{
    if (interfaces.find(DetectorConfigIntf::interface) == interfaces.end())
    {
        co_return;
    }
    auto detectorName = objectPath.filename();
    if (detectors.find(detectorName) == detectors.end())
    {
        co_return;
    }
    detectors.erase(detectorName);
    debug("Removed detector {DETECTOR}", "DETECTOR", detectorName);

    co_return;
}

} // namespace phosphor::leak::manager

using namespace phosphor::leak::manager;

int main()
{
    constexpr auto path =
        DetectorIntf::DetectorIntf::Detector::namespace_path::value;
    sdbusplus::async::context ctx;
    sdbusplus::server::manager_t manager{ctx, path};

    LeakEventsIntf leakEvents{ctx};
    SystemdIntf systemd{ctx.get_bus()};

    info("Creating leak detection manager at {PATH}", "PATH", path);
    LeakDetectionManager leakDetectionManager{ctx, leakEvents, systemd};
    ctx.request_name(serviceName);

    ctx.run();
    return 0;
}
