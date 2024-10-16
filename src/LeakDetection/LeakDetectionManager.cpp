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
        detectors[detectorName] =
            std::make_unique<DetectorIntf::LeakDetector>(ctx, config.value());
    }
    catch (std::exception& e)
    {
        error("Failed to create detector {DETECTOR}: {ERROR}", "DETECTOR",
              detectorName, "ERROR", e.what());
    }

    co_return;
}

/*

auto LeakDetectionManager::handleConfigGet() -> sdbusplus::async::task<>
{
    constexpr auto entityManager =
        sdbusplus::async::proxy()
            .service(EntityManagerServiceName)
            .path("/xyz/openbmc_project/inventory")
            .interface("org.freedesktop.DBus.ObjectManager");

    for (const auto& [objectPath, detectorConfig] :
         co_await entityManager.call<ManagedObjectType>(ctx,
                                                        "GetManagedObjects"))
    {
        co_await processConfigAdded(objectPath, detectorConfig);
    }

    co_return;
}

auto LeakDetectionManager::handleConfigAdded() -> sdbusplus::async::task<>
{
    while (!ctx.stop_requested())
    {
        auto [objectPath, detectorConfig] =
            co_await configAddedMatch
                ->next<sdbusplus::message::object_path, SensorData>();
        co_await processConfigAdded(objectPath, detectorConfig);
    }

    co_return;
}
*/

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

/*
auto LeakDetectionManager::handleConfigRemoved() -> sdbusplus::async::task<>
{
    while (!ctx.stop_requested())
    {
        auto [objectPath, interfaces] =
            co_await configRemovedMatch
                ->next<sdbusplus::message::object_path, interface_list_t>();
        co_await processConfigRemoved(objectPath, interfaces);
    }

    co_return;
}
*/

} // namespace phosphor::leak::manager

using namespace phosphor::leak::manager;

int main()
{
    constexpr auto path =
        DetectorIntf::DetectorIntf::Detector::namespace_path::value;
    sdbusplus::async::context ctx;
    sdbusplus::server::manager_t manager{ctx, path};

    info("Creating leak detection manager at {PATH}", "PATH", path);
    LeakDetectionManager leakDetectionManager{ctx};
    ctx.request_name(serviceName);

    ctx.run();
    return 0;
}
