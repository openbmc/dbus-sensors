#include "LeakDetectionManager.hpp"

#include <phosphor-logging/lg2.hpp>

PHOSPHOR_LOG2_USING;

namespace phosphor::leak::manager
{
namespace RulesIntf = sdbusplus::bus::match::rules;

static const std::vector<std::string> configTypes = {"LeakDetector.Group",
                                                     "GpioLeakDetector"};
const std::string GPIOConfigInterface =
    "xyz.openbmc_project.Configuration.GpioLiquidLeakDetector";
const std::string inventoryPath = "/xyz/openbmc_project/inventory/";
const std::string EntityManagerServiceName =
    "xyz.openbmc_project.EntityManager";

auto LeakDetectionManager::startup() -> sdbusplus::async::task<>
{
    info("Starting leak detection manager");

    ctx.spawn(handleConfigAdded());
    ctx.spawn(handleConfigRemoved());
    ctx.spawn(handleConfigGet());

    co_return;
}

auto LeakDetectionManager::processConfig(
    const sdbusplus::message::object_path& objectPath,
    const SensorData& detectorConfig) -> sdbusplus::async::task<>
{
    auto data = detectorConfig.find(LeakDetectorConfigIntf::interface);
    if (data == detectorConfig.end())
    {
        warning("Config doesn't contain interface {INTERFACE}", "INTERFACE",
                LeakDetectorConfigIntf::interface);
        co_return;
    }
    auto config = LeakDetectorConfigIntf::processConfig(data->second);
    if (!config.has_value())
    {
        warning("Invalid config for {PATH}", "PATH", objectPath);
        co_return;
    }

    auto detectorName = config.value().name;

    if (detectors.find(detectorName) != detectors.end())
    {
        warning("Detector {DETECTOR} already exist", "DETECTOR", detectorName);
        co_return;
    }

    detectors[detectorName] =
        std::make_unique<LeakDetectorIntf::LeakDetector>(ctx, config.value());

    // ctx.spawn(detectors[detectorName]->setupAsyncEvent());

    co_return;
}

auto LeakDetectionManager::handleConfigGet() -> sdbusplus::async::task<>
{
    info("Config Get for leak detectors");
    constexpr auto entityManager =
        sdbusplus::async::proxy()
            .service("xyz.openbmc_project.EntityManager")
            .path("/xyz/openbmc_project/inventory")
            .interface("org.freedesktop.DBus.ObjectManager");

    for (const auto& [objectPath, detectorConfig] :
         co_await entityManager.call<ManagedObjectType>(ctx,
                                                        "GetManagedObjects"))
    {
        // sdbusplus::message::object_path objectPath = managedObject.first;
        info("JAG: handleConfigGet: Config Get {PATH}", "PATH", objectPath);
        co_await processConfig(objectPath, detectorConfig);
    }

    co_return;
}

auto LeakDetectionManager::handleConfigAdded() -> sdbusplus::async::task<>
{
    info("Config added for leak detectors");
    auto match = sdbusplus::async::match(
        ctx, RulesIntf::interfacesAdded() +
                 RulesIntf::sender(EntityManagerServiceName));
    auto [objectPath, detectorConfig] =
        co_await match.next<sdbusplus::message::object_path, SensorData>();
    info("handleConfigAdded: Config added {PATH}", "PATH", objectPath);
    co_await processConfig(objectPath, detectorConfig);

    co_return ctx.spawn(handleConfigAdded());
}

auto LeakDetectionManager::handleConfigRemoved() -> sdbusplus::async::task<>
{
    info("Config removed for leak detectors");
    auto match = sdbusplus::async::match(
        ctx, RulesIntf::interfacesRemoved() +
                 RulesIntf::sender(EntityManagerServiceName));
    auto [objectPath, ifcAndProperties] =
        co_await match.next<sdbusplus::message::object_path, SensorData>();
    info("handleConfigRemoved: Config removed {PATH}", "PATH", objectPath);

    co_return ctx.spawn(handleConfigRemoved());
}
} // namespace phosphor::leak::manager

using namespace phosphor::leak::manager;

int main()
{
    constexpr auto path =
        LeakDetectorIntf::LeakDetectorIntf::Detector::namespace_path::value;
    sdbusplus::async::context ctx;
    sdbusplus::server::manager_t manager{ctx, path};

    info("Creating leak detection manager at {PATH}", "PATH", path);
    LeakDetectionManager leakDetectionManager{ctx};
    ctx.request_name(serviceName);

    ctx.run();
    return 0;
}
