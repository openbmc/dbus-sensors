#include "LeakDetectionManager.hpp"

#include "LeakGPIODetector.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <sdbusplus/server/manager.hpp>

#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <string>

PHOSPHOR_LOG2_USING;

namespace leak
{

DetectionManager::DetectionManager(sdbusplus::async::context& ctx) :
    ctx(ctx), leakEvents(ctx),
    entityManager(
        ctx, {GPIODetectorConfigIntf::interface},
        std::bind_front(&DetectionManager::processInventoryAdded, this),
        std::bind_front(&DetectionManager::processInventoryRemoved, this))
{
    ctx.spawn(entityManager.handleInventoryGet());
}

auto DetectionManager::processInventoryAdded(
    const sdbusplus::object_path& objectPath, const std::string& /*unused*/)
    -> void
{
    ctx.spawn(processConfigAddedAsync(objectPath));
}

auto DetectionManager::processInventoryRemoved(
    const sdbusplus::object_path& objectPath, const std::string& /*unused*/)
    -> void
{
    if (!detectors.contains(objectPath))
    {
        return;
    }
    debug("Removed detector {DETECTOR}", "DETECTOR", objectPath);
    detectors.erase(objectPath);
}

auto DetectionManager::processConfigAddedAsync(
    sdbusplus::object_path objectPath) -> sdbusplus::async::task<>
{
    auto res = co_await getDetectorConfig(objectPath);
    if (!res)
    {
        co_return;
    }
    auto config = res.value();

    if (detectors.contains(objectPath))
    {
        warning("Detector {DETECTOR} already exist", "DETECTOR", config.name);
        co_return;
    }

    try
    {
        detectors[objectPath] =
            std::make_unique<GPIODetector>(ctx, leakEvents, config);
    }
    catch (std::exception& e)
    {
        error("Failed to create detector {DETECTOR}: {ERROR}", "DETECTOR",
              config.name, "ERROR", e.what());
    }

    co_return;
}

auto DetectionManager::getDetectorConfig(sdbusplus::object_path objectPath)
    -> sdbusplus::async::task<std::optional<config::DetectorConfig>>
{
    config::DetectorConfig config = {};

    // TODO: fix up async client to handle object_path parameter
    const std::string pathStr = objectPath.string();

    auto properties =
        co_await GPIODetectorConfigIntf(ctx)
            .service(entity_manager::EntityManagerInterface::serviceName)
            .path(pathStr)
            .properties();

    config.name = properties.name;

    for (const auto& [key, value] : config::validDetectorTypes)
    {
        if (properties.type == key)
        {
            config.type = value;
            break;
        }
    }

    config.pinName = properties.pin_name;

    for (const auto& [key, value] : config::validPinPolarity)
    {
        if (properties.polarity == key)
        {
            config.polarity = value;
            break;
        }
    }
    if (config.polarity == config::PinPolarity::unknown)
    {
        error("Invalid polarity {POLARITY} for {NAME}", "POLARITY",
              properties.polarity, "NAME", config.name);
        co_return std::nullopt;
    }

    for (const auto& [key, value] : config::validDetectorLevel)
    {
        if (properties.level == key)
        {
            config.level = value;
            break;
        }
    }
    if (config.level == config::DetectorLevel::unknown)
    {
        error("Invalid level {LEVEL} for {NAME}", "LEVEL", properties.level,
              "NAME", config.name);
        co_return std::nullopt;
    }

    config.parentInventoryPath = objectPath.parent_path();

    debug(
        "Detector config: {NAME} {PIN_NAME} {POLARITY} {LEVEL} {INVENTORY_PATH}",
        "NAME", config.name, "PIN_NAME", config.pinName, "POLARITY",
        config.polarity, "LEVEL", config.level, "INVENTORY_PATH",
        config.parentInventoryPath);

    co_return config;
}

} // namespace leak

int main()
{
    constexpr auto path = leak::DetectorIntf::namespace_path::value;
    constexpr auto serviceName = "xyz.openbmc_project.leakdetector";
    sdbusplus::async::context ctx;
    sdbusplus::server::manager_t manager{ctx, path};

    info("Creating leak detection manager at {PATH}", "PATH", path);
    leak::DetectionManager leakDetectionManager{ctx};

    ctx.request_name(serviceName);

    ctx.run();
    return 0;
}
