#include "LeakDetectionManager.hpp"

#include "LeakGPIODetector.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <sdbusplus/server/manager.hpp>

#include <exception>
#include <memory>
#include <string>

PHOSPHOR_LOG2_USING;

namespace leak
{

void DetectionManager::processInventoryAdded(
    const sdbusplus::message::object_path& objectPath,
    const std::string& /*unused*/)
{
    ctx.spawn(processConfigAddedAsync(objectPath));
}

void DetectionManager::processInventoryRemoved(
    const sdbusplus::message::object_path& objectPath,
    const std::string& /*unused*/)
{
    if (detectors.find(objectPath.str) == detectors.end())
    {
        return;
    }
    detectors.erase(objectPath.str);
    debug("Removed detector {DETECTOR}", "DETECTOR", objectPath.filename());
}

sdbusplus::async::task<> DetectionManager::startup()
{
    ctx.spawn(handleInventoryAdded());
    ctx.spawn(handleInventoryRemoved());
    ctx.spawn(handleInventoryGet());
    co_return;
}

sdbusplus::async::task<> DetectionManager::processConfigAddedAsync(
    sdbusplus::message::object_path objectPath)
{
    auto res = co_await getDetectorConfig(objectPath);
    if (!res)
    {
        co_return;
    }
    auto config = res.value();

    if (detectors.find(objectPath.str) != detectors.end())
    {
        warning("Detector {DETECTOR} already exist", "DETECTOR", config.name);
        co_return;
    }

    try
    {
        detectors[objectPath.str] =
            std::make_unique<GPIODetector>(ctx, leakEvents, config);
    }
    catch (std::exception& e)
    {
        error("Failed to create detector {DETECTOR}: {ERROR}", "DETECTOR",
              config.name, "ERROR", e.what());
    }

    co_return;
}

sdbusplus::async::task<std::optional<config::DetectorConfig>>
    DetectionManager::getDetectorConfig(
        sdbusplus::message::object_path objectPath)
{
    config::DetectorConfig config = {};

    auto client =
        GPIODetectorConfigIntf(ctx).service(serviceName).path(objectPath.str);

    config.name = co_await client.name();

    auto type = co_await client.sub_type();
    for (auto& [key, value] : config::validDetectorTypes)
    {
        if (type == key)
        {
            config.type = value;
            break;
        }
    }

    config.pinName = co_await client.pin_name();

    auto polarity = co_await client.polarity();
    for (auto& [key, value] : config::validPinPolarity)
    {
        if (polarity == key)
        {
            config.polarity = value;
            break;
        }
    }
    if (config.polarity == config::PinPolarity::unknown)
    {
        error("Invalid polarity {POLARITY} for {NAME}", "POLARITY", polarity,
              "NAME", config.name);
        co_return std::nullopt;
    }

    auto level = co_await client.level();
    for (auto& [key, value] : config::validDetectorLevel)
    {
        if (level == key)
        {
            config.level = value;
            break;
        }
    }
    if (config.level == config::DetectorLevel::unknown)
    {
        error("Invalid level {LEVEL} for {NAME}", "LEVEL", level, "NAME",
              config.name);
        co_return std::nullopt;
    }

    debug("Detector config: {NAME} {PIN_NAME} {POLARITY} {LEVEL}", "NAME",
          config.name, "PIN_NAME", config.pinName, "POLARITY", config.polarity,
          "LEVEL", config.level);

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
