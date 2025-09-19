#include "ExternalSensor.hpp"

#include "asio/Thresholds.hpp"
#include "asio/sensor.hpp"
#include "utils/SensorPaths.hpp"
#include "utils/Utils.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

ExternalSensor::ExternalSensor(
    const std::string& objectType, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    const std::string& sensorName, const std::string& sensorUnits,
    std::vector<thresholds::Threshold>&& thresholdsIn,
    const std::string& sensorConfiguration, double maxReading,
    double minReading, double timeoutSecs, const PowerState& powerState) :
    Sensor(escapeName(sensorName), std::move(thresholdsIn), sensorConfiguration,
           objectType, true, true, maxReading, minReading, conn, powerState),
    objServer(objectServer), writeLast(std::chrono::steady_clock::now()),
    writeTimeout(
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(timeoutSecs))),
    writePerishable(timeoutSecs > 0.0)
{
    // The caller must specify what physical characteristic
    // an external sensor is expected to be measuring, such as temperature,
    // as, unlike others, this is not implied by device type name.
    std::string dbusPath = sensor_paths::getPathForUnits(sensorUnits);
    if (dbusPath.empty())
    {
        throw std::runtime_error("Units not in allow list");
    }
    std::string objectPath = "/xyz/openbmc_project/sensors/";
    objectPath += dbusPath;
    objectPath += '/';
    objectPath += name;

    sensorInterface = objectServer.add_interface(
        objectPath, "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface(objectPath, interface);
    }

    association =
        objectServer.add_interface(objectPath, association::interface);
    std::string fullUnits = sensor_paths::convertToFullUnits(sensorUnits);
    if (fullUnits.empty())
    {
        lg2::error("Invalid units for sensor: {UNITS}", "UNITS", sensorUnits);
        throw std::invalid_argument("Invalid units for sensor");
    }
    setInitialProperties(fullUnits);

    lg2::debug(
        "ExternalSensor '{NAME}' constructed: path '{PATH}', type '{TYPE}', "
        "min '{MIN}', max '{MAX}', timeout '{TIMEOUT}' us",
        "NAME", name, "PATH", objectPath, "TYPE", objectType, "MIN", minReading,
        "MAX", maxReading, "TIMEOUT",
        std::chrono::duration_cast<std::chrono::microseconds>(writeTimeout)
            .count());
}

// Separate function from constructor, because of a gotcha: can't use the
// enable_shared_from_this() API until after the constructor has completed.
void ExternalSensor::initWriteHook(
    std::function<void(std::chrono::steady_clock::time_point now)>&&
        writeHookIn)
{
    // Connect ExternalSensorMain with ExternalSensor
    writeHook = std::move(writeHookIn);

    // Connect ExternalSensor with Sensor
    auto weakThis = weak_from_this();
    externalSetHook = [weakThis]() {
        auto lockThis = weakThis.lock();
        if (lockThis)
        {
            lockThis->externalSetTrigger();
            return;
        }
        lg2::debug("ExternalSensor receive ignored, sensor gone");
    };
}

ExternalSensor::~ExternalSensor()
{
    // Make sure the write hook does not reference this object anymore
    externalSetHook = nullptr;

    objServer.remove_interface(association);
    for (const auto& iface : thresholdInterfaces)
    {
        objServer.remove_interface(iface);
    }
    objServer.remove_interface(sensorInterface);

    lg2::debug("ExternalSensor '{NAME}' destructed", "NAME", name);
}

void ExternalSensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}

bool ExternalSensor::isAliveAndPerishable() const
{
    return (writeAlive && writePerishable);
}

bool ExternalSensor::isAliveAndFresh(
    const std::chrono::steady_clock::time_point& now) const
{
    // Must be alive and perishable, to have possibility of being fresh
    if (!isAliveAndPerishable())
    {
        return false;
    }

    // If age, as of now, is less than timeout, it is deemed fresh
    // NOLINTNEXTLINE
    return (ageElapsed(now) < writeTimeout);
}

void ExternalSensor::writeBegin(
    const std::chrono::steady_clock::time_point& now)
{
    if (!writeAlive)
    {
        lg2::error(
            "ExternalSensor '{NAME}' online, receiving first value '{VALUE}'",
            "NAME", name, "VALUE", value);
    }

    writeLast = now;
    writeAlive = true;
}

void ExternalSensor::writeInvalidate()
{
    writeAlive = false;

    lg2::error("ExternalSensor '{NAME}' offline, timed out", "NAME", name);

    // Take back control of this sensor from the external override,
    // as the external source has timed out.
    // This allows sensor::updateValue() to work normally,
    // as it would do for internal sensors with values from hardware.
    overriddenState = false;

    // Invalidate the existing Value, similar to what internal sensors do,
    // when they encounter errors trying to read from hardware.
    updateValue(std::numeric_limits<double>::quiet_NaN());
}

std::chrono::steady_clock::duration ExternalSensor::ageElapsed(
    const std::chrono::steady_clock::time_point& now) const
{
    // Comparing 2 time_point will return duration
    return (now - writeLast);
}

std::chrono::steady_clock::duration ExternalSensor::ageRemaining(
    const std::chrono::steady_clock::time_point& now) const
{
    // Comparing duration will return another duration
    return (writeTimeout - ageElapsed(now));
}

void ExternalSensor::externalSetTrigger()
{
    lg2::debug("ExternalSensor '{NAME}' received '{VALUE}'", "NAME", name,
               "VALUE", value);

    if (std::isfinite(value))
    {
        markAvailable(true);
    }

    auto now = std::chrono::steady_clock::now();

    writeBegin(now);

    // Tell the owner to recalculate the expiration timer
    writeHook(now);
}
