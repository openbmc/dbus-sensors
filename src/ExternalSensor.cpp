#include "ExternalSensor.hpp"

#include "SensorPaths.hpp"

#include <unistd.h>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <iostream>
#include <istream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

static constexpr bool debug = false;

static constexpr const char* objectPathSuperPrefix =
    "/xyz/openbmc_project/sensors";

static std::string buildObjectPathPrefix(const std::string& sensorUnits)
{
    // The caller must specify what physical characteristic
    // an external sensor is expected to be measuring, such as temperature,
    // as, unlike others, this is not implied by device type name.
    std::string dbusPath = sensor_paths::getPathForUnits(sensorUnits);
    if (dbusPath.empty())
    {
        throw std::runtime_error("Units not in allow list");
    }

    std::string objectPathPrefix(objectPathSuperPrefix);
    objectPathPrefix += '/';
    objectPathPrefix += dbusPath;

    return objectPathPrefix;
}

ExternalSensor::ExternalSensor(
    const std::string& objectType, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    const std::string& sensorName, const std::string& sensorUnits,
    std::vector<thresholds::Threshold>&& thresholdsIn,
    const std::string& sensorConfiguration, double maxReading,
    double minReading, double timeoutSecs, const PowerState& powerState) :
    // TODO(): When the Mutable feature is integrated,
    // make sure all ExternalSensor instances are mutable,
    // because that is the entire point of ExternalSensor,
    // to accept sensor values written by an external source.
    Sensor(buildObjectPathPrefix(sensorUnits), sensorName,
           std::move(thresholdsIn), sensorConfiguration, objectType, maxReading,
           minReading, conn, powerState),
    std::enable_shared_from_this<ExternalSensor>(), objServer(objectServer),
    writeLast(std::chrono::steady_clock::now()),
    writeTimeout(
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(timeoutSecs))),
    writeAlive(false), writePerishable(timeoutSecs > 0.0)
{
    sensorInterface = objectServer.add_interface(
        objectPath, "xyz.openbmc_project.Sensor.Value");

    if (thresholds::hasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            objectPath, "xyz.openbmc_project.Sensor.Threshold.Warning");
    }
    if (thresholds::hasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            objectPath, "xyz.openbmc_project.Sensor.Threshold.Critical");
    }

    association =
        objectServer.add_interface(objectPath, association::interface);
    setInitialProperties(conn);

    if constexpr (debug)
    {
        std::cerr << "ExternalSensor " << name << " constructed: path "
                  << configurationPath << ", type " << objectType << ", min "
                  << minReading << ", max " << maxReading << ", timeout "
                  << std::chrono::duration_cast<std::chrono::microseconds>(
                         writeTimeout)
                         .count()
                  << " us\n";
    }
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
    externalSetHook = std::move([weakThis]() {
        auto lockThis = weakThis.lock();
        if (lockThis)
        {
            lockThis->externalSetTrigger();
            return;
        }
        if constexpr (debug)
        {
            std::cerr << "ExternalSensor receive ignored, sensor gone\n";
        }
    });
}

ExternalSensor::~ExternalSensor()
{
    // Make sure the write hook does not reference this object anymore
    externalSetHook = nullptr;

    objServer.remove_interface(association);
    objServer.remove_interface(thresholdInterfaceCritical);
    objServer.remove_interface(thresholdInterfaceWarning);
    objServer.remove_interface(sensorInterface);

    if constexpr (debug)
    {
        std::cerr << "ExternalSensor " << name << " destructed\n";
    }
}

void ExternalSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}

bool ExternalSensor::isAliveAndPerishable(void) const
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
    return (ageElapsed(now) < writeTimeout);
}

void ExternalSensor::writeBegin(
    const std::chrono::steady_clock::time_point& now)
{
    if (!writeAlive)
    {
        std::cerr << "ExternalSensor " << name
                  << " online, receiving first value " << value << "\n";
    }

    writeLast = now;
    writeAlive = true;
}

void ExternalSensor::writeInvalidate(void)
{
    writeAlive = false;

    std::cerr << "ExternalSensor " << name << " offline, timed out\n";

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

void ExternalSensor::externalSetTrigger(void)
{
    if constexpr (debug)
    {
        std::cerr << "ExternalSensor " << name << " received " << value << "\n";
    }

    auto now = std::chrono::steady_clock::now();

    writeBegin(now);

    // Tell the owner to recalculate the expiration timer
    writeHook(now);
}
