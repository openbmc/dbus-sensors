#include "ExternalSensor.hpp"

#include "SensorPaths.hpp"

#include <unistd.h>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <iostream>
#include <istream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

ExternalSensor::ExternalSensor(
    const std::string& objectType, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    const std::string& sensorName, const std::string& sensorUnits,
    std::vector<thresholds::Threshold>&& thresholds,
    const std::string& sensorConfiguration, const double& maxReading,
    const double& minReading, const PowerState& powerState) :
    // TODO(): When the Mutable feature is integrated,
    // make sure all ExternalSensor instances are mutable,
    // because that is the entire point of ExternalSensor,
    // to accept sensor values written by an external source.
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(thresholds), sensorConfiguration, objectType, maxReading,
           minReading, conn, powerState),
    std::enable_shared_from_this<ExternalSensor>(), objServer(objectServer)
{
    // The caller must specify what physical characteristic
    // an external sensor is expected to be measuring, such as temperature,
    // as, unlike others, this is not implied by device type name.
    std::string dbusPath = sensors::getPathForUnits(sensorUnits);
    if (dbusPath.empty())
    {
        throw std::runtime_error("Units not in allow list");
    }
    std::string objectPath = sensors::objectPathPrefix;
    objectPath += dbusPath;
    objectPath += '/';
    objectPath += sensorName;

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
}

ExternalSensor::~ExternalSensor()
{
    objServer.remove_interface(association);
    objServer.remove_interface(thresholdInterfaceCritical);
    objServer.remove_interface(thresholdInterfaceWarning);
    objServer.remove_interface(sensorInterface);
}

void ExternalSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}
