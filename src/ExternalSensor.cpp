#include "ExternalSensor.hpp"

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
    const std::string& sensorName, const std::string& sensorPrefix,
    std::vector<thresholds::Threshold>&& _thresholds,
    const std::string& sensorConfiguration, const double& maxReading,
    const double& minReading, const PowerState& powerState) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(_thresholds), sensorConfiguration, objectType, maxReading,
           minReading, powerState),
    std::enable_shared_from_this<ExternalSensor>(), objServer(objectServer)
{
    // This matches the syntax expected by phosphor-pid-control
    // Optionally insert another path component before the sensor name
    std::string objectPath = "/xyz/openbmc_project/extsensors/";
    if (!(sensorPrefix.empty()))
    {
        objectPath += sensorPrefix;
        objectPath += '/';
    }
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
