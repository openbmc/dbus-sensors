/*
// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include <NVMeSensor.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <iostream>

static constexpr double maxReading = 127;
static constexpr double minReading = 0;

NVMeSensor::NVMeSensor(sdbusplus::asio::object_server& objectServer,
                       boost::asio::io_service&,
                       std::shared_ptr<sdbusplus::asio::connection>& conn,
                       const std::string& sensorName,
                       std::vector<thresholds::Threshold>&& thresholdsIn,
                       const std::string& sensorConfiguration,
                       const int busNumber) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(thresholdsIn), sensorConfiguration,
           "xyz.openbmc_project.Configuration.NVMe", maxReading, minReading,
           conn, PowerState::on),
    bus(busNumber), objServer(objectServer)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        "xyz.openbmc_project.Sensor.Value");

    if (thresholds::hasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/temperature/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Warning");
    }
    if (thresholds::hasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/temperature/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Critical");
    }
    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        association::interface);

    setInitialProperties(conn, sensor_paths::unitDegreesC);
}

NVMeSensor::~NVMeSensor()
{
    // close the input dev to cancel async operations
    objServer.remove_interface(thresholdInterfaceWarning);
    objServer.remove_interface(thresholdInterfaceCritical);
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

void NVMeSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}
