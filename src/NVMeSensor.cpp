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

#include "NVMeSensor.hpp"

#include <iostream>

static constexpr double maxReading = 127;
static constexpr double minReading = 0;

NVMeSensor::NVMeSensor(sdbusplus::asio::object_server& objectServer,
                       boost::asio::io_context& /*unused*/,
                       std::shared_ptr<sdbusplus::asio::connection>& conn,
                       const std::string& sensorName,
                       std::vector<thresholds::Threshold>&& thresholdsIn,
                       const std::string& sensorConfiguration,
                       const int busNumber, const uint8_t slaveAddr) :
    Sensor(escapeName(sensorName), std::move(thresholdsIn), sensorConfiguration,
           NVMeSensor::sensorType, false, false, maxReading, minReading, conn,
           PowerState::on),
    bus(busNumber), address(slaveAddr), objServer(objectServer)
{
    if (bus < 0)
    {
        throw std::invalid_argument("Invalid bus: Bus ID must not be negative");
    }

    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface(
                "/xyz/openbmc_project/sensors/temperature/" + name, interface);
    }
    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        association::interface);

    setInitialProperties(sensor_paths::unitDegreesC);
}

NVMeSensor::~NVMeSensor()
{
    // close the input dev to cancel async operations
    for (const auto& iface : thresholdInterfaces)
    {
        objServer.remove_interface(iface);
    }
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

bool NVMeSensor::sample()
{
    if (inError())
    {
        if (scanDelay == 0)
        {
            scanDelay = scanDelayTicks;
        }

        scanDelay--;
    }

    return scanDelay == 0;
}

void NVMeSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}
