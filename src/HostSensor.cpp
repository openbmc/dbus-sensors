/*
// Copyright (c) 2020 Ed Tanous
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

#include "HostSensor.hpp"

#include <unistd.h>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

static constexpr double maxReading = 100;
static constexpr double minReading = 0;

HostSensor::HostSensor(const std::string& path,
                       sdbusplus::asio::object_server& objectServer,
                       std::shared_ptr<sdbusplus::asio::connection>& conn,
                       boost::asio::io_service& ,
                       const std::string& sensorName,
                       std::vector<thresholds::Threshold>&& _thresholds,
                       const std::string& sensorConfiguration) :

    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(_thresholds), sensorConfiguration,
           "xyz.openbmc_project.Configuration.HostSensor", maxReading,
           minReading),
    objServer(objectServer), path(path)
{
    std::string sensorType = "temperature";
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/" + sensorType + "/" + name,
        "xyz.openbmc_project.Value");
    association = objectServer.add_interface("/xyz/openbmc_project/sensors/" +
                                                 sensorType + "/" + name,
                                             association::interface);
    setInitialProperties(conn);
}

HostSensor::~HostSensor()
{
    objServer.remove_interface(thresholdInterfaceWarning);
    objServer.remove_interface(thresholdInterfaceCritical);
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

void HostSensor::checkThresholds(void)
{
    // Does this need to support threadholds?  I'm not sure how this would be
    // different than normal thresholding?
}
