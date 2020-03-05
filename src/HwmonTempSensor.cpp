/*
// Copyright (c) 2017 Intel Corporation
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

#include "HwmonTempSensor.hpp"

#include <unistd.h>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <istream>
#include <limits>
#include <memory>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <string>
#include <vector>

static constexpr unsigned int sensorPollMs = 500;
static constexpr unsigned int sensorScaleFactor = 1000;
static constexpr size_t warnAfterErrorCount = 10;

static constexpr double maxReading = 127;
static constexpr double minReading = -128;

HwmonTempSensor::HwmonTempSensor(
    const std::string& path, const std::string& objectType,
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_service& io, const std::string& sensorName,
    std::vector<thresholds::Threshold>&& _thresholds,
    const std::string& sensorConfiguration, const PowerState powerState) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(_thresholds), sensorConfiguration, objectType, maxReading,
           minReading),
    objServer(objectServer), inputDev(io, open(path.c_str(), O_RDONLY)),
    waitTimer(io), path(path), errCount(0), readState(powerState)
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
    setInitialProperties(conn);
    setupPowerMatch(conn);
    setupRead();
}

HwmonTempSensor::~HwmonTempSensor()
{
    // close the input dev to cancel async operations
    inputDev.close();
    waitTimer.cancel();
    objServer.remove_interface(thresholdInterfaceWarning);
    objServer.remove_interface(thresholdInterfaceCritical);
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

void HwmonTempSensor::setupRead(void)
{
    boost::asio::async_read_until(
        inputDev, readBuf, '\n',
        [&](const boost::system::error_code& ec,
            std::size_t /*bytes_transfered*/) { handleResponse(ec); });
}

void HwmonTempSensor::handleResponse(const boost::system::error_code& err)
{
    if (err == boost::system::errc::bad_file_descriptor)
    {
        return; // we're being destroyed
    }
    std::istream responseStream(&readBuf);
    if (!err)
    {
        std::string response;
        std::getline(responseStream, response);
        try
        {
            double nvalue = std::stod(response);
            nvalue /= sensorScaleFactor;
            updateValue(nvalue);
            errCount = 0;
        }
        catch (const std::invalid_argument&)
        {
            errCount++;
        }
    }
    else if (readState == PowerState::on && !isPowerOn())
    {
        errCount = 0;
        updateValue(std::numeric_limits<double>::quiet_NaN());
    }
    else
    {
        errCount++;
    }

    // only print once
    if (errCount == warnAfterErrorCount)
    {
        std::cerr << "Failure to read sensor " << name << " at " << path
                  << " ec:" << err << "\n";
    }

    if (errCount >= warnAfterErrorCount)
    {
        updateValue(0);
    }
    responseStream.clear();
    inputDev.close();
    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0)
    {
        return; // we're no longer valid
    }
    inputDev.assign(fd);
    waitTimer.expires_from_now(boost::posix_time::milliseconds(sensorPollMs));
    waitTimer.async_wait([&](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        setupRead();
    });
}

void HwmonTempSensor::checkThresholds(void)
{
    if (readState == PowerState::on && !isPowerOn())
    {
        return;
    }
    thresholds::checkThresholds(this);
}
