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

#include <unistd.h>

#include <PSUSensor.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <fstream>
#include <iostream>
#include <limits>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <string>

PSUSensor::PSUSensor(const std::string& path, const std::string& objectType,
                     sdbusplus::asio::object_server& objectServer,
                     std::shared_ptr<sdbusplus::asio::connection>& conn,
                     boost::asio::io_service& io, const std::string& sensorName,
                     std::vector<thresholds::Threshold>&& _thresholds,
                     const std::string& sensorConfiguration,
                     std::string& sensorTypeName, unsigned int factor,
                     double max, double min) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"), path,
           std::move(_thresholds), sensorConfiguration, objectType, max, min),
    objServer(objectServer), inputDev(io, open(path.c_str(), O_RDONLY)),
    waitTimer(io), errCount(0), sensorFactor(factor)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/" + sensorTypeName + name,
        "xyz.openbmc_project.Sensor.Value");

    if (thresholds::hasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/" + sensorTypeName + name,
            "xyz.openbmc_project.Sensor.Threshold.Warning");
    }
    if (thresholds::hasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/" + sensorTypeName + name,
            "xyz.openbmc_project.Sensor.Threshold.Critical");
    }
    setInitialProperties(conn);
    setupRead();
}

PSUSensor::~PSUSensor()
{
    inputDev.close();
    waitTimer.cancel();
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(thresholdInterfaceWarning);
    objServer.remove_interface(thresholdInterfaceCritical);
}

void PSUSensor::setupRead(void)
{
    boost::asio::async_read_until(
        inputDev, readBuf, '\n',
        [&](const boost::system::error_code& ec,
            std::size_t /*bytes_transfered*/) { handleResponse(ec); });
}

void PSUSensor::handleResponse(const boost::system::error_code& err)
{
    if (err == boost::system::errc::bad_file_descriptor)
    {
        return;
    }
    std::istream responseStream(&readBuf);
    if (!err)
    {
        std::string response;
        try
        {
            std::getline(responseStream, response);
            float nvalue = std::stof(response);
            responseStream.clear();
            nvalue /= sensorFactor;
            if (overridenState)
            {
                nvalue = overriddenValue;
            }
            if (nvalue != value)
            {
                updateValue(nvalue);
            }
            errCount = 0;
        }
        catch (const std::invalid_argument&)
        {
            errCount++;
        }
    }
    else
    {
        errCount++;
    }

    if (errCount >= warnAfterErrorCount)
    {
        if (errCount == warnAfterErrorCount)
        {
            std::cerr << "Failure to read sensor " << name << " at " << path
                      << "\n";
        }
        updateValue(0);
        errCount++;
    }

    responseStream.clear();
    inputDev.close();
    int fd = open(path.c_str(), O_RDONLY);
    if (fd <= 0)
    {
        return;
    }
    inputDev.assign(fd);
    waitTimer.expires_from_now(boost::posix_time::milliseconds(sensorPollMs));
    waitTimer.async_wait([&](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return;
        }
        setupRead();
    });
}

void PSUSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}

PSUPWMSensor::PSUPWMSensor(const std::string& targetPath,
                           sdbusplus::asio::object_server& objectServer,
                           const std::string& sensorConfiguration,
                           const std::string& pwmName) :
    PwmSensor(boost::replace_all_copy(targetPath, "target", pwmName),
              objectServer, sensorConfiguration),
    targetPath(targetPath)
{
}

void PSUPWMSensor::setValue(uint32_t value)
{
    std::ofstream ref(targetPath);
    if (!ref.good())
    {
        throw std::runtime_error("Bad Write File");
        return;
    }
    ref << value;
}

uint32_t PSUPWMSensor::getValue(bool errThrow)
{
    std::ifstream ref(targetPath);
    if (!ref.good())
    {
        return -1;
    }
    std::string line;
    if (!std::getline(ref, line))
    {
        return -1;
    }
    try
    {
        uint32_t value = std::stoi(line);
        return value;
    }
    catch (std::invalid_argument)
    {
        std::cerr << "Error reading pwm at " << targetPath << "\n";
        // throw if not initial read to be caught by dbus bindings
        if (errThrow)
        {
            throw std::runtime_error("Bad Read");
        }
    }
    return 0;
}
