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

#include "PSUSensor.hpp"

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

static constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";

static constexpr bool DEBUG = false;

PSUSensor::PSUSensor(const std::string& path, const std::string& objectType,
                     sdbusplus::asio::object_server& objectServer,
                     std::shared_ptr<sdbusplus::asio::connection>& conn,
                     boost::asio::io_service& io, const std::string& sensorName,
                     std::vector<thresholds::Threshold>&& _thresholds,
                     const std::string& sensorConfiguration,
                     std::string& sensorTypeName, unsigned int factor,
                     double max, double min) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(_thresholds), sensorConfiguration, objectType, max, min),
    objServer(objectServer), inputDev(io), waitTimer(io), path(path),
    errCount(0),

    sensorFactor(factor)
{
    if constexpr (DEBUG)
    {
        std::cerr << "Constructed sensor: path " << path << " type "
                  << objectType << " config " << sensorConfiguration
                  << " typename " << sensorTypeName << " factor " << factor
                  << " min " << min << " max " << max << " name \""
                  << sensorName << "\"\n";
    }

    fd = open(path.c_str(), O_RDONLY);
    if (fd < 0)
    {
        std::cerr << "PSU sensor failed to open file\n";
        return;
    }
    inputDev.assign(fd);

    std::string dbusPath = sensorPathPrefix + sensorTypeName + name;

    sensorInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Sensor.Value");

    if (thresholds::hasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            dbusPath, "xyz.openbmc_project.Sensor.Threshold.Warning");
    }
    if (thresholds::hasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            dbusPath, "xyz.openbmc_project.Sensor.Threshold.Critical");
    }

    // This should be called before initializing association.
    // createInventoryAssoc() does add more associations before doing
    // register and initialize "Associations" property.
    setInitialProperties(conn);

    association = objectServer.add_interface(dbusPath, association::interface);

    createInventoryAssoc(conn, association, configurationPath);
    setupRead();
}

PSUSensor::~PSUSensor()
{
    waitTimer.cancel();
    inputDev.close();
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(thresholdInterfaceWarning);
    objServer.remove_interface(thresholdInterfaceCritical);
    objServer.remove_interface(association);
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
        std::cerr << "Bad file descriptor from " << path << "\n";
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

            if constexpr (DEBUG)
            {
                std::cerr << "Read " << path << " scale " << sensorFactor
                          << " value " << nvalue << "\n";
            }
            if (static_cast<double>(nvalue) != value)
            {
                if constexpr (DEBUG)
                {
                    std::cerr << "Update " << path << " from " << value
                              << " to " << nvalue << "\n";
                }
                updateValue(nvalue);
            }
            errCount = 0;
        }
        catch (const std::invalid_argument&)
        {
            std::cerr << "Could not parse " << response << " from path " << path
                      << "\n";
            errCount++;
        }
    }
    else
    {
        std::cerr << "System error " << err << " from path " << path << "\n";
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

    lseek(fd, 0, SEEK_SET);
    waitTimer.expires_from_now(boost::posix_time::milliseconds(sensorPollMs));
    waitTimer.async_wait([&](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            std::cerr << "Failed to reschedule wait for " << path << "\n";
            return;
        }
        setupRead();
    });
}

void PSUSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}
