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
                     double max, double min, const std::string& label,
                     size_t tSize) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(_thresholds), sensorConfiguration, objectType, max, min),
    objServer(objectServer), inputStream(io), path(path), errCount(0),
    slowCount(0), readCount(0), goodCount(0), sensorFactor(factor),
    disposition(DISPOSITION_NEW), readPending(false)
{
    if constexpr (DEBUG)
    {
        std::cerr << "Constructed sensor: path " << path << " type "
                  << objectType << " config " << sensorConfiguration
                  << " typename " << sensorTypeName << " factor " << factor
                  << " min " << min << " max " << max << " name \""
                  << sensorName << "\"\n";
    }

    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0)
    {
        std::cerr << "PSU sensor failed to open file\n";
        return;
    }

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
    if (label.empty() || tSize == _thresholds.size())
    {
        setInitialProperties(conn);
    }
    else
    {
        setInitialProperties(conn, label, tSize);
    }

    association = objectServer.add_interface(dbusPath, association::interface);

    createInventoryAssoc(conn, association, configurationPath);

    inputStream.assign(fd);
    inputStream.non_blocking(true);
    // The sensor is now ready to read. Do not schedule input here, because
    // input is now all scheduled simultaneously by sensorScheduleAll() in main.
}

PSUSensor::~PSUSensor()
{
    inputStream.cancel();
    inputStream.close();
    objServer.remove_interface(association);
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(thresholdInterfaceWarning);
    objServer.remove_interface(thresholdInterfaceCritical);
    objServer.remove_interface(association);
}

PSUDisposition PSUSensor::prepareInput()
{
    // If previous async read did not complete, note and cancel it
    if (readPending)
    {
        inputStream.cancel();

        ++slowCount;
        if (slowCount == warnAfterErrorCount)
        {
            std::cerr << "Slow sensor is missing readings: " << path << "\n";
        }

        // Callback never called, replace anything earlier set by it
        disposition = DISPOSITION_SLOW;

        if constexpr (DEBUG)
        {
            // Show this every time a reading was missed, hopefully rare
            std::cerr << "Slow sensor: slow=" << slowCount
                      << " read=" << readCount << " good=" << goodCount
                      << " path=" << path << "\n";
        }
    }

    // Rewind hwmon file input stream to start
    lseek(inputStream.native_handle(), 0, SEEK_SET);

    // Initiate another async read
    readPending = true;
    boost::asio::async_read_until(
        inputStream, inputBuf, '\n',
        [&](const boost::system::error_code& ec,
            std::size_t /*bytes_transferred*/) { handleResponse(ec); });

    return disposition;
}

void PSUSensor::handleResponse(const boost::system::error_code& err)
{
    // Note the async read as having completed, regardless of success
    readPending = false;
    disposition = DISPOSITION_BAD;

    if (err == boost::system::errc::bad_file_descriptor)
    {
        std::cerr << "Bad file descriptor from\n";
        return;
    }

    std::istream responseStream(&inputBuf);
    if (!err)
    {
        std::string response;
        try
        {
            // Read one line, discard newline and all thereafter
            std::getline(responseStream, response);
            inputBuf.consume(inputBuf.size());

            // Convert text to double, and scale appropriately
            double nvalue = std::stod(response);
            nvalue /= sensorFactor;
            updateValue(nvalue);

            // Note this is the only good path within this function
            disposition = DISPOSITION_GOOD;
            errCount = 0;
            ++goodCount;
        }
        catch (const std::invalid_argument&)
        {
            std::cerr << "Could not parse " << response << "\n";
            errCount++;
        }
    }
    else
    {
        std::cerr << "System error " << err << "\n";
        errCount++;
    }

    if (errCount >= warnAfterErrorCount)
    {
        if (errCount == warnAfterErrorCount)
        {
            std::cerr << "Failure to read sensor " << name << "\n";
        }
        updateValue(0);
        errCount++;
    }

    // Do not reschedule input here, because input is now all rescheduled
    // simultaneously by sensorScheduleAll() in main.
    ++readCount;
    if constexpr (DEBUG)
    {
        // Throttle to avoid debug output flood
        if ((readCount % warnAfterErrorCount) == 0)
        {
            std::cerr << "Sensor: slow=" << slowCount << " read=" << readCount
                      << " good=" << goodCount << " value=" << value
                      << " path=" << path << "\n";
        }
    }
}

void PSUSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}
