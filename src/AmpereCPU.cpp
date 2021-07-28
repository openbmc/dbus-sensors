/*
// Copyright 2021 Ampere Computing LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include <unistd.h>

#include <AmpereCPU.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <iostream>
#include <istream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

static constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";
static constexpr unsigned int sensorScaleFactor = 1000;
static constexpr double roundFactor = 10000; // 3 decimal places

static constexpr bool debug = false;

CPUSensor::CPUSensor(const std::string& path, const std::string& objectType,
                     sdbusplus::asio::object_server& objectServer,
                     std::shared_ptr<sdbusplus::asio::connection>& conn,
                     boost::asio::io_service& io, const std::string& sensorName,
                     std::vector<thresholds::Threshold>&& thresholdsIn,
                     const std::string& sensorConfiguration,
                     std::string& sensorTypeName, unsigned int factor,
                     double max, double min, bool addAssociation,
                     const std::string& label, size_t tSize,
                     PowerState readState) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(thresholdsIn), sensorConfiguration, objectType, false, max,
           min, conn, readState),
    std::enable_shared_from_this<CPUSensor>(), objServer(objectServer),
    inputDev(io), waitTimer(io), path(path), pathRatedMax(""), pathRatedMin(""),
    sensorFactor(factor), minMaxReadCounter(0)
{
    if constexpr (debug)
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
        std::cerr << "SOC sensor failed to open file\n";
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
    if (label.empty() || tSize == thresholds.size())
    {
        setInitialProperties(conn, sensorTypeName);
    }
    else
    {
        setInitialProperties(conn, sensorTypeName, label, tSize,
                             addAssociation);
    }
    if (addAssociation)
    {
        association =
            objectServer.add_interface(dbusPath, association::interface);
        createInventoryAssoc(conn, association, configurationPath);
    }

    if (auto fileParts = splitFileName(path))
    {
        auto& [type, nr, item] = *fileParts;
        if (item.compare("input") == 0)
        {
            pathRatedMax = boost::replace_all_copy(path, item, "rated_max");
            pathRatedMin = boost::replace_all_copy(path, item, "rated_min");
        }
    }
    if constexpr (debug)
    {
        std::cerr << "File: " << pathRatedMax
                  << " will be used to update MaxValue\n";
        std::cerr << "File: " << pathRatedMin
                  << " will be used to update MinValue\n";
    }
}

CPUSensor::~CPUSensor()
{
    waitTimer.cancel();
    inputDev.close();
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(thresholdInterfaceWarning);
    objServer.remove_interface(thresholdInterfaceCritical);
    objServer.remove_interface(association);
}

void CPUSensor::setupRead(void)
{
    std::shared_ptr<boost::asio::streambuf> buffer =
        std::make_shared<boost::asio::streambuf>();
    std::weak_ptr<CPUSensor> weakRef = weak_from_this();
    boost::asio::async_read_until(
        inputDev, *buffer, '\n',
        [weakRef, buffer](const boost::system::error_code& ec,
                          std::size_t /*bytes_transfered*/) {
            std::shared_ptr<CPUSensor> self = weakRef.lock();
            if (self)
            {
                self->readBuf = buffer;
                self->handleResponse(ec);
            }
        });
}

void CPUSensor::updateMinMaxValues(void)
{
    if (auto newVal = readFile(pathRatedMin, sensorFactor))
    {
        updateProperty(sensorInterface, minValue, *newVal, "MinValue");
    }

    if (auto newVal = readFile(pathRatedMax, sensorFactor))
    {
        updateProperty(sensorInterface, maxValue, *newVal, "MaxValue");
    }
}

void CPUSensor::handleResponse(const boost::system::error_code& err)
{
    if (!err)
    {
        std::string response;
        try
        {
            std::istream responseStream(readBuf.get());
            std::getline(responseStream, response);
            rawValue = std::stod(response);
            responseStream.clear();
            double nvalue = (rawValue / sensorScaleFactor) / sensorFactor;
            nvalue = std::round(nvalue * roundFactor) / roundFactor;

            updateValue(nvalue);

            if (minMaxReadCounter++ % 8 == 0)
            {
                updateMinMaxValues();
            }
        }
        catch (const std::invalid_argument&)
        {
            std::cerr << "Could not parse " << response << "\n";
        }
    }

    if (err == boost::system::errc::no_such_device ||
        err == boost::system::errc::no_such_device_or_address)
    {
        updateValue(std::numeric_limits<double>::quiet_NaN());
    }

    int fd = open(path.c_str(), O_RDONLY);
    if (fd > 0)
    {
        inputDev.close();
        inputDev.assign(fd);
    }

    std::weak_ptr<CPUSensor> weakRef = weak_from_this();
    waitTimer.expires_from_now(boost::posix_time::milliseconds(sensorPollMs));
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        std::shared_ptr<CPUSensor> self = weakRef.lock();
        if (ec == boost::asio::error::operation_aborted)
        {
            std::cerr << "Failed to reschedule\n";
            return;
        }
        if (self)
        {
            self->setupRead();
        }
    });
}

void CPUSensor::checkThresholds(void)
{
    if (!readingStateGood())
    {
        return;
    }

    thresholds::checkThresholds(this);
}
