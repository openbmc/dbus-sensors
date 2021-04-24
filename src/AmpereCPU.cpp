/*
// Copyright 2022 Ampere Computing LLC
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
#include <Utils.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/asio/random_access_file.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <charconv>
#include <iostream>
#include <istream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

static constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";
// scaling factor from hwmon
static constexpr unsigned int sensorScaleFactor = 1000;
static constexpr double roundFactor = 10000.000; // 3 decimal places

AmpereCPUSensor::AmpereCPUSensor(
    const std::string& path, const std::string& objectType,
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_service& io, const std::string& sensorName,
    std::vector<thresholds::Threshold>&& thresholdsIn,
    const std::string& sensorConfiguration, std::string& sensorTypeName,
    double factor, double max, double min, const std::string& label,
    size_t tSize, PowerState readState) :
    Sensor(escapeName(sensorName), std::move(thresholdsIn), sensorConfiguration,
           objectType, false, false, max, min, conn, readState),
    objServer(objectServer),
    inputDev(io, path, boost::asio::random_access_file::read_only),
    waitTimer(io), path(path), sensorFactor(factor)
{
    std::string unitPath = sensor_paths::getPathForUnits(sensorTypeName);
    sdbusplus::message::object_path objectPath(sensorPathPrefix);
    objectPath /= unitPath;
    objectPath /= name;

    std::string dbusPath = objectPath.str;
    sensorInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Sensor.Value");
    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface(dbusPath, interface);
    }

    // This should be called before initializing association.
    // createInventoryAssoc() does add more associations before doing
    // register and initialize "Associations" property.
    if (label.empty() || tSize == thresholds.size())
    {
        setInitialProperties(sensorTypeName);
    }
    else
    {
        setInitialProperties(sensorTypeName, label, tSize);
    }
    association = objectServer.add_interface(dbusPath, association::interface);
    createInventoryAssoc(conn, association, configurationPath);
}

AmpereCPUSensor::~AmpereCPUSensor()
{
    waitTimer.cancel();
    inputDev.close();
    objServer.remove_interface(sensorInterface);
    for (const auto& iface : thresholdInterfaces)
    {
        objServer.remove_interface(iface);
    }
    objServer.remove_interface(association);
}

void AmpereCPUSensor::setupRead()
{
    std::weak_ptr<AmpereCPUSensor> weak = weak_from_this();
    inputDev.async_read_some_at(
        0, boost::asio::buffer(readBuf),
        [weak](const boost::system::error_code& ec, size_t bytesRead) {
        std::shared_ptr<AmpereCPUSensor> self = weak.lock();
        if (self)
        {
            self->handleResponse(ec, bytesRead);
        }
        });
}

void AmpereCPUSensor::restartRead(void)
{
    std::weak_ptr<AmpereCPUSensor> weakRef = weak_from_this();
    waitTimer.expires_from_now(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return;
        }
        std::shared_ptr<AmpereCPUSensor> self = weakRef.lock();
        if (self)
        {
            self->setupRead();
        }
    });
}

void AmpereCPUSensor::handleResponse(const boost::system::error_code& err,
                                     size_t bytesRead)
{
    if (err == boost::asio::error::operation_aborted)
    {
        std::cerr << "Read aborted\n";
        return;
    }
    if ((err == boost::system::errc::bad_file_descriptor) ||
        (err == boost::asio::error::misc_errors::not_found))
    {
        std::cerr << "Bad file descriptor for " << path << "\n";
        return;
    }

    if (!err)
    {
        try
        {
            const char* bufEnd = readBuf.data() + bytesRead;
            int rawValue = 0;
            std::from_chars_result ret =
                std::from_chars(readBuf.data(), bufEnd, rawValue);
            if (ret.ec == std::errc())
            {
                double nvalue =
                    ((double)rawValue / sensorScaleFactor) / sensorFactor;
                nvalue = std::round(nvalue * roundFactor) / roundFactor;
                updateValue(nvalue);
            }
        }
        catch (const std::invalid_argument&)
        {
            std::cerr << "Could not parse  input from " << path << "\n";
        }
    }

    restartRead();
}

void AmpereCPUSensor::checkThresholds()
{
    if (!readingStateGood())
    {
        return;
    }

    thresholds::checkThresholds(this);
}
