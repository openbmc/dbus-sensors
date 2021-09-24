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
#include <boost/asio/read_until.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <iostream>
#include <istream>
#include <limits>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

static constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";

static constexpr bool debug = false;

PSUSensor::PSUSensor(const std::string& path, const std::string& objectType,
                     sdbusplus::asio::object_server& objectServer,
                     std::shared_ptr<sdbusplus::asio::connection>& conn,
                     boost::asio::io_service& io, const std::string& sensorName,
                     std::vector<thresholds::Threshold>&& thresholdsIn,
                     const std::string& sensorConfiguration,
                     const PowerState& powerState,
                     const std::string& sensorUnits, unsigned int factor,
                     double max, double min, double offset,
                     const std::string& label, size_t tSize, double pollRate) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(thresholdsIn), sensorConfiguration, objectType, false, max,
           min, conn, powerState),
    std::enable_shared_from_this<PSUSensor>(), objServer(objectServer),
    inputDev(io), waitTimer(io), path(path), pathRatedMax(""), pathRatedMin(""),
    sensorFactor(factor), ratedReadCounter(0),
    maxRatedValue(std::numeric_limits<double>::quiet_NaN()),
    minRatedValue(std::numeric_limits<double>::quiet_NaN()),
    sensorOffset(offset), thresholdTimer(io)
{
    std::string unitPath = sensor_paths::getPathForUnits(sensorUnits);
    if constexpr (debug)
    {
        std::cerr << "Constructed sensor: path " << path << " type "
                  << objectType << " config " << sensorConfiguration
                  << " typename " << unitPath << " factor " << factor << " min "
                  << min << " max " << max << " offset " << offset << " name \""
                  << sensorName << "\"\n";
    }
    if (pollRate > 0.0)
    {
        sensorPollMs = static_cast<unsigned int>(pollRate * 1000);
    }

    fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0)
    {
        std::cerr << "PSU sensor failed to open file\n";
        return;
    }
    inputDev.assign(fd);

    std::string dbusPath = sensorPathPrefix + unitPath + "/" + name;

    sensorInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Sensor.Value");

    ratedValueInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Sensor.RatedValue");

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

    ratedValueInterface->register_property("MaxValue", maxRatedValue);
    ratedValueInterface->register_property("MinValue", minRatedValue);
    if (!ratedValueInterface->initialize())
    {
        std::cerr << "error initializing rated value interface\n";
    }

    // This should be called before initializing association.
    // createInventoryAssoc() does add more associations before doing
    // register and initialize "Associations" property.
    if (label.empty() || tSize == thresholds.size())
    {
        setInitialProperties(conn, sensorUnits);
    }
    else
    {
        setInitialProperties(conn, sensorUnits, label, tSize);
    }

    association = objectServer.add_interface(dbusPath, association::interface);

    createInventoryAssoc(conn, association, configurationPath);

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

PSUSensor::~PSUSensor()
{
    waitTimer.cancel();
    inputDev.close();
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(ratedValueInterface);
    objServer.remove_interface(thresholdInterfaceWarning);
    objServer.remove_interface(thresholdInterfaceCritical);
    objServer.remove_interface(association);
}

void PSUSensor::setupRead(void)
{
    if (!readingStateGood())
    {
        markAvailable(false);
        updateValue(std::numeric_limits<double>::quiet_NaN());
        restartRead();
        return;
    }

    std::weak_ptr<PSUSensor> weakRef = weak_from_this();
    inputDev.async_wait(boost::asio::posix::descriptor_base::wait_read,
                        [weakRef](const boost::system::error_code& ec) {
                            std::shared_ptr<PSUSensor> self = weakRef.lock();
                            if (self)
                            {
                                self->handleResponse(ec);
                            }
                        });
}

void PSUSensor::restartRead(void)
{
    std::weak_ptr<PSUSensor> weakRef = weak_from_this();
    waitTimer.expires_from_now(boost::posix_time::milliseconds(sensorPollMs));
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            std::cerr << "Failed to reschedule\n";
            return;
        }
        std::shared_ptr<PSUSensor> self = weakRef.lock();
        if (self)
        {
            self->setupRead();
        }
    });
}

void PSUSensor::updateRatedValues(void)
{
    if (auto newVal = readFile(pathRatedMin, sensorFactor))
    {
        updateProperty(ratedValueInterface, minValue, *newVal, "MinValue");
    }

    if (auto newVal = readFile(pathRatedMax, sensorFactor))
    {
        updateProperty(ratedValueInterface, maxValue, *newVal, "MaxValue");
    }
}

// Create a buffer expected to be able to hold more characters than will be
// present in the input file.
static constexpr uint32_t psuBufLen = 128;
void PSUSensor::handleResponse(const boost::system::error_code& err)
{
    if ((err == boost::system::errc::bad_file_descriptor) ||
        (err == boost::asio::error::misc_errors::not_found))
    {
        std::cerr << "Bad file descriptor for " << path << "\n";
        return;
    }

    std::string buffer;
    buffer.resize(psuBufLen);
    lseek(fd, 0, SEEK_SET);
    int rdLen = read(fd, buffer.data(), psuBufLen);

    if (rdLen > 0)
    {
        try
        {
            rawValue = std::stod(buffer);

            updateValue((rawValue / sensorFactor) + sensorOffset);

            if (ratedReadCounter++ % 8 == 0)
            {
                updateRatedValues();
            }
        }
        catch (const std::invalid_argument&)
        {
            std::cerr << "Could not parse  input from " << path << "\n";
            incrementError();
        }
    }
    else
    {
        std::cerr
            << "System error " << errno << " ("
            << std::generic_category().default_error_condition(errno).message()
            << ") reading from " << path << ", line: " << __LINE__ << "\n";
        incrementError();
    }

    lseek(fd, 0, SEEK_SET);
    restartRead();
}

void PSUSensor::checkThresholds(void)
{
    if (!readingStateGood())
    {
        return;
    }

    thresholds::checkThresholdsPowerDelay(weak_from_this(), thresholdTimer);
}
