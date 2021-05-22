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

#include <unistd.h>

#include <HwmonTempSensor.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cerrno>
#include <iostream>
#include <istream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

static constexpr unsigned int sensorScaleFactor = 1000;
static constexpr size_t warnAfterErrorCount = 10;

static constexpr double maxReading = 127;
static constexpr double minReading = -128;

HwmonTempSensor::HwmonTempSensor(
    const std::string& path, const std::string& objectType,
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_service& io, const std::string& sensorName,
    std::vector<thresholds::Threshold>&& thresholdsIn, const float pollRate,
    const std::string& sensorConfiguration, const PowerState powerState) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(thresholdsIn), sensorConfiguration, objectType, maxReading,
           minReading, conn, powerState),
    std::enable_shared_from_this<HwmonTempSensor>(), objServer(objectServer),
    inputDev(io), waitTimer(io), path(path),
    sensorPollMs(static_cast<unsigned int>(pollRate * 1000))
{
    // Open here, not in inputDev() constructor call, so error can show
    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0)
    {
        int err = errno;
        std::cerr << "Hwmon temp sensor " << name
                  << " open error: " << std::strerror(err) << "\n";
    }

    // Even if error, assign anyway, checked by setupRead() before use
    inputDev.assign(fd);

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
    int fd = inputDev.native_handle();
    if (fd < 0)
    {
        std::cerr << "Hwmon temp sensor " << name << " closed " << path << "\n";
        return; // we're no longer valid
    }

    std::weak_ptr<HwmonTempSensor> weakRef = weak_from_this();

    boost::asio::async_read_until(inputDev, readBuf, '\n',
                                  [weakRef](const boost::system::error_code& ec,
                                            std::size_t /*bytes_transfered*/) {
                                      std::shared_ptr<HwmonTempSensor> self =
                                          weakRef.lock();
                                      if (self)
                                      {
                                          self->handleResponse(ec);
                                      }
                                  });
}

void HwmonTempSensor::handleResponse(const boost::system::error_code& err)
{
    if ((err == boost::system::errc::bad_file_descriptor) ||
        (err == boost::asio::error::misc_errors::not_found))
    {
        std::cerr << "Hwmon temp sensor " << name << " removed " << path
                  << "\n";
        return; // we're being destroyed
    }

    std::istream responseStream(&readBuf);
    bool errorHappened = false;
    if (!err)
    {
        std::string response;
        try
        {
            std::getline(responseStream, response);
            rawValue = std::stod(response);
            responseStream.clear();

            double nvalue = rawValue / sensorScaleFactor;
            updateValue(nvalue);
        }
        catch (const std::invalid_argument&)
        {
            std::cerr << "Hwmon temp sensor " << name << " parse error\n";
            errorHappened = true;
        }
    }
    else
    {
        std::cerr << "Hwmon temp sensor " << name
                  << " read error: " << err.message() << "\n";
        errorHappened = true;
    }

    if (errorHappened)
    {
        // Invalidate sensor value, to remove old misleading stale value
        updateValue(std::numeric_limits<double>::quiet_NaN());
        incrementError();

        // Reopen the hwmon file
        inputDev.close();
        int fd = open(path.c_str(), O_RDONLY);
        if (fd < 0)
        {
            int err = errno;
            std::cerr << "Hwmon temp sensor " << name
                      << " reopen error: " << std::strerror(err) << "\n";
        }

        // Even if error, assign anyway, checked by setupRead() before use
        inputDev.assign(fd);
    }
    else
    {
        // Rewind same fd, without reopening the hwmon file
        int fd = inputDev.native_handle();
        lseek(fd, 0, SEEK_SET);
    }

    waitTimer.expires_from_now(boost::posix_time::milliseconds(sensorPollMs));
    std::weak_ptr<HwmonTempSensor> weakRef = weak_from_this();
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        std::shared_ptr<HwmonTempSensor> self = weakRef.lock();
        if (ec == boost::asio::error::operation_aborted)
        {
            if (self)
            {
                std::cerr << "Hwmon temp sensor " << self->name
                          << " read cancelled " << self->path << "\n";
            }
            else
            {
                std::cerr << "Hwmon sensor read cancelled, no self\n";
            }
            return; // we're being canceled
        }
        if (ec)
        {
            std::cerr << "Hwmon temp sensor "
                      << (self ? self->name : "cancellation")
                      << " timer error: " << ec.message() << "\n";
            return;
        }
        if (self)
        {
            self->setupRead();
        }
    });
}

void HwmonTempSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}
