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

#include "DeviceMgmt.hpp"
#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

#include <boost/asio/buffer.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/random_access_file.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <charconv>
#include <chrono>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

static constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";

PSUSensor::PSUSensor(
    const std::string& path, const std::string& objectType,
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const std::string& sensorName,
    std::vector<thresholds::Threshold>&& thresholdsIn,
    const std::string& sensorConfiguration, const PowerState& powerState,
    const std::string& sensorUnits, unsigned int factor, double max, double min,
    double offset, const std::string& label, size_t tSize, double pollRate,
    const std::shared_ptr<I2CDevice>& i2cDevice) :
    Sensor(escapeName(sensorName), std::move(thresholdsIn), sensorConfiguration,
           objectType, false, false, max, min, conn, powerState),
    i2cDevice(i2cDevice), objServer(objectServer),
    inputDev(io, path, boost::asio::random_access_file::read_only),
    waitTimer(io), path(path), sensorFactor(factor), sensorOffset(offset),
    thresholdTimer(io)
{
    std::string unitPath = sensor_paths::getPathForUnits(sensorUnits);
    lg2::debug(
        "Constructed sensor - path: {PATH}, type: {TYPE}, config: {CONFIG}, "
        "typename: {TYPENAME}, factor: {FACTOR}, min: {MIN}, max: {MAX}, "
        "offset: {OFFSET}, name: {NAME}",
        "PATH", path, "TYPE", objectType, "CONFIG", sensorConfiguration,
        "TYPENAME", unitPath, "FACTOR", factor, "MIN", min, "MAX", max,
        "OFFSET", offset, "NAME", sensorName);
    if (pollRate > 0.0)
    {
        sensorPollMs = static_cast<unsigned int>(pollRate * 1000);
    }

    std::string dbusPath = sensorPathPrefix + unitPath + "/" + name;

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
        setInitialProperties(sensorUnits);
    }
    else
    {
        setInitialProperties(sensorUnits, label, tSize);
    }

    association = objectServer.add_interface(dbusPath, association::interface);

    createInventoryAssoc(conn, association, configurationPath);
}

PSUSensor::~PSUSensor()
{
    deactivate();

    objServer.remove_interface(sensorInterface);
    for (const auto& iface : thresholdInterfaces)
    {
        objServer.remove_interface(iface);
    }
    objServer.remove_interface(association);
}

bool PSUSensor::isActive()
{
    return inputDev.is_open();
}

void PSUSensor::activate(const std::string& newPath,
                         const std::shared_ptr<I2CDevice>& newI2CDevice)
{
    if (isActive())
    {
        // Avoid activating an active sensor
        return;
    }
    path = newPath;
    i2cDevice = newI2CDevice;
    inputDev.open(path, boost::asio::random_access_file::read_only);
    markAvailable(true);
    setupRead();
}

void PSUSensor::deactivate()
{
    markAvailable(false);
    // close the input dev to cancel async operations
    inputDev.close();
    waitTimer.cancel();
    i2cDevice = nullptr;
    path = "";
}

void PSUSensor::handleResponseStatic(const std::weak_ptr<PSUSensor>& weak,
                                     const boost::system::error_code& ec,
                                     size_t bytesRead)
{
    std::shared_ptr<PSUSensor> self = weak.lock();
    if (!self)
    {
        return;
    }

    self->handleResponse(ec, bytesRead);
}

void PSUSensor::setupRead()
{
    if (!readingStateGood())
    {
        markAvailable(false);
        updateValue(std::numeric_limits<double>::quiet_NaN());
        restartRead();
        return;
    }

    std::weak_ptr<PSUSensor> weak = weak_from_this();
    inputDev.async_read_some_at(
        0, boost::asio::buffer(buffer),
        std::bind_front(&PSUSensor::handleResponseStatic, weak_from_this()));
}

void PSUSensor::restartRead()
{
    std::weak_ptr<PSUSensor> weakRef = weak_from_this();
    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            lg2::error("Failed to reschedule");
            return;
        }
        std::shared_ptr<PSUSensor> self = weakRef.lock();
        if (self)
        {
            self->setupRead();
        }
    });
}

// Create a buffer expected to be able to hold more characters than will be
// present in the input file.
void PSUSensor::handleResponse(const boost::system::error_code& err,
                               size_t bytesRead)
{
    if (err == boost::asio::error::operation_aborted)
    {
        lg2::error("Read aborted");
        return;
    }
    if ((err == boost::system::errc::bad_file_descriptor) ||
        (err == boost::asio::error::misc_errors::not_found))
    {
        lg2::error("Bad file descriptor for '{PATH}'", "PATH", path);
        return;
    }
    if (err || bytesRead == 0)
    {
        if (readingStateGood())
        {
            lg2::error("'{NAME}' read failed", "NAME", name);
        }
        restartRead();
        return;
    }

    const char* bufferEnd = buffer.data() + bytesRead;
    std::from_chars_result ret =
        std::from_chars(buffer.data(), bufferEnd, rawValue);

    if (ret.ec != std::errc())
    {
        lg2::error("Could not parse input from '{PATH}'", "PATH", path);
        incrementError();
    }
    else
    {
        updateValue((rawValue / sensorFactor) + sensorOffset);
    }

    restartRead();
}

void PSUSensor::checkThresholds()
{
    if (!readingStateGood())
    {
        return;
    }

    thresholds::checkThresholdsPowerDelay(weak_from_this(), thresholdTimer);
}
