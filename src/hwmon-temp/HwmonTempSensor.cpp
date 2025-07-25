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

#include "DeviceMgmt.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/random_access_file.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <charconv>
#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

// Temperatures are read in milli degrees Celsius, we need degrees Celsius.
// Pressures are read in kilopascal, we need Pascals.  On D-Bus for Open BMC
// we use the International System of Units without prefixes.
// Links to the kernel documentation:
// https://www.kernel.org/doc/Documentation/hwmon/sysfs-interface
// https://www.kernel.org/doc/Documentation/ABI/testing/sysfs-bus-iio
// For IIO RAW sensors we get a raw_value, an offset, and scale to compute
// the value = (raw_value + offset) * scale

HwmonTempSensor::HwmonTempSensor(
    const std::string& path, const std::string& objectType,
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const std::string& sensorName,
    std::vector<thresholds::Threshold>&& thresholdsIn,
    const struct SensorParams& thisSensorParameters, const float pollRate,
    const std::string& sensorConfiguration, const PowerState powerState,
    const std::shared_ptr<I2CDevice>& i2cDevice) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(thresholdsIn), sensorConfiguration, objectType, false,
           false, thisSensorParameters.maxValue, thisSensorParameters.minValue,
           conn, powerState),
    i2cDevice(i2cDevice), objServer(objectServer),
    inputDev(io, path, boost::asio::random_access_file::read_only),
    waitTimer(io), path(path), offsetValue(thisSensorParameters.offsetValue),
    scaleValue(thisSensorParameters.scaleValue),
    sensorPollMs(static_cast<unsigned int>(pollRate * 1000))
{
    std::string dbusPath = "/xyz/openbmc_project/sensors/" +
                           thisSensorParameters.typeName + "/" + name;
    sensorInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        size_t index = static_cast<size_t>(threshold.level);
        if (thresholdInterfaces[index])
        {
            lg2::error("{INTERFACE} under {PATH} has already been created",
                       "INTERFACE", interface, "PATH", dbusPath);
            continue;
        }

        thresholdInterfaces[index] =
            objectServer.add_interface(dbusPath, interface);
    }
    association = objectServer.add_interface(dbusPath, association::interface);
    setInitialProperties(thisSensorParameters.units);
}

bool HwmonTempSensor::isActive()
{
    return inputDev.is_open();
}

void HwmonTempSensor::activate(const std::string& newPath,
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

void HwmonTempSensor::deactivate()
{
    markAvailable(false);
    // close the input dev to cancel async operations
    inputDev.close();
    waitTimer.cancel();
    i2cDevice = nullptr;
    path = "";
}

HwmonTempSensor::~HwmonTempSensor()
{
    deactivate();

    for (const auto& iface : thresholdInterfaces)
    {
        objServer.remove_interface(iface);
    }
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

void HwmonTempSensor::setupRead()
{
    if (!readingStateGood())
    {
        markAvailable(false);
        updateValue(std::numeric_limits<double>::quiet_NaN());
        restartRead();
        return;
    }

    std::weak_ptr<HwmonTempSensor> weakRef = weak_from_this();
    inputDev.async_read_some_at(
        0, boost::asio::buffer(readBuf),
        [weakRef](const boost::system::error_code& ec, std::size_t bytesRead) {
            std::shared_ptr<HwmonTempSensor> self = weakRef.lock();
            if (self)
            {
                self->handleResponse(ec, bytesRead);
            }
        });
}

void HwmonTempSensor::restartRead()
{
    std::weak_ptr<HwmonTempSensor> weakRef = weak_from_this();
    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        std::shared_ptr<HwmonTempSensor> self = weakRef.lock();
        if (!self)
        {
            return;
        }
        self->setupRead();
    });
}

void HwmonTempSensor::handleResponse(const boost::system::error_code& err,
                                     size_t bytesRead)
{
    if ((err == boost::system::errc::bad_file_descriptor) ||
        (err == boost::asio::error::misc_errors::not_found))
    {
        lg2::error("Hwmon temp sensor '{NAME}' removed '{PATH}'", "NAME", name,
                   "PATH", path);
        return; // we're being destroyed
    }

    if (!err)
    {
        const char* bufEnd = readBuf.data() + bytesRead;
        int nvalue = 0;
        std::from_chars_result ret =
            std::from_chars(readBuf.data(), bufEnd, nvalue);
        if (ret.ec != std::errc())
        {
            incrementError();
        }
        else
        {
            updateValue((nvalue + offsetValue) * scaleValue);
        }
    }
    else
    {
        incrementError();
    }

    restartRead();
}

void HwmonTempSensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}
