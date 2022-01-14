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
    boost::asio::io_service& io, const std::string& sensorName,
    std::vector<thresholds::Threshold>&& thresholdsIn,
    const struct SensorParams& thisSensorParameters, const float pollRate,
    const std::string& sensorConfiguration, const PowerState powerState) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(thresholdsIn), sensorConfiguration, objectType, false,
           false, thisSensorParameters.maxValue, thisSensorParameters.minValue,
           conn, powerState),
    std::enable_shared_from_this<HwmonTempSensor>(), objServer(objectServer),
    inputDev(io), waitTimer(io), path(path),
    offsetValue(thisSensorParameters.offsetValue),
    scaleValue(thisSensorParameters.scaleValue),
    sensorPollMs(static_cast<unsigned int>(pollRate * 1000))
{
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0)
    {
        std::cerr << "HwmonTempSensor " << sensorName << " failed to open "
                  << path << "\n";
    }
    inputDev.assign(fd);

    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/" + thisSensorParameters.typeName + "/" +
            name,
        "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface("/xyz/openbmc_project/sensors/" +
                                           thisSensorParameters.typeName + "/" +
                                           name,
                                       interface);
    }
    association = objectServer.add_interface("/xyz/openbmc_project/sensors/" +
                                                 thisSensorParameters.typeName +
                                                 "/" + name,
                                             association::interface);
    setInitialProperties(thisSensorParameters.units);
}

HwmonTempSensor::~HwmonTempSensor()
{
    // close the input dev to cancel async operations
    inputDev.close();
    waitTimer.cancel();
    for (const auto& iface : thresholdInterfaces)
    {
        objServer.remove_interface(iface);
    }
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

void HwmonTempSensor::setupRead(void)
{
    if (!readingStateGood())
    {
        markAvailable(false);
        updateValue(std::numeric_limits<double>::quiet_NaN());
        restartRead();
        return;
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

void HwmonTempSensor::restartRead()
{
    std::weak_ptr<HwmonTempSensor> weakRef = weak_from_this();
    waitTimer.expires_from_now(boost::posix_time::milliseconds(sensorPollMs));
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
    if (!err)
    {
        std::string response;
        std::getline(responseStream, response);
        try
        {
            rawValue = std::stod(response);
            double nvalue = (rawValue + offsetValue) * scaleValue;
            updateValue(nvalue);
        }
        catch (const std::invalid_argument&)
        {
            incrementError();
        }
    }
    else
    {
        incrementError();
    }

    responseStream.clear();
    inputDev.close();

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0)
    {
        std::cerr << "Hwmon temp sensor " << name << " not valid " << path
                  << "\n";
        return; // we're no longer valid
    }
    inputDev.assign(fd);
    restartRead();
}

void HwmonTempSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}
