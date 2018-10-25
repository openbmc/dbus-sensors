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
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <limits>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <string>

static constexpr unsigned int sensorPollMs = 500;
static constexpr unsigned int sensorScaleFactor = 1000;
static constexpr size_t warnAfterErrorCount = 10;

HwmonTempSensor::HwmonTempSensor(
    const std::string &path, const std::string &objectType,
    sdbusplus::asio::object_server &objectServer,
    std::shared_ptr<sdbusplus::asio::connection> &conn,
    boost::asio::io_service &io, const std::string &sensorName,
    std::vector<thresholds::Threshold> &&_thresholds,
    const std::string &sensorConfiguration) :
    Sensor(),
    path(path), objectType(objectType), configuration(sensorConfiguration),
    objServer(objectServer),
    name(boost::replace_all_copy(sensorName, " ", "_")),
    inputDev(io, open(path.c_str(), O_RDONLY)), waitTimer(io), errCount(0),
    // todo, get these from config
    maxValue(127), minValue(-128)
{
    thresholds = std::move(_thresholds);
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
    setInitialProperties(conn);
    setupRead();
}

HwmonTempSensor::~HwmonTempSensor()
{
    // close the input dev to cancel async operations
    inputDev.close();
    waitTimer.cancel();
    objServer.remove_interface(thresholdInterfaceWarning);
    objServer.remove_interface(thresholdInterfaceCritical);
    objServer.remove_interface(sensorInterface);
}

void HwmonTempSensor::setupRead(void)
{
    boost::asio::async_read_until(
        inputDev, readBuf, '\n',
        [&](const boost::system::error_code &ec,
            std::size_t /*bytes_transfered*/) { handleResponse(ec); });
}

void HwmonTempSensor::handleResponse(const boost::system::error_code &err)
{
    if (err == boost::system::errc::bad_file_descriptor)
    {
        return; // we're being destroyed
    }
    std::istream responseStream(&readBuf);
    if (!err)
    {
        std::string response;
        std::getline(responseStream, response);
        try
        {
            float nvalue = std::stof(response);

            nvalue /= sensorScaleFactor;
            if (nvalue != value)
            {
                updateValue(nvalue);
            }
            errCount = 0;
        }
        catch (const std::invalid_argument &)
        {
            errCount++;
        }
    }
    else
    {
        std::cerr << "Failure to read sensor " << name << " at " << path
                  << "\n";
        errCount++;
    }
    // only send value update once
    if (errCount == warnAfterErrorCount)
    {
        updateValue(0);
    }
    responseStream.clear();
    inputDev.close();
    int fd = open(path.c_str(), O_RDONLY);
    if (fd <= 0)
    {
        return; // we're no longer valid
    }
    inputDev.assign(fd);
    waitTimer.expires_from_now(boost::posix_time::milliseconds(sensorPollMs));
    ;
    waitTimer.async_wait([&](const boost::system::error_code &ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        setupRead();
    });
}

void HwmonTempSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}

void HwmonTempSensor::updateValue(const double &newValue)
{
    sensorInterface->set_property("Value", newValue);
    value = newValue;
    checkThresholds();
}

void HwmonTempSensor::setInitialProperties(
    std::shared_ptr<sdbusplus::asio::connection> &conn)
{
    // todo, get max and min from configuration
    sensorInterface->register_property("MaxValue", maxValue);
    sensorInterface->register_property("MinValue", minValue);
    sensorInterface->register_property("Value", value);

    for (auto &threshold : thresholds)
    {
        std::shared_ptr<sdbusplus::asio::dbus_interface> iface;
        std::string level;
        std::string alarm;
        if (threshold.level == thresholds::Level::CRITICAL)
        {
            iface = thresholdInterfaceCritical;
            if (threshold.direction == thresholds::Direction::HIGH)
            {
                level = "CriticalHigh";
                alarm = "CriticalAlarmHigh";
            }
            else
            {
                level = "CriticalLow";
                alarm = "CriticalAlarmLow";
            }
        }
        else if (threshold.level == thresholds::Level::WARNING)
        {
            iface = thresholdInterfaceWarning;
            if (threshold.direction == thresholds::Direction::HIGH)
            {
                level = "WarningHigh";
                alarm = "WarningAlarmHigh";
            }
            else
            {
                level = "WarningLow";
                alarm = "WarningAlarmLow";
            }
        }
        else
        {
            std::cerr << "Unknown threshold level" << threshold.level << "\n";
            continue;
        }
        if (!iface)
        {
            std::cout << "trying to set uninitialized interface\n";
            continue;
        }
        iface->register_property(
            level, threshold.value,
            [&](const double &request, double &oldValue) {
                oldValue = request; // todo, just let the config do this?
                threshold.value = request;
                thresholds::persistThreshold(configuration, objectType,
                                             threshold, conn);
                return 1;
            });
        iface->register_property(alarm, false);
    }
    if (!sensorInterface->initialize())
    {
        std::cerr << "error initializing value interface\n";
    }
    if (thresholdInterfaceWarning && !thresholdInterfaceWarning->initialize())
    {
        std::cerr << "error initializing warning threshold interface\n";
    }

    if (thresholdInterfaceCritical && !thresholdInterfaceCritical->initialize())
    {
        std::cerr << "error initializing critical threshold interface\n";
    }
}
