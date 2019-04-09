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

#include <ADCSensor.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <string>
static constexpr unsigned int sensorPollMs = 500;
static constexpr size_t warnAfterErrorCount = 10;
static constexpr unsigned int gpioBridgeEnableMs = 20;
// scaling factor from hwmon
static constexpr unsigned int sensorScaleFactor = 1000;

static constexpr double roundFactor = 10000; // 3 decimal places
static constexpr double maxReading = 20;
static constexpr double minReading = 0;
static constexpr const char* sysGpioPath = "/sys/class/gpio/gpio";
static constexpr const char* postfixValue = "/value";

void setGpio(int gpioN, int value)
{
    std::string device = sysGpioPath + std::to_string(gpioN) + postfixValue;
    std::fstream gpioFile;

    gpioFile.open(device, std::ios::out);

    if (!gpioFile.good())
    {
        std::cerr << "Error opening device " << device << "\n";
        return;
    }
    gpioFile << std::to_string(value);
    gpioFile.close();
}

ADCSensor::ADCSensor(const std::string& path,
                     sdbusplus::asio::object_server& objectServer,
                     std::shared_ptr<sdbusplus::asio::connection>& conn,
                     boost::asio::io_service& io, const std::string& sensorName,
                     std::vector<thresholds::Threshold>&& _thresholds,
                     const double scaleFactor, PowerState readState,
                     const std::string& sensorConfiguration,
                     std::optional<int> bridgeGpio) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"), path,
           std::move(_thresholds), sensorConfiguration,
           "xyz.openbmc_project.Configuration.ADC", maxReading, minReading),
    objServer(objectServer), scaleFactor(scaleFactor),
    readState(std::move(readState)), inputDev(io, open(path.c_str(), O_RDONLY)),
    waitTimer(io), errCount(0), thresholdTimer(io, this), bridgeGpio(bridgeGpio)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/voltage/" + name,
        "xyz.openbmc_project.Sensor.Value");
    if (thresholds::hasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/voltage/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Warning");
    }
    if (thresholds::hasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/voltage/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Critical");
    }
    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/voltage/" + name,
        "org.openbmc.Associations");
    setInitialProperties(conn);
    setupRead();

    // setup match
    setupPowerMatch(conn);
}

ADCSensor::~ADCSensor()
{
    // close the input dev to cancel async operations
    inputDev.close();
    waitTimer.cancel();
    objServer.remove_interface(thresholdInterfaceWarning);
    objServer.remove_interface(thresholdInterfaceCritical);
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

void ADCSensor::setupRead(void)
{
    if (bridgeGpio.has_value())
    {
        setGpio(*bridgeGpio, 1);
        // In case a channel has a bridge circuit,we have to turn the bridge on
        // prior to reading a value at least for one scan cycle to get a valid
        // value. Guarantee that the HW signal can be stable, the HW signal
        // could be instability.
        waitTimer.expires_from_now(
            boost::posix_time::milliseconds(gpioBridgeEnableMs));
        waitTimer.async_wait([&](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                return; // we're being canceled
            }
            boost::asio::async_read_until(
                inputDev, readBuf, '\n',
                [&](const boost::system::error_code& ec,
                    std::size_t /*bytes_transfered*/) { handleResponse(ec); });
        });
    }
    else
    {
        boost::asio::async_read_until(
            inputDev, readBuf, '\n',
            [&](const boost::system::error_code& ec,
                std::size_t /*bytes_transfered*/) { handleResponse(ec); });
    }
}

void ADCSensor::handleResponse(const boost::system::error_code& err)
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

        // todo read scaling factors from configuration
        try
        {
            double nvalue = std::stof(response);

            nvalue = (nvalue / sensorScaleFactor) / scaleFactor;
            nvalue = std::round(nvalue * roundFactor) / roundFactor;

            if (overridenState)
            {
                nvalue = overriddenValue;
            }

            if (nvalue != value)
            {
                updateValue(nvalue);
            }
            errCount = 0;
        }
        catch (std::invalid_argument)
        {
            errCount++;
        }
    }
    else
    {

        errCount++;
    }
    // only print once
    if (errCount == warnAfterErrorCount)
    {
        std::cerr << "Failure to read sensor " << name << " at " << path
                  << " ec:" << err << "\n";
    }

    if (errCount >= warnAfterErrorCount)
    {
        updateValue(0);
    }

    responseStream.clear();
    inputDev.close();
    if (bridgeGpio.has_value())
    {
        setGpio(*bridgeGpio, 0);
    }
    int fd = open(path.c_str(), O_RDONLY);
    if (fd <= 0)
    {
        return; // we're no longer valid
    }
    inputDev.assign(fd);
    waitTimer.expires_from_now(boost::posix_time::milliseconds(sensorPollMs));
    waitTimer.async_wait([&](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        setupRead();
    });
}

void ADCSensor::checkThresholds(void)
{
    if (readState == PowerState::on && !isPowerOn())
    {
        return;
    }

    thresholds::checkThresholdsPowerDelay(this, thresholdTimer);
}
