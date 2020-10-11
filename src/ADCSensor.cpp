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
#include <boost/asio/read_until.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

static constexpr unsigned int sensorPollMs = 500;
static constexpr size_t warnAfterErrorCount = 10;
static constexpr unsigned int gpioBridgeEnableMs = 20;
// scaling factor from hwmon
static constexpr unsigned int sensorScaleFactor = 1000;

static constexpr double roundFactor = 10000; // 3 decimal places
static constexpr double maxReading = 20;
static constexpr double minReading = 0;

ADCSensor::ADCSensor(const std::string& path,
                     sdbusplus::asio::object_server& objectServer,
                     std::shared_ptr<sdbusplus::asio::connection>& conn,
                     boost::asio::io_service& io, const std::string& sensorName,
                     std::vector<thresholds::Threshold>&& thresholds,
                     const double scaleFactor, PowerState readState,
                     const std::string& sensorConfiguration,
                     std::optional<BridgeGpio>&& bridgeGpio) :
    sensor(boost::replace_all_copy(sensorName, " ", "_"), std::move(thresholds),
           sensorConfiguration, "xyz.openbmc_project.Configuration.ADC",
           maxReading, minReading, conn, readState),
    objServer(objectServer), inputDev(io, open(path.c_str(), O_RDONLY)),
    waitTimer(io), path(path), scaleFactor(scaleFactor),
    bridgeGpio(std::move(bridgeGpio)), thresholdTimer(io, sensor)
{
    sensor.checkThresholdsFunc = [this]() { checkThresholds(); };
    sensor.sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/voltage/" + sensor.name,
        "xyz.openbmc_project.Sensor.Value");
    if (thresholds::hasWarningInterface(thresholds))
    {
        sensor.thresholdInterfaceWarning = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/voltage/" + sensor.name,
            "xyz.openbmc_project.Sensor.Threshold.Warning");
    }
    if (thresholds::hasCriticalInterface(thresholds))
    {
        sensor.thresholdInterfaceCritical = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/voltage/" + sensor.name,
            "xyz.openbmc_project.Sensor.Threshold.Critical");
    }
    sensor.association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/voltage/" + sensor.name,
        association::interface);

    sensor.setInitialProperties(conn);
}

ADCSensor::~ADCSensor()
{
    // close the input dev to cancel async operations
    inputDev.close();
    waitTimer.cancel();
    objServer.remove_interface(sensor.thresholdInterfaceWarning);
    objServer.remove_interface(sensor.thresholdInterfaceCritical);
    objServer.remove_interface(sensor.sensorInterface);
    objServer.remove_interface(sensor.association);
}

void ADCSensor::setupRead(void)
{
    std::shared_ptr<boost::asio::streambuf> buffer =
        std::make_shared<boost::asio::streambuf>();

    if (bridgeGpio.has_value())
    {
        (*bridgeGpio).set(1);
        // In case a channel has a bridge circuit,we have to turn the bridge on
        // prior to reading a value at least for one scan cycle to get a valid
        // value. Guarantee that the HW signal can be stable, the HW signal
        // could be instability.
        waitTimer.expires_from_now(
            boost::posix_time::milliseconds(gpioBridgeEnableMs));
        waitTimer.async_wait(
            [this, buffer](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return; // we're being canceled
                }

                boost::asio::async_read_until(
                    this->inputDev, *buffer, '\n',
                    [this, buffer](const boost::system::error_code& ec,
                                   std::size_t /*bytes_transfered*/) {
                        this->readBuf = buffer;
                        this->handleResponse(ec);
                    });
            });
    }
    else
    {
        boost::asio::async_read_until(
            inputDev, *buffer, '\n',
            [this, buffer](const boost::system::error_code& ec,
                           std::size_t /*bytes_transfered*/) {
                this->readBuf = buffer;
                this->handleResponse(ec);
            });
    }
}

void ADCSensor::handleResponse(const boost::system::error_code& err)
{
    if (err == boost::system::errc::bad_file_descriptor)
    {
        return; // we're being destroyed
    }
    std::istream responseStream(readBuf.get());

    if (!err)
    {
        std::string response;
        std::getline(responseStream, response);

        // todo read scaling factors from configuration
        try
        {
            sensor.rawValue = std::stod(response);
            double nvalue = (sensor.rawValue / sensorScaleFactor) / scaleFactor;
            nvalue = std::round(nvalue * roundFactor) / roundFactor;
            sensor.updateValue(nvalue);
        }
        catch (std::invalid_argument&)
        {
            sensor.incrementError();
        }
    }
    else
    {
        sensor.incrementError();
    }

    responseStream.clear();
    inputDev.close();
    if (bridgeGpio.has_value())
    {
        (*bridgeGpio).set(0);
    }
    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0)
    {
        std::cerr << "adcsensor " << sensor.name << " failed to open " << path
                  << "\n";
        return; // we're no longer valid
    }
    inputDev.assign(fd);
    waitTimer.expires_from_now(boost::posix_time::milliseconds(sensorPollMs));
    waitTimer.async_wait([this](const boost::system::error_code ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            std::cerr << "adcsensor " << this->sensor.name
                      << " read cancelled\n";

            return; // we're being canceled
        }

        this->setupRead();
    });
}

void ADCSensor::checkThresholds(void)
{
    if (!sensor.readingStateGood())
    {
        return;
    }

    thresholds::checkThresholdsPowerDelay(sensor, thresholdTimer);
}
