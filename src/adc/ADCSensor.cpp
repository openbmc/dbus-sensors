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

#include "ADCSensor.hpp"

#include "asio/Thresholds.hpp"
#include "asio/sensor.hpp"
#include "utils/SensorPaths.hpp"
#include "utils/Utils.hpp"

#include <fcntl.h>

#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/streambuf.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <istream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// scaling factor from hwmon
static constexpr unsigned int sensorScaleFactor = 1000;

static constexpr double roundFactor = 10000;     // 3 decimal places
static constexpr double maxVoltageReading = 1.8; // pre sensor scaling
static constexpr double minVoltageReading = 0;

ADCSensor::ADCSensor(
    const std::string& path, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const std::string& sensorName,
    std::vector<thresholds::Threshold>&& thresholdsIn, const double scaleFactor,
    const float pollRate, PowerState readState,
    const std::string& sensorConfiguration,
    std::optional<BridgeGpio>&& bridgeGpio) :
    Sensor(escapeName(sensorName), std::move(thresholdsIn), sensorConfiguration,
           "ADC", false, false, maxVoltageReading / scaleFactor,
           minVoltageReading / scaleFactor, conn, readState),
    objServer(objectServer), inputDev(io), waitTimer(io), path(path),
    scaleFactor(scaleFactor),
    sensorPollMs(static_cast<unsigned int>(pollRate * 1000)),
    bridgeGpio(std::move(bridgeGpio)), thresholdTimer(io)
{
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0)
    {
        lg2::error("unable to open acd device");
    }

    inputDev.assign(fd);

    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/voltage/" + name,
        "xyz.openbmc_project.Sensor.Value");
    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface(
                "/xyz/openbmc_project/sensors/voltage/" + name, interface);
    }
    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/voltage/" + name, association::interface);
    setInitialProperties(sensor_paths::unitVolts);
}

ADCSensor::~ADCSensor()
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

void ADCSensor::setupRead()
{
    if (!readingStateGood())
    {
        markAvailable(false);
        updateValue(std::numeric_limits<double>::quiet_NaN());
        restartRead();
        return;
    }

    std::shared_ptr<boost::asio::streambuf> buffer =
        std::make_shared<boost::asio::streambuf>();

    std::weak_ptr<ADCSensor> weakRef = weak_from_this();

    if (bridgeGpio.has_value())
    {
        (*bridgeGpio).set(1);
        // In case a channel has a bridge circuit,we have to turn the bridge on
        // prior to reading a value at least for one scan cycle to get a valid
        // value. Guarantee that the HW signal can be stable, the HW signal
        // could be instability.
        waitTimer.expires_after(
            std::chrono::milliseconds(bridgeGpio->setupTimeMs));
        waitTimer.async_wait(
            [weakRef, buffer](const boost::system::error_code& ec) {
                std::shared_ptr<ADCSensor> self = weakRef.lock();
                if (ec == boost::asio::error::operation_aborted)
                {
                    return; // we're being canceled
                }

                if (self)
                {
                    boost::asio::async_read_until(
                        self->inputDev, *buffer, '\n',
                        [weakRef, buffer](const boost::system::error_code& ec,
                                          std::size_t /*bytes_transfered*/) {
                            std::shared_ptr<ADCSensor> self = weakRef.lock();
                            if (self)
                            {
                                self->readBuf = buffer;
                                self->handleResponse(ec);
                            }
                        });
                }
            });
    }
    else
    {
        boost::asio::async_read_until(
            inputDev, *buffer, '\n',
            [weakRef, buffer](const boost::system::error_code& ec,
                              std::size_t /*bytes_transfered*/) {
                std::shared_ptr<ADCSensor> self = weakRef.lock();
                if (self)
                {
                    self->readBuf = buffer;
                    self->handleResponse(ec);
                }
            });
    }
}

void ADCSensor::restartRead()
{
    std::weak_ptr<ADCSensor> weakRef = weak_from_this();
    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        std::shared_ptr<ADCSensor> self = weakRef.lock();
        if (ec == boost::asio::error::operation_aborted)
        {
            if (self)
            {
                lg2::error("adcsensor '{NAME}' read cancelled", "NAME",
                           self->name);
            }
            else
            {
                lg2::error("adcsensor read cancelled no self");
            }
            return; // we're being canceled
        }

        if (self)
        {
            self->setupRead();
        }
        else
        {
            lg2::error("adcsensor weakref no self");
        }
    });
}

void ADCSensor::handleResponse(const boost::system::error_code& err)
{
    std::weak_ptr<ADCSensor> weakRef = weak_from_this();

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
            rawValue = std::stod(response);
            double nvalue = (rawValue / sensorScaleFactor) / scaleFactor;
            nvalue = std::round(nvalue * roundFactor) / roundFactor;
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
    if (bridgeGpio.has_value())
    {
        (*bridgeGpio).set(0);
    }

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0)
    {
        lg2::error("adcsensor '{NAME}' failed to open '{PATH}'", "NAME", name,
                   "PATH", path);
        return; // we're no longer valid
    }
    inputDev.assign(fd);
    restartRead();
}

void ADCSensor::checkThresholds()
{
    if (!readingStateGood())
    {
        return;
    }

    thresholds::checkThresholdsPowerDelay(weak_from_this(), thresholdTimer);
}
