/*
// Copyright (c) 2018 Intel Corporation
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

#include "PresenceSensor.hpp"

#include "Utils.hpp"

#include <unistd.h>

#include <boost/asio/read_until.hpp>
#include <gpiod.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <charconv>
#include <fstream>
#include <iostream>
#include <istream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

std::unordered_map<std::string, gpiod::line>
    PollingPresenceSensor::staticGpioMap;

EventPresenceSensor::EventPresenceSensor(const std::string& iSensorType,
                                         const std::string& gpioName,
                                         bool inverted,
                                         boost::asio::io_context& io,
                                         const std::string& iname) :
    gpioFd(io)
{
    gpioLine = gpiod::find_line(gpioName);
    name = iname;
    sensorType = iSensorType;
    if (!gpioLine)
    {
        std::cerr << "Error requesting gpio: " << gpioName << "\n";
        status = false;
        return;
    }

    try
    {
        gpioLine.request({sensorType + "Sensor",
                          gpiod::line_request::EVENT_BOTH_EDGES,
                          inverted ? gpiod::line_request::FLAG_ACTIVE_LOW : 0});
        tracePresence();

        int gpioLineFd = gpioLine.event_get_fd();
        if (gpioLineFd < 0)
        {
            std::cerr << "Failed to get " << gpioName << " fd\n";
            return;
        }

        gpioFd.assign(gpioLineFd);
    }
    catch (const std::system_error& e)
    {
        std::cerr << "Error reading gpio " << gpioName << ": " << e.what()
                  << "\n";
        status = false;
        return;
    }
}

void EventPresenceSensor::monitorPresence(void)
{
    std::weak_ptr<EventPresenceSensor> weakRef = weak_from_this();
    gpioFd.async_wait(boost::asio::posix::stream_descriptor::wait_read,
                      [weakRef](const boost::system::error_code& ec) {
        std::shared_ptr<EventPresenceSensor> self = weakRef.lock();
        if (!self)
        {
            std::cerr << "Failed to get lock for eventPresenceSensor: "
                      << ec.message() << "\n";
            return;
        }
        if (ec)
        {
            if (ec != boost::system::errc::bad_file_descriptor)
            {
                std::cerr << "Error on event presence sensor " << self->name
                          << ": " << ec.message() << "\n";
            }
            return;
        }
        self->read();
        self->monitorPresence();
    });
}

void EventPresenceSensor::read(void)
{
    // Read is invoked when an edge event is detected by monitorPresence
    gpioLine.event_read();
    tracePresence();
}

void EventPresenceSensor::tracePresence(void)
{
    status = (gpioLine.get_value() != 0);
    if (status)
    {
        logInserted(name);
    }
    else
    {
        logRemoved(name);
    }
}

void PollingPresenceSensor::initGpio(const std::string& gpioName, bool inverted)
{
    auto search = staticGpioMap.find(gpioName);
    if (search != staticGpioMap.end())
    {
        gpioLine = search->second;
        return;
    }

    gpioLine = gpiod::find_line(gpioName);
    if (!gpioLine)
    {
        std::cerr << "Unable to find gpio " << gpioName << " (polling)\n";
        status = false;
        return;
    }

    try
    {
        gpioLine.request({sensorType + "Sensor",
                          gpiod::line_request::DIRECTION_INPUT,
                          inverted ? gpiod::line_request::FLAG_ACTIVE_LOW : 0});
        status = (gpioLine.get_value() != 0);
    }
    catch (const std::system_error& e)
    {
        std::cerr << "Error reading gpio " << gpioName << ": " << e.what()
                  << " (polling)\n";
        status = false;
        return;
    }

    staticGpioMap.emplace(gpioName, gpioLine);
}

PollingPresenceSensor::PollingPresenceSensor(const std::string& iSensorType,
                                             const std::string& gpioName,
                                             bool inverted,
                                             boost::asio::io_context& io,
                                             const std::string& iname) :
    gpioName(gpioName),
    pollTimer(io)
{
    name = iname;
    sensorType = iSensorType;
    initGpio(gpioName, inverted);
}

void PollingPresenceSensor::monitorPresence(void)
{
    if (!gpioLine)
    {
        std::cerr << "monitorPresence encountered null gpioLine for " << name
                  << "\n";
        return;
    }

    int statusTry = gpioLine.get_value();
    if (static_cast<int>(status) != statusTry)
    {
        status = (statusTry != 0);
        if (status)
        {
            logInserted(name);
        }
        else
        {
            logRemoved(name);
        }
    }

    std::weak_ptr<PollingPresenceSensor> weakRef = weak_from_this();
    pollTimer.expires_after(std::chrono::seconds(1));
    pollTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        std::shared_ptr<PollingPresenceSensor> self = weakRef.lock();
        if (!self)
        {
            std::cerr << "Failed to get lock for pollingPresenceSensor: "
                      << ec.message() << "\n";
            return;
        }
        if (ec)
        {
            if (ec != boost::system::errc::bad_file_descriptor)
            {
                std::cerr << "GPIO polling timer failed for " << self->name
                          << ": " << ec.what() << ")\n";
            }
            return;
        }
        self->monitorPresence();
    });
}
