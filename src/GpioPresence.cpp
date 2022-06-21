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

#include "GpioPresence.hpp"

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

std::unordered_map<std::string, gpiod::line> PSensor::staticGpioMap;

PresenceSensor::PresenceSensor(const std::string& gpioName, bool inverted,
                               boost::asio::io_context& io,
                               const std::string& name) :
    gpioLine(gpiod::find_line(gpioName)),
    gpioFd(io), name(name)
{
    if (!gpioLine)
    {
        std::cerr << "Error requesting gpio: " << gpioName << "\n";
        status = false;
        return;
    }

    try
    {
        gpioLine.request({"FanSensor", gpiod::line_request::EVENT_BOTH_EDGES,
                          inverted ? gpiod::line_request::FLAG_ACTIVE_LOW : 0});
        status = (gpioLine.get_value() != 0);

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

PresenceSensor::~PresenceSensor()
{
    gpioFd.close();
    gpioLine.release();
}

void PresenceSensor::monitorPresence(void)
{
    gpioFd.async_wait(boost::asio::posix::stream_descriptor::wait_read,
                      [this](const boost::system::error_code& ec) {
        if (ec == boost::system::errc::bad_file_descriptor)
        {
            return; // we're being destroyed
        }
        if (ec)
        {
            std::cerr << "Error on presence sensor " << name << " \n";
            ;
        }
        else
        {
            read();
        }
        monitorPresence();
    });
}

void PresenceSensor::read(void)
{
    gpioLine.event_read();
    status = (gpioLine.get_value() != 0);
    // Read is invoked when an edge event is detected by monitorPresence
    if (status)
    {
        logFanInserted(name);
    }
    else
    {
        logFanRemoved(name);
    }
}

bool PresenceSensor::getValue(void)
{
    return status;
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
        fanInserted = false;
        return;
    }

    try
    {
        gpioLine.request({"FanSensor", gpiod::line_request::DIRECTION_INPUT,
                          inverted ? gpiod::line_request::FLAG_ACTIVE_LOW : 0});
        fanInserted = (gpioLine.get_value() != 0);
    }
    catch (const std::system_error& e)
    {
        std::cerr << "Error reading gpio " << gpioName << ": " << e.what()
                  << " (polling)\n";
        fanInserted = false;
        return;
    }

    staticGpioMap.emplace(gpioName, gpioLine);
}

PollingPresenceSensor::PollingPresenceSensor(const std::string& gpioName,
                                             bool inverted,
                                             boost::asio::io_context& io,
                                             const std::string& name) :
    name(name),
    gpioName(gpioName), repeatTimer(io)
{
    initGpio(gpioName, inverted);
}

PollingPresenceSensor::~PollingPresenceSensor()
{
    gpioLine.release();
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
    if (static_cast<int>(fanInserted) != statusTry)
    {
        fanInserted = (statusTry != 0);
        if (fanInserted)
        {
            logFanInserted(name);
        }
        else
        {
            logFanRemoved(name);
        }
    }

    std::weak_ptr<PollingPresenceSensor> weakRef = weak_from_this();
    repeatTimer.expires_after(std::chrono::seconds(1));
    repeatTimer.async_wait([weakRef](const boost::system::error_code& error) {
        std::shared_ptr<PollingPresenceSensor> self = weakRef.lock();
        if (self)
        {
            if (!error)
            {
                self->monitorPresence();
            }
            else
            {
                std::cerr << "GPIO polling timer failed for " << self->name
                          << " (" << error.what() << ")\n";
            }
        }
    });
}

bool PollingPresenceSensor::getValue(void)
{
    return fanInserted;
}
