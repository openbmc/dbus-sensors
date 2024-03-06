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

#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <gpiod.hpp>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>

PresenceSensor::~PresenceSensor()
{
    gpioLine.release();
}

void PresenceSensor::updateAndTracePresence()
{
    status = (gpioLine.get_value() != 0);
    if (status)
    {
        logPresent(sensorName);
    }
    else
    {
        logRemoved(sensorName);
    }
}

EventPresenceSensor::EventPresenceSensor(
    const std::string& iSensorType, const std::string& iSensorName,
    const std::string& gpioName, bool inverted, boost::asio::io_context& io) :
    PresenceSensor(iSensorType, iSensorName), gpioFd(io)
{
    gpioLine = gpiod::find_line(gpioName);
    if (!gpioLine)
    {
        std::cerr << "Error requesting gpio: " << gpioName << "\n";
        return;
    }

    try
    {
        gpioLine.request(
            {sensorType + "Sensor", gpiod::line_request::EVENT_BOTH_EDGES,
             inverted ? gpiod::line_request::FLAG_ACTIVE_LOW : 0});
        updateAndTracePresence();

        int gpioLineFd = gpioLine.event_get_fd();
        if (gpioLineFd < 0)
        {
            std::cerr << "Failed to get " << gpioName << " fd\n";
            throw std::runtime_error("Failed to get GPIO fd " + gpioName);
        }

        gpioFd.assign(gpioLineFd);
    }
    catch (const std::system_error& e)
    {
        std::cerr << "Error reading gpio " << gpioName << ": " << e.what()
                  << "\n";
        return;
    }

    monitorPresence();
}

void EventPresenceSensor::monitorPresence()
{
    std::weak_ptr<EventPresenceSensor> weakRef = weak_from_this();
    gpioFd.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
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
                    std::cerr
                        << "Error on event presence sensor " << self->sensorName
                        << ": " << ec.message() << "\n";
                }
                return;
            }
            self->read();
            self->monitorPresence();
        });
}

void EventPresenceSensor::read()
{
    // Read is invoked when an edge event is detected by monitorPresence
    gpioLine.event_read();
    updateAndTracePresence();
}
