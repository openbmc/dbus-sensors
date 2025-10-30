// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright 2018 Intel Corporation

#include "PresenceGpio.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <gpiod.hpp>
#include <phosphor-logging/lg2.hpp>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>

static constexpr unsigned int pollIntervalSec = 1;

PresenceGpio::PresenceGpio(const std::string& deviceType,
                           const std::string& deviceName,
                           const std::string& gpioName) :
    deviceType(deviceType), deviceName(deviceName), gpioName(gpioName)
{
    gpioLine = gpiod::find_line(gpioName);
    if (!gpioLine)
    {
        lg2::error("Error requesting gpio: '{NAME}'", "NAME", gpioName);
        throw std::runtime_error("Failed to find GPIO " + gpioName);
    }
}

PresenceGpio::~PresenceGpio()
{
    gpioLine.release();
}

void PresenceGpio::updateAndTracePresence(int newValue)
{
    status = (newValue != 0);
    if (status)
    {
        logPresent(deviceName);
    }
    else
    {
        logRemoved(deviceName);
    }
}

EventPresenceGpio::EventPresenceGpio(
    const std::string& deviceType, const std::string& deviceName,
    const std::string& gpioName, bool inverted, boost::asio::io_context& io) :
    PresenceGpio(deviceType, deviceName, gpioName), gpioFd(io)
{
    try
    {
        gpioLine.request(
            {deviceType + "Sensor", gpiod::line_request::EVENT_BOTH_EDGES,
             inverted ? gpiod::line_request::FLAG_ACTIVE_LOW : 0});
        updateAndTracePresence(gpioLine.get_value());
    }
    catch (const std::system_error& e)
    {
        lg2::error("Error reading gpio '{NAME}': '{ERR}'", "NAME", gpioName,
                   "ERR", e);
        throw std::runtime_error("Failed to read GPIO fd " + gpioName);
    }

    int gpioLineFd = gpioLine.event_get_fd();
    if (gpioLineFd < 0)
    {
        lg2::error("Failed to get '{NAME}' fd", "NAME", gpioName);
        throw std::runtime_error("Failed to get GPIO fd " + gpioName);
    }

    gpioFd.assign(gpioLineFd);
}

void EventPresenceGpio::monitorPresence()
{
    std::weak_ptr<EventPresenceGpio> weakRef = weak_from_this();
    gpioFd.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [weakRef](const boost::system::error_code& ec) {
            std::shared_ptr<EventPresenceGpio> self = weakRef.lock();
            if (!self)
            {
                lg2::error(
                    "Failed to get lock for eventPresenceGpio: '{ERROR_MESSAGE}'",
                    "ERROR_MESSAGE", ec.message());
                return;
            }
            if (ec)
            {
                if (ec != boost::system::errc::bad_file_descriptor)
                {
                    lg2::error(
                        "Error on event presence device '{NAME}': '{ERROR_MESSAGE}'",
                        "NAME", self->deviceName, "ERROR_MESSAGE",
                        ec.message());
                }
                return;
            }
            self->read();
            self->monitorPresence();
        });
}

void EventPresenceGpio::read()
{
    // Read is invoked when an edge event is detected by monitorPresence
    gpioLine.event_read();
    updateAndTracePresence(gpioLine.get_value());
}

PollingPresenceGpio::PollingPresenceGpio(
    const std::string& deviceType, const std::string& deviceName,
    const std::string& gpioName, bool inverted, boost::asio::io_context& io) :
    PresenceGpio(deviceType, deviceName, gpioName), pollTimer(io)
{
    try
    {
        gpioLine.request(
            {deviceType + "Sensor", gpiod::line_request::DIRECTION_INPUT,
             inverted ? gpiod::line_request::FLAG_ACTIVE_LOW : 0});
        updateAndTracePresence(gpioLine.get_value());
    }
    catch (const std::system_error& e)
    {
        lg2::error("PollingPresenceGpio: Error reading gpio '{NAME}': '{ERR}'",
                   "NAME", gpioName, "ERR", e);
        status = false;
        throw std::runtime_error("Failed to get Polling GPIO fd " + gpioName);
    }
}

inline void PollingPresenceGpio::pollTimerHandler(
    const std::weak_ptr<PollingPresenceGpio>& weakRef,
    const boost::system::error_code& ec)
{
    std::shared_ptr<PollingPresenceGpio> self = weakRef.lock();
    if (!self)
    {
        lg2::error(
            "Failed to get lock for pollingPresenceGpio: '{ERROR_MESSAGE}'",
            "ERROR_MESSAGE", ec.message());
        return;
    }
    if (ec)
    {
        if (ec != boost::system::errc::bad_file_descriptor)
        {
            lg2::error(
                "GPIO polling timer failed for '{NAME}': '{ERROR_MESSAGE}'",
                "NAME", self->gpioName, "ERROR_MESSAGE", ec.message());
        }
        return;
    }
    self->monitorPresence();
}

void PollingPresenceGpio::monitorPresence()
{
    // Determine if the value has changed
    int newStatus = gpioLine.get_value();
    if (static_cast<int>(status) != newStatus)
    {
        updateAndTracePresence(newStatus);
    }

    std::weak_ptr<PollingPresenceGpio> weakRef = weak_from_this();
    pollTimer.expires_after(std::chrono::seconds(pollIntervalSec));
    pollTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        pollTimerHandler(weakRef, ec);
    });
}
