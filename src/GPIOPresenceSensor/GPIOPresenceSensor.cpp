/*
 * SPDX-FileCopyrightText: Copyright (c) 2022-2024.
 * All rights reserved. SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "GPIOPresenceSensor.hpp"

#include "Config.hpp"
#include "Interfaces.hpp"

#include <boost/asio/posix/stream_descriptor.hpp>
#include <gpiod.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cerrno>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>

namespace gpio_presence_sensing
{

void GPIOPresence::removeObj(const std::string& objPath)
{
    if (!configs.contains(objPath))
    {
        return;
    }

    lg2::debug("erasing configuration for object path {OBJPATH}", "OBJPATH",
               objPath);
    configs.erase(objPath);
}

void GPIOPresence::waitForGPIOEvent(
    const std::string& name, const std::function<void(bool)>& eventHandler)
{
    if (!linesSD.contains(name))
    {
        lg2::error(
            "event stream descriptor map did not contain stream for {GPIO_NAME}",
            "GPIO_NAME", name);
        return;
    }
    auto event = linesSD[name];

    event->async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [ref{this}, &name, eventHandler,
         this](const boost::system::error_code ec) {
            if (ec)
            {
                lg2::error("{NAME} fd handler error: {MSG} Error", "NAME", name,
                           "MSG", ec.message());
                return;
            }
            if (!gpioLines.contains(name))
            {
                lg2::error("gpio line map did not contain {GPIO_NAME}",
                           "GPIO_NAME", name);
                return;
            }
            gpiod::line& line = gpioLines[name];
            gpiod::line_event lineEvent = line.event_read();
            const bool gpioState =
                (lineEvent.event_type == gpiod::line_event::RISING_EDGE);

            eventHandler(gpioState);
            ref->waitForGPIOEvent(name, eventHandler);
        });
}

bool GPIOPresence::requestGPIOEvents(const std::string& name,
                                     const std::function<void(bool)>& handler)
{
    lg2::debug("requesting gpio events for gpio {NAME}", "NAME", name);

    gpiod::line gpioLine = gpiod::find_line(name);

    if (!gpioLine)
    {
        lg2::error("Failed to find the {GPIO_NAME} line", "GPIO_NAME", name);
        return false;
    }

    std::shared_ptr<boost::asio::posix::stream_descriptor> gpioEventDescriptor =
        std::make_shared<boost::asio::posix::stream_descriptor>(ioContext);

    gpioLines[name] = gpioLine;
    linesSD[name] = gpioEventDescriptor;

    try
    {
        gpioLine.request({appName, gpiod::line_request::EVENT_BOTH_EDGES, {}});
    }
    catch (const std::exception& e)
    {
        lg2::error("Failed to request events for {GPIO_NAME}: Error {MSG}",
                   "GPIO_NAME", name, "MSG", e.what());
        return false;
    }
    int gpioLineFd = gpioLine.event_get_fd();
    if (gpioLineFd < 0)
    {
        lg2::error("Failed to get {GPIO_NAME} line", "GPIO_NAME", name);
        return false;
    }

    gpioEventDescriptor->assign(gpioLineFd);

    waitForGPIOEvent(name, handler);
    return true;
}

void GPIOPresence::updateHWDetectedInterface(Config& config)
{
    lg2::debug("updating dbus interface for config {OBJPATH}", "OBJPATH",
               config.name);

    if (!config.isPresent() && config.detectedIface.has_value())
    {
        lg2::info("detected {NAME} as absent", "NAME", config.name);

        lg2::debug("removing dbus interface");

        objectServer->remove_interface(config.detectedIface.value());
        config.detectedIface = std::nullopt;
        return;
    }

    if (!config.isPresent())
    {
        return;
    }

    if (config.detectedIface.has_value())
    {
        lg2::debug("interface was already created, nothing to do");
        return;
    }

    lg2::info("detected {NAME} as present", "NAME", config.name);

    sdbusplus::message::object_path objPathBase(
        "/xyz/openbmc_project/GPIODeviceDetected/");
    sdbusplus::message::object_path objPath = objPathBase / config.name;

    config.detectedIface = objectServer->add_interface(
        objPath, interfaces::configurationGPIODeviceDetectedInterface);

    config.detectedIface.value()->register_property("Name", config.name);
    config.detectedIface.value()->initialize();

    lg2::debug("created dbus path {PATH}", "PATH", objPath);
}

void GPIOPresence::updatePresenceForConfig(const std::string& gpioLine,
                                           bool state, Config& config)
{
    if (!config.gpioConfigs.contains(gpioLine))
    {
        lg2::error("not updating presence for unrecognized gpio {GPIO_NAME}",
                   "GPIO_NAME", gpioLine);
        return;
    }

    GpioConfig& gc = config.gpioConfigs[gpioLine];

    const bool present = gc.activeLow ? !state : state;

    if (present == gc.present)
    {
        return;
    }

    gc.present = present;

    lg2::debug(
        "gpioLine {GPIO_NAME} went {GPIO_LEVEL}, meaning {CONNECT_STATUS}",
        "GPIO_NAME", gc.gpioLine, "GPIO_LEVEL", (state) ? "high" : "low",
        "CONNECT_STATUS", (present) ? "connected" : "disconnected");

    updateHWDetectedInterface(config);
}

void GPIOPresence::updatePresence(const std::string& gpioLine, bool state)
{
    for (auto& [objpath, config] : configs)
    {
        updatePresenceForConfig(gpioLine, state, config);
    }
}

// returns true on success
bool GPIOPresence::addInputLine(const std::string& lineLabel)
{
    if (gpioLines.find(lineLabel) != gpioLines.end())
    {
        return true;
    }

    gpiod::line line = ::gpiod::find_line(lineLabel);

    if (!line)
    {
        lg2::error("Failed to find line {GPIO_NAME}", "GPIO_NAME", lineLabel);
        return false;
    }
    line.request({service, ::gpiod::line_request::DIRECTION_INPUT,
                  /*default_val=*/0});
    gpioLines[lineLabel] = line;

    return true;
}

// returns -1 on failure
int GPIOPresence::readLine(const std::string& lineLabel)
{
    if (gpioLines.find(lineLabel) == gpioLines.end())
    {
        bool success = addInputLine(lineLabel);
        if (!success)
        {
            return -1;
        }
    }
    return gpioLines[lineLabel].get_value();
}

void GPIOPresence::releaseLine(const std::string& lineLabel)
{
    if (gpioLines.find(lineLabel) == gpioLines.end())
    {
        return;
    }

    ::gpiod::line line = ::gpiod::find_line(lineLabel);
    line.release();
    gpioLines.erase(lineLabel);
}

bool GPIOPresence::readPresentSingleGPIO(GpioConfig& config)
{
    int lineValue = 0;
    try
    {
        lineValue = readLine(config.gpioLine);
        if (lineValue < 0)
        {
            lg2::error("Failed gpio line read {GPIO_NAME}", "GPIO_NAME",
                       config.gpioLine);
            return false;
        }
        releaseLine(config.gpioLine);
    }
    catch (const std::invalid_argument& e)
    {
        lg2::error("Failed gpio line read {GPIO_NAME} error is: {MSG}",
                   "GPIO_NAME", config.gpioLine, "MSG", e.what());
        return false;
    }
    catch (const std::runtime_error& e)
    {
        lg2::error("Failed gpio line read {GPIO_NAME} error is: {MSG}",
                   "GPIO_NAME", config.gpioLine, "MSG", e.what());
        return false;
    }

    updatePresence(config.gpioLine, lineValue == 1);

    return true;
}

// returns true on success
bool GPIOPresence::readPresentForAllConfigs()
{
    lg2::debug(
        "initially read of all configured gpios to check device presence");

    for (auto& [objpath, config] : configs)
    {
        for (auto& [key, value] : config.gpioConfigs)
        {
            bool success = readPresentSingleGPIO(value);
            if (!success)
            {
                return false;
            }
        }
    }
    return true;
}

void GPIOPresence::startGPIOEventMonitor()
{
    for (auto& [objpath, config] : configs)
    {
        for (auto& [key, gpioConfig] : config.gpioConfigs)
        {
            (void)key;
            bool success = requestGPIOEvents(
                gpioConfig.gpioLine,

                [self(shared_from_this()), gpioLine{gpioConfig.gpioLine}](
                    bool state) { self->updatePresence(gpioLine, state); });

            if (!success)
            {
                lg2::error("Failed to start GPIO Event Monitor");
                return;
            }
        }
    }
}
} // namespace gpio_presence_sensing
