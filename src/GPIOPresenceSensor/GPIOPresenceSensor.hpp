/*
 * SPDX-FileCopyrightText: Copyright (c) 2022-2024. All rights
 * reserved. SPDX-License-Identifier: Apache-2.0
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
#pragma once

#include "Config.hpp"

#include <boost/asio.hpp>
#include <boost/container/flat_map.hpp>
#include <gpiod.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace gpio_presence_sensing
{

static const std::chrono::seconds pollRateDefault = std::chrono::seconds(5);

const std::string appName = "gpiopresencesensor";

const std::string service = "xyz.openbmc_project." + appName;

class GPIOPresence : public std::enable_shared_from_this<GPIOPresence>
{
  public:
    explicit GPIOPresence(
        const std::shared_ptr<sdbusplus::asio::connection>& bus,
        std::shared_ptr<sdbusplus::asio::object_server> objServer,
        boost::asio::io_context& io) :
        bus(bus), ioContext(io), objectServer(std::move(objServer)),
        timer(bus->get_io_context())
    {}

    void removeObj(const std::string& objPath);

    // returns true on success
    bool readPresentForAllConfigs();

    void startGPIOEventMonitor();

    // maps dbus object paths to configurations
    // e.g. /xyz/openbmc_project/inventory/system/board/My_Baseboard/cable0
    std::unordered_map<std::string, Config> configs;

    std::shared_ptr<sdbusplus::asio::connection> bus;

  private:
    boost::asio::io_context& ioContext;

    // returns true on success
    bool readPresentSingleGPIO(GpioConfig& config);

    void updateHWDetectedInterface(Config& config);

    void updatePresenceForConfig(const std::string& gpioLine, bool state,
                                 Config& config);
    void updatePresence(const std::string& gpioLine, bool state);

    void waitForGPIOEvent(const std::string& name,
                          const std::function<void(bool)>& eventHandler);

    bool addInputLine(const std::string& lineLabel);

    int readLine(const std::string& lineLabel);

    void releaseLine(const std::string& lineLabel);

    // Start to request GPIO events
    bool requestGPIOEvents(const std::string& name,
                           const std::function<void(bool)>& handler);

    // dbus connection.
    std::shared_ptr<sdbusplus::asio::object_server> objectServer;
    boost::asio::steady_timer timer;

    // Reference to gpioLines.
    // this maps gpio names to gpio lines
    std::unordered_map<std::string, gpiod::line> gpioLines;
    std::unordered_map<std::string,
                       std::shared_ptr<boost::asio::posix::stream_descriptor>>
        linesSD;
};

} // namespace gpio_presence_sensing
