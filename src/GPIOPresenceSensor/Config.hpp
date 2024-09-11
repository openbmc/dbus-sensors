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

#include "../Utils.hpp"

#include <boost/asio.hpp>
#include <boost/container/flat_map.hpp>
#include <gpiod.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <string>

namespace gpio_presence_sensing
{

struct GpioConfig
{
    std::string gpioLine;
    // GPIO Polarity.
    bool activeLow{};

    // is this gpio in the state indicating presence
    bool present;
};

class Config
{
  public:
    Config();
    Config(const sdbusplus::message::object_path& obj, const SensorData& item);

    // computed from the state of the configured gpios
    bool isPresent();

    // properties from the EM config:

    // name of the device to detect, e.g. 'cable0'
    std::string name;

    // maps the name of the gpio to its config
    std::map<std::string, GpioConfig> gpioConfigs;

    // properties added when the hw is detected
    std::optional<std::shared_ptr<sdbusplus::asio::dbus_interface>>
        detectedIface;
};

} // namespace gpio_presence_sensing
