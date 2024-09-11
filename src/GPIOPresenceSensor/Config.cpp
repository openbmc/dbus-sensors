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

#include "Config.hpp"

#include "../Utils.hpp"
#include "Interfaces.hpp"
#include "phosphor-logging/lg2.hpp"

#include <boost/asio.hpp>
#include <boost/container/flat_map.hpp>
#include <gpiod.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <string>

namespace gpio_presence_sensing
{

Config::Config() : detectedIface(std::nullopt) {}

static std::string getGpioInterface(unsigned int n)
{
    return std::format(
        "{}{}", interfaces::configurationGPIOHWDetectPresenceGpioInterface, n);
}

Config::Config(const sdbusplus::message::object_path& obj,
               const SensorData& item) : detectedIface(std::nullopt)
{
    (void)obj;
    auto found = item.find(interfaces::emConfigurationGPIOHWDetectInterface);
    const SensorBaseConfigMap& properties = found->second;

    name = loadVariant<std::string>(properties, properties::propertyName);

    // configure gpios
    for (unsigned int i = 0; true; i++)
    {
        GpioConfig gc;

        found = item.find(getGpioInterface(i));

        if (found == item.end())
        {
            if (i > 0)
            {
                // we already have 1 gpio
                break;
            }
            lg2::error("could not find gpio interface for device {NAME}",
                       "NAME", name);
            throw std::invalid_argument("did not find gpio interface");
        }

        const SensorBaseConfigMap& gpioProperties = found->second;

        auto gpioName =
            loadVariant<std::string>(gpioProperties, properties::propertyName);
        auto polarity = loadVariant<std::string>(gpioProperties,
                                                 properties::propertyPolarity);

        gc.gpioLine = gpioName;
        gc.activeLow = polarity == "Low";

        gpioConfigs[gpioName] = gc;
    }
}

bool Config::isPresent()
{
    bool present = true;
    for (const auto& [key, value] : gpioConfigs)
    {
        (void)key;
        if (!value.present)
        {
            present = false;
        }
    }
    return present;
}

} // namespace gpio_presence_sensing
