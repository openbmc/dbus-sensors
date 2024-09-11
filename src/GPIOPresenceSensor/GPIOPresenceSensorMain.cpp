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

#include "../Utils.hpp"
#include "Config.hpp"
#include "GPIOPresenceSensor.hpp"
#include "Interfaces.hpp"

#include <phosphor-logging/lg2.hpp>

#include <format>

namespace gpio_presence_sensing
{
std::unique_ptr<sdbusplus::bus::match::match> ifcAdded;
std::unique_ptr<sdbusplus::bus::match::match> ifcRemoved;

using OnInterfaceAddedCallback =
    std::function<void(std::string_view, const Config&)>;
using OnInterfaceRemovedCallback = std::function<void(std::string_view)>;

void startMain(
    const std::shared_ptr<gpio_presence_sensing::GPIOPresence>& controller)
{
    static boost::asio::steady_timer timer(controller->bus->get_io_context());

    bool success = controller->readPresentForAllConfigs();

    if (!success)
    {
        lg2::error("could not readPresent value of gpios");
        return;
    }

    timer.cancel();
    // 5 seconds
    timer.expires_after(std::chrono::seconds(5));
    timer.async_wait([controller](const boost::system::error_code ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            lg2::warning("Delaying update loop");
            return;
        }
        controller->startGPIOEventMonitor();
        lg2::debug("Update loop started");
    });
}

void setupConfigurationSingleDevice(
    const sdbusplus::message::object_path& obj, const SensorData& item,
    const std::shared_ptr<gpio_presence_sensing::GPIOPresence>& controller)
{
    auto found = item.find(interfaces::emConfigurationGPIOHWDetectInterface);
    if (found == item.end())
    {
        return;
    }

    try
    {
        Config config(obj, item);

        controller->configs[std::string(obj.str)] = std::move(config);

        lg2::debug("found valid configuration at object path {OBJPATH}",
                   "OBJPATH", obj.str);

        startMain(controller);
    }
    catch (std::exception& e)
    {
        lg2::error("Incomplete config found: {ERR} obj = {OBJPATH}", "ERR",
                   e.what(), "OBJPATH", obj.str);
        return;
    }
}

void setupConfiguration(
    const boost::system::error_code ec, const ManagedObjectType& managedObjs,
    const std::shared_ptr<gpio_presence_sensing::GPIOPresence>& controller)
{
    if (ec)
    {
        lg2::error("error calling 'GetManagedObjects': {MSG}", "MSG",
                   ec.message());
        return;
    }
    for (const auto& obj : managedObjs)
    {
        sdbusplus::message::object_path path = obj.first;
        const SensorData& second = obj.second;
        setupConfigurationSingleDevice(path, second, controller);
    }
}

void setupInterfaceAdded(
    const std::shared_ptr<gpio_presence_sensing::GPIOPresence>& controller)
{
    lg2::debug("setting up dbus match for interfaces added...");

    const std::shared_ptr<sdbusplus::asio::connection>& conn = controller->bus;

    std::function<void(sdbusplus::message::message & msg)> handler =
        [controller](sdbusplus::message::message& msg) {
            sdbusplus::message::object_path objPath;
            SensorData ifcAndProperties;
            msg.read(objPath, ifcAndProperties);

            auto managedObjs = std::make_shared<ManagedObjectType>();

            (*managedObjs)[objPath] = ifcAndProperties;

            setupConfiguration(boost::system::error_code(), *managedObjs,
                               controller);
        };

    lg2::debug("calling 'GetManagedObjects' to see existing interfaces...");
    // call the user callback for all the device that is already available
    conn->async_method_call(
        [controller](const boost::system::error_code ec,
                     const ManagedObjectType& managedObjs) {
            setupConfiguration(ec, managedObjs, controller);
        },
        "xyz.openbmc_project.EntityManager", "/xyz/openbmc_project/inventory",
        "org.freedesktop.DBus.ObjectManager", "GetManagedObjects");

    ifcAdded = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*conn),
        sdbusplus::bus::match::rules::interfacesAdded() +
            sdbusplus::bus::match::rules::sender(
                "xyz.openbmc_project.EntityManager"),
        handler);

    lg2::debug("done setting up dbus match for interfacess added");
}

void setupInterfaceRemoved(
    const std::shared_ptr<gpio_presence_sensing::GPIOPresence>& controller)
{
    // Listen to the interface removed event.
    std::function<void(sdbusplus::message::message & msg)> handler =
        [controller](sdbusplus::message::message msg) {
            sdbusplus::message::object_path objPath;
            msg.read(objPath);

            lg2::debug("detected interface removed on {OBJPATH}", "OBJPATH",
                       objPath);
            controller->removeObj(objPath);
        };
    const std::shared_ptr<sdbusplus::asio::connection>& conn = controller->bus;
    ifcRemoved = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*conn),
        sdbusplus::bus::match::rules::interfacesRemoved() +
            sdbusplus::bus::match::rules::sender(
                "xyz.openbmc_project.EntityManager"),
        handler);
}

} // namespace gpio_presence_sensing

void setupInterfaceAddedRemovedHandlers(
    const std::shared_ptr<gpio_presence_sensing::GPIOPresence>& controller)
{
    gpio_presence_sensing::setupInterfaceAdded(controller);

    gpio_presence_sensing::setupInterfaceRemoved(controller);
}

int main()
{
    lg2::debug("starting GPIO Presence Sensor");

    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);

    systemBus->request_name(gpio_presence_sensing::service.c_str());

    auto objServerShared =
        std::make_shared<sdbusplus::asio::object_server>(systemBus);

    std::shared_ptr<gpio_presence_sensing::GPIOPresence> controller =
        std::make_shared<gpio_presence_sensing::GPIOPresence>(
            systemBus, objServerShared, io);

    setupInterfaceAddedRemovedHandlers(controller);

    io.run();
}
