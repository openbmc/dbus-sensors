/*
// Copyright (c) 2019 Intel Corporation
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

#include "MCUTempSensor.hpp"

#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

extern "C"
{
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

constexpr const bool debug = false;

constexpr const char* sensorType = "MCUTempSensor";
static constexpr double mcuTempMaxReading = 0xFF;
static constexpr double mcuTempMinReading = 0;

boost::container::flat_map<std::string, std::unique_ptr<MCUTempSensor>> sensors;

MCUTempSensor::MCUTempSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                             boost::asio::io_context& io,
                             const std::string& sensorName,
                             const std::string& sensorConfiguration,
                             sdbusplus::asio::object_server& objectServer,
                             std::vector<thresholds::Threshold>&& thresholdData,
                             uint8_t busId, uint8_t mcuAddress,
                             uint8_t tempReg) :
    Sensor(escapeName(sensorName), std::move(thresholdData),
           sensorConfiguration, "MCUTempSensor", false, false,
           mcuTempMaxReading, mcuTempMinReading, conn),
    busId(busId), mcuAddress(mcuAddress), tempReg(tempReg),
    objectServer(objectServer), waitTimer(io)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface(
                "/xyz/openbmc_project/sensors/temperature/" + name, interface);
    }
    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        association::interface);
}

MCUTempSensor::~MCUTempSensor()
{
    waitTimer.cancel();
    for (const auto& iface : thresholdInterfaces)
    {
        objectServer.remove_interface(iface);
    }
    objectServer.remove_interface(sensorInterface);
    objectServer.remove_interface(association);
}

void MCUTempSensor::init(void)
{
    setInitialProperties(sensor_paths::unitDegreesC);
    read();
}

void MCUTempSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}

int MCUTempSensor::getMCURegsInfoWord(uint8_t regs, int16_t* pu16data) const
{
    std::string i2cBus = "/dev/i2c-" + std::to_string(busId);

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    int fd = open(i2cBus.c_str(), O_RDWR);
    if (fd < 0)
    {
        std::cerr << " unable to open i2c device" << i2cBus << "  err=" << fd
                  << "\n";
        return -1;
    }

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    if (ioctl(fd, I2C_SLAVE_FORCE, mcuAddress) < 0)
    {
        std::cerr << " unable to set device address\n";
        close(fd);
        return -1;
    }

    unsigned long funcs = 0;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    if (ioctl(fd, I2C_FUNCS, &funcs) < 0)
    {
        std::cerr << " not support I2C_FUNCS\n";
        close(fd);
        return -1;
    }

    if ((funcs & I2C_FUNC_SMBUS_READ_WORD_DATA) == 0U)
    {
        std::cerr << " not support I2C_FUNC_SMBUS_READ_WORD_DATA\n";
        close(fd);
        return -1;
    }

    *pu16data = i2c_smbus_read_word_data(fd, regs);
    close(fd);

    if (*pu16data < 0)
    {
        std::cerr << " read word data failed at " << static_cast<int>(regs)
                  << "\n";
        return -1;
    }

    return 0;
}

void MCUTempSensor::read(void)
{
    static constexpr size_t pollTime = 1; // in seconds

    waitTimer.expires_after(std::chrono::seconds(pollTime));
    waitTimer.async_wait([this](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being cancelled
        }
        // read timer error
        if (ec)
        {
            std::cerr << "timer error\n";
            return;
        }
        int16_t temp = 0;
        int ret = getMCURegsInfoWord(tempReg, &temp);
        if (ret >= 0)
        {
            double v = static_cast<double>(temp) / 1000;
            if constexpr (debug)
            {
                std::cerr << "Value update to " << v << "raw reading "
                          << static_cast<int>(temp) << "\n";
            }
            updateValue(v);
        }
        else
        {
            std::cerr << "Invalid read getMCURegsInfoWord\n";
            incrementError();
        }
        read();
    });
}

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::unique_ptr<MCUTempSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    if (!dbusConnection)
    {
        std::cerr << "Connection not created\n";
        return;
    }

    dbusConnection->async_method_call(
        [&io, &objectServer, &dbusConnection, &sensors](
            boost::system::error_code ec, const ManagedObjectType& resp) {
        if (ec)
        {
            std::cerr << "Error contacting entity manager\n";
            return;
        }
        for (const auto& [path, interfaces] : resp)
        {
            for (const auto& [intf, cfg] : interfaces)
            {
                if (intf != configInterfaceName(sensorType))
                {
                    continue;
                }
                std::string name = loadVariant<std::string>(cfg, "Name");

                std::vector<thresholds::Threshold> sensorThresholds;
                if (!parseThresholdsFromConfig(interfaces, sensorThresholds))
                {
                    std::cerr << "error populating thresholds for " << name
                              << "\n";
                }

                uint8_t busId = loadVariant<uint8_t>(cfg, "Bus");
                uint8_t mcuAddress = loadVariant<uint8_t>(cfg, "Address");
                uint8_t tempReg = loadVariant<uint8_t>(cfg, "Reg");

                std::string sensorClass =
                    loadVariant<std::string>(cfg, "Class");

                if constexpr (debug)
                {
                    std::cerr << "Configuration parsed for \n\t" << intf << "\n"
                              << "with\n"
                              << "\tName: " << name << "\n"
                              << "\tBus: " << static_cast<int>(busId) << "\n"
                              << "\tAddress: " << static_cast<int>(mcuAddress)
                              << "\n"
                              << "\tReg: " << static_cast<int>(tempReg) << "\n"
                              << "\tClass: " << sensorClass << "\n";
                }

                auto& sensor = sensors[name];

                sensor = std::make_unique<MCUTempSensor>(
                    dbusConnection, io, name, path, objectServer,
                    std::move(sensorThresholds), busId, mcuAddress, tempReg);

                sensor->init();
            }
        }
        },
        entityManagerName, "/xyz/openbmc_project/inventory",
        "org.freedesktop.DBus.ObjectManager", "GetManagedObjects");
}

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");

    systemBus->request_name("xyz.openbmc_project.MCUTempSensor");

    boost::asio::post(
        io, [&]() { createSensors(io, objectServer, sensors, systemBus); });

    boost::asio::steady_timer configTimer(io);

    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&](sdbusplus::message_t&) {
        configTimer.expires_after(std::chrono::seconds(1));
        // create a timer because normally multiple properties change
        configTimer.async_wait([&](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                return; // we're being canceled
            }
            // config timer error
            if (ec)
            {
                std::cerr << "timer error\n";
                return;
            }
            createSensors(io, objectServer, sensors, systemBus);
            if (sensors.empty())
            {
                std::cout << "Configuration not detected\n";
            }
        });
    };

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(
            *systemBus, std::to_array<const char*>({sensorType}), eventHandler);
    setupManufacturingModeMatch(*systemBus);
    io.run();
    return 0;
}
