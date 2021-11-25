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
#include <MCUTempSensor.hpp>
#include <Utils.hpp>
#include <VariantVisitors.hpp>
#include <boost/algorithm/string/predicate.hpp>
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

constexpr const char* configInterface =
    "xyz.openbmc_project.Configuration.MCUTempSensor";
static constexpr double mcuTempMaxReading = 0xFF;
static constexpr double mcuTempMinReading = 0;

boost::container::flat_map<std::string, std::unique_ptr<MCUTempSensor>> sensors;

MCUTempSensor::MCUTempSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                             boost::asio::io_service& io,
                             const std::string& sensorName,
                             const std::string& sensorConfiguration,
                             sdbusplus::asio::object_server& objectServer,
                             std::vector<thresholds::Threshold>&& thresholdData,
                             uint8_t busId, uint8_t mcuAddress, uint8_t tempReg,
                             std::string modeStr, uint8_t length,
                             uint64_t scale) :
    Sensor(escapeName(sensorName), std::move(thresholdData),
           sensorConfiguration, "xyz.openbmc_project.Configuration.ExitAirTemp",
           false, false, mcuTempMaxReading, mcuTempMinReading, conn),
    busId(busId), mcuAddress(mcuAddress), tempReg(tempReg),
    modeStr(std::move(modeStr)), length(length), scale(scale),
    objectServer(objectServer), waitTimer(io)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        "xyz.openbmc_project.Sensor.Value");

    if (thresholds::hasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/temperature/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Warning");
    }
    if (thresholds::hasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/temperature/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Critical");
    }
    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        association::interface);
}

MCUTempSensor::~MCUTempSensor()
{
    waitTimer.cancel();
    objectServer.remove_interface(thresholdInterfaceWarning);
    objectServer.remove_interface(thresholdInterfaceCritical);
    objectServer.remove_interface(sensorInterface);
    objectServer.remove_interface(association);
}

void MCUTempSensor::init(void)
{
    setInitialProperties(dbusConnection, sensor_paths::unitDegreesC);
    read();
}

void MCUTempSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}

int MCUTempSensor::getMCURegsInfoWord(uint8_t regs, int16_t* pu16data)
{
    std::string i2cBus = "/dev/i2c-" + std::to_string(busId);
    int fd = open(i2cBus.c_str(), O_RDWR);

    if (fd < 0)
    {
        std::cerr << " unable to open i2c device" << i2cBus << "  err=" << fd
                  << "\n";
        return -1;
    }

    if (ioctl(fd, I2C_SLAVE_FORCE, mcuAddress) < 0)
    {
        std::cerr << " unable to set device address\n";
        close(fd);
        return -1;
    }

    unsigned long funcs = 0;
    if (ioctl(fd, I2C_FUNCS, &funcs) < 0)
    {
        std::cerr << " not support I2C_FUNCS\n";
        close(fd);
        return -1;
    }

    if (!(funcs & I2C_FUNC_SMBUS_READ_WORD_DATA))
    {
        std::cerr << " not support I2C_FUNC_SMBUS_READ_WORD_DATA\n";
        close(fd);
        return -1;
    }

    if (modeStr == "Word")
    {
        *pu16data = i2c_smbus_read_word_data(fd, regs);
    }
    else if (modeStr == "Byte")
    {
        *pu16data = i2c_smbus_read_byte_data(fd, regs);
    }
    else if (modeStr == "Block")
    {
        uint8_t* val = new uint8_t[static_cast<int>(length)];
        int result = i2c_smbus_read_i2c_block_data(fd, regs, length, val);

        if (result < 0)
        {
            std::cerr << "Failed to read block data"
                      << "\n";
            free(val);
            return -1;
        }

        *pu16data = static_cast<int>(val[static_cast<int>(length) - 1]);
        free(val);
    }
    else
    {
        std::cerr << " invalid mode string"
                  << "\n";
        return -1;
    }

    close(fd);

    if (*pu16data < 0)
    {
        std::cerr << " read data failed at " << static_cast<int>(regs) << "\n";
        return -1;
    }

    return 0;
}

void MCUTempSensor::read(void)
{
    static constexpr size_t pollTime = 1; // in seconds

    waitTimer.expires_from_now(boost::posix_time::seconds(pollTime));
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
        int16_t temp;
        int ret = getMCURegsInfoWord(tempReg, &temp);
        if (ret >= 0)
        {
            double v = static_cast<double>(temp) / static_cast<double>(scale);
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
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
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
            for (const auto& pathPair : resp)
            {
                for (const auto& entry : pathPair.second)
                {
                    if (entry.first != configInterface)
                    {
                        continue;
                    }
                    std::string name =
                        loadVariant<std::string>(entry.second, "Name");

                    std::vector<thresholds::Threshold> sensorThresholds;
                    if (!parseThresholdsFromConfig(pathPair.second,
                                                   sensorThresholds))
                    {
                        std::cerr << "error populating thresholds for " << name
                                  << "\n";
                    }

                    uint8_t busId = loadVariant<uint8_t>(entry.second, "Bus");

                    uint8_t mcuAddress =
                        loadVariant<uint8_t>(entry.second, "Address");

                    uint8_t tempReg = loadVariant<uint8_t>(entry.second, "Reg");

                    std::string modeStr =
                        loadVariant<std::string>(entry.second, "Mode");

                    uint8_t length = 0;
                    if (modeStr == "Block")
                    {
                        length = loadVariant<uint8_t>(entry.second, "Length");
                    }

                    uint64_t scale =
                        loadVariant<uint64_t>(entry.second, "Scale");

                    std::string sensorClass =
                        loadVariant<std::string>(entry.second, "Class");

                    if constexpr (debug)
                    {
                        std::cerr
                            << "Configuration parsed for \n\t" << entry.first
                            << "\n"
                            << "with\n"
                            << "\tName: " << name << "\n"
                            << "\tBus: " << static_cast<int>(busId) << "\n"
                            << "\tAddress: " << static_cast<int>(mcuAddress)
                            << "\n"
                            << "\tReg: " << static_cast<int>(tempReg) << "\n"
                            << "\tMode: " << modeStr << "\n"
                            << "\tLength: " << static_cast<int>(length) << "\n"
                            << "\tScale: " << scale << "\n"
                            << "\tClass: " << sensorClass << "\n";
                    }

                    auto& sensor = sensors[name];

                    sensor = std::make_unique<MCUTempSensor>(
                        dbusConnection, io, name, pathPair.first, objectServer,
                        std::move(sensorThresholds), busId, mcuAddress, tempReg,
                        modeStr, length, scale);

                    sensor->init();
                }
            }
        },
        entityManagerName, "/", "org.freedesktop.DBus.ObjectManager",
        "GetManagedObjects");
}

int main()
{
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.MCUTempSensor");
    sdbusplus::asio::object_server objectServer(systemBus);

    io.post([&]() { createSensors(io, objectServer, sensors, systemBus); });

    boost::asio::deadline_timer configTimer(io);

    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message&) {
            configTimer.expires_from_now(boost::posix_time::seconds(1));
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

    sdbusplus::bus::match::match configMatch(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',"
        "path_namespace='" +
            std::string(inventoryPath) +
            "',"
            "arg0namespace='" +
            configInterface + "'",
        eventHandler);

    setupManufacturingModeMatch(*systemBus);
    io.run();
    return 0;
}
