/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "SmbpbiSensor.hpp"

#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

#include <linux/i2c.h>

#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

extern "C"
{
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
}

constexpr const char* configInterface =
    "xyz.openbmc_project.Configuration.SmbpbiVirtualEeprom";
constexpr const char* sensorRootPath = "/xyz/openbmc_project/sensors/";
constexpr const char* objectType = "SmbpbiVirtualEeprom";

boost::container::flat_map<std::string, std::unique_ptr<SmbpbiSensor>> sensors;

SmbpbiSensor::SmbpbiSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const std::string& sensorName,
    const std::string& sensorConfiguration, const std::string& objType,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData, uint8_t busId,
    uint8_t addr, uint16_t offset, std::string& sensorUnits,
    std::string& valueType, size_t pollTime, double minVal, double maxVal,
    std::string& path, const PowerState& powerState) :
    Sensor(escapeName(sensorName), std::move(thresholdData),
           sensorConfiguration, objType, false, false, maxVal, minVal, conn,
           powerState),
    busId(busId), addr(addr), offset(offset), sensorUnits(sensorUnits),
    valueType(valueType), objectServer(objectServer),
    inputDev(io, path, boost::asio::random_access_file::read_only),
    waitTimer(io), pollRateSecond(pollTime)
{
    sensorType = sensor_paths::getPathForUnits(sensorUnits);
    std::string sensorPath = sensorRootPath + sensorType + "/";

    sensorInterface =
        objectServer.add_interface(sensorPath + name, sensorValueInterface);

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface(sensorPath + name, interface);
    }
    association =
        objectServer.add_interface(sensorPath + name, association::interface);

    if (sensorType == "temperature")
    {
        setInitialProperties(sensor_paths::unitDegreesC);
    }
    else if (sensorType == "power")
    {
        setInitialProperties(sensor_paths::unitWatts);
    }
    else if (sensorType == "energy")
    {
        setInitialProperties(sensor_paths::unitJoules);
    }
    else if (sensorType == "voltage")
    {
        setInitialProperties(sensor_paths::unitVolts);
    }
    else
    {
        lg2::error("no sensor type found");
    }
}

SmbpbiSensor::~SmbpbiSensor()
{
    inputDev.close();
    waitTimer.cancel();
    for (const auto& iface : thresholdInterfaces)
    {
        objectServer.remove_interface(iface);
    }
    objectServer.remove_interface(sensorInterface);
    objectServer.remove_interface(association);
}

void SmbpbiSensor::init()
{
    read();
}

void SmbpbiSensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}

double SmbpbiSensor::convert2Temp(const uint8_t* raw)
{
    // Temp data is encoded in SMBPBI format. The 3 MSBs denote
    // the integer portion, LSB is an encoded fraction.
    // this automatic convert to int (two's complement integer)
    int32_t intg = (raw[3] << 24 | raw[2] << 16 | raw[1] << 8 | raw[0]);
    uint8_t frac = uint8_t(raw[0]);
    // shift operation on a int keeps the sign in two's complement
    intg >>= 8;

    double temp = 0;
    if (intg > 0)
    {
        temp = double(intg) + double(frac / 256.0);
    }
    else
    {
        temp = double(intg) - double(frac / 256.0);
    }

    return temp;
}

double SmbpbiSensor::convert2Power(const uint8_t* raw)
{
    // Power data is encoded as a 4-byte unsigned integer
    uint32_t val = (raw[3] << 24) + (raw[2] << 16) + (raw[1] << 8) + raw[0];

    // mWatts to Watts
    double power = static_cast<double>(val) / 1000;

    return power;
}

int SmbpbiSensor::i2cReadDataBytesDouble(double& reading)
{
    constexpr int length =
        i2CReadLenValues[static_cast<size_t>(I2C_READ_LEN_INDEX::FLOAT64)];

    static_assert(length == sizeof(reading), "Unsupported arch");

    std::array<uint8_t, length> buf{};
    int ret = i2cReadDataBytes(buf.data(), length);
    if (ret < 0)
    {
        return ret;
    }
    // there is no value updated from HMC if reading data is all 0xff
    // Return NaN since reading is already a double
    if (checkInvalidReading(buf.data(), length))
    {
        reading = std::numeric_limits<double>::quiet_NaN();
        return 0;
    }
    uint64_t tempd = 0;
    for (int byteI = 0; byteI < length; byteI++)
    {
        tempd |= static_cast<uint64_t>(buf[byteI]) << (8 * byteI);
    }
    std::memcpy(&reading, &tempd, sizeof(reading));

    return 0;
}

int SmbpbiSensor::i2cReadDataBytesUI64(uint64_t& reading)
{
    constexpr int length =
        i2CReadLenValues[static_cast<size_t>(I2C_READ_LEN_INDEX::UINT64)];

    static_assert(length == sizeof(reading), "Unsupported arch");

    std::array<uint8_t, length> buf{};
    int ret = i2cReadDataBytes(buf.data(), length);
    if (ret < 0)
    {
        return ret;
    }
    reading = 0;
    for (int byteI = 0; byteI < length; byteI++)
    {
        reading |= static_cast<uint64_t>(buf[byteI]) << (8 * byteI);
    }
    return 0;
}

// Generic i2c Command to read bytes
int SmbpbiSensor::i2cReadDataBytes(uint8_t* reading, int length)
{
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    const int fd = inputDev.native_handle();
    if (fd < 0)
    {
        lg2::error(" unable to open i2c device on bus {BUS} err={FD}", "BUS",
                   busId, "FD", fd);
        return -1;
    }

    unsigned long funcs = 0;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    if (ioctl(fd, I2C_FUNCS, &funcs) < 0)
    {
        lg2::error(" I2C_FUNCS not supported");
        return -1;
    }

    int ret = 0;
    struct i2c_rdwr_ioctl_data args = {nullptr, 0};
    std::array<struct i2c_msg, 2> msgs = {
        {{0, 0, 0, nullptr}, {0, 0, 0, nullptr}}};
    std::array<uint8_t, 8> cmd{};

    args.msgs = msgs.data();
    args.nmsgs = msgs.size();

    msgs[0].addr = addr;
    msgs[0].flags = 0;
    msgs[0].buf = cmd.data();
    // handle two bytes offset
    if (offset > 255)
    {
        msgs[0].len = 2;
        msgs[0].buf[0] = offset >> 8;
        msgs[0].buf[1] = offset & 0xFF;
    }
    else
    {
        msgs[0].len = 1;
        msgs[0].buf[0] = offset & 0xFF;
    }

    msgs[1].addr = addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = length;
    msgs[1].buf = reading;

    // write offset
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    ret = ioctl(fd, I2C_RDWR, &args);
    if (ret < 0)
    {
        return ret;
    }
    return 0;
}

int SmbpbiSensor::readRawEEPROMData(double& data)
{
    uint64_t reading = 0;
    int ret = i2cReadDataBytesUI64(reading);
    if (ret < 0)
    {
        return ret;
    }
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    if (checkInvalidReading(reinterpret_cast<uint8_t*>(&reading),
                            sizeof(reading)))
    {
        data = std::numeric_limits<double>::quiet_NaN();
        return 0;
    }
    lg2::debug("offset: {OFFSET} reading: {READING}", "OFFSET", offset,
               "READING", reading);
    if (sensorType == "temperature")
    {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        data = convert2Temp(reinterpret_cast<uint8_t*>(&reading));
    }
    else if (sensorType == "power")
    {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        data = convert2Power(reinterpret_cast<uint8_t*>(&reading));
    }
    else if (sensorType == "energy")
    {
        data = reading / 1000.0; // mJ to J (double)
    }
    else
    {
        data = reading; // Voltage
    }
    return 0;
}

int SmbpbiSensor::readFloat64EEPROMData(double& data)
{
    double reading = 0;
    int ret = i2cReadDataBytesDouble(reading);
    if (ret < 0)
    {
        return ret;
    }
    data = reading;
    return 0;
}

void SmbpbiSensor::waitReadCallback(const boost::system::error_code& ec)
{
    if (ec == boost::asio::error::operation_aborted)
    {
        // we're being cancelled
        return;
    }
    // read timer error
    if (ec)
    {
        lg2::error("timer error");
        return;
    }
    double temp = 0;

    int ret = 0;
    // Sensor reading value types are sensor-specific. So, read
    // and interpret sensor data based on it's value type.
    if (valueType == "UINT64")
    {
        ret = readRawEEPROMData(temp);
    }
    else if (valueType == "FLOAT64")
    {
        ret = readFloat64EEPROMData(temp);
    }
    else
    {
        return;
    }

    if (ret >= 0)
    {
        // Update value when either:
        // 1) the sensor name is NOT one of the HGX GPU energy sensors (always
        // update), OR 2) the sensor IS one of the HGX GPU energy sensors AND
        // the read value (temp) is non-zero.
        //
        // Rationale: some HGX GPU energy sensors may return 0 as an
        // invalid/unreliable reading; we skip updates for those zero readings
        // but allow updates for non-GPU sensors even if zero.
        if ((name != "HGX_GPU0_ENERGY_J" && name != "HGX_GPU1_ENERGY_J" &&
             name != "HGX_GPU2_ENERGY_J" && name != "HGX_GPU3_ENERGY_J") ||
            (name == "HGX_GPU0_ENERGY_J" && temp != 0) ||
            (name == "HGX_GPU1_ENERGY_J" && temp != 0) ||
            (name == "HGX_GPU2_ENERGY_J" && temp != 0) ||
            (name == "HGX_GPU3_ENERGY_J" && temp != 0))
        {
            lg2::debug("Value update to {TEMP}", "TEMP", temp);
            updateValue(temp);
        }
    }
    else
    {
        lg2::error("Invalid read getRegsInfo");
        incrementError();
    }
    read();
}

void SmbpbiSensor::read()
{
    size_t pollTime = getPollRate(); // in seconds

    waitTimer.expires_after(std::chrono::seconds(pollTime));
    waitTimer.async_wait([this](const boost::system::error_code& ec) {
        this->waitReadCallback(ec);
    });
}

static void createSensorCallback(
    boost::system::error_code ec, const ManagedObjectType& resp,
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    boost::container::flat_map<std::string, std::unique_ptr<SmbpbiSensor>>&
        sensors)
{
    if (ec)
    {
        lg2::error("Error contacting entity manager");
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
            std::string name = loadVariant<std::string>(entry.second, "Name");

            std::vector<thresholds::Threshold> sensorThresholds;
            if (!parseThresholdsFromConfig(pathPair.second, sensorThresholds))
            {
                lg2::error("error populating thresholds for {NAME}", "NAME",
                           name);
            }

            uint8_t busId = loadVariant<uint8_t>(entry.second, "Bus");

            uint8_t addr = loadVariant<uint8_t>(entry.second, "Address");

            uint16_t off = loadVariant<uint16_t>(entry.second, "ReadOffset");

            PowerState pwrState = getPowerState(entry.second);

            std::string sensorUnits =
                loadVariant<std::string>(entry.second, "Units");

            std::string valueType =
                loadVariant<std::string>(entry.second, "ValueType");
            if (valueType != "UINT64" && valueType != "FLOAT64")
            {
                lg2::error("Invalid ValueType for sensor: {NAME}", "NAME",
                           name);
                break;
            }

            size_t rate = loadVariant<uint8_t>(entry.second, "PollRate");

            double minVal = loadVariant<double>(entry.second, "MinValue");

            double maxVal = loadVariant<double>(entry.second, "MaxValue");
            lg2::debug("Configuration parsed for \n\t {CONF}\nwith\n"
                       "\tName: {NAME}\n"
                       "\tBus: {BUS}\n"
                       "\tAddress:{ADDR}\n"
                       "\tOffset: {OFF}\n"
                       "\tType : {TYPE}\n"
                       "\tValue Type : {VALUETYPE}\n"
                       "\tPollrate: {RATE}\n"
                       "\tMinValue: {MIN}\n"
                       "\tMaxValue: {MAX}\n"
                       "\tPowerState: {PWRSTATE}\n",
                       "CONF", entry.first, "NAME", name, "BUS",
                       static_cast<int>(busId), "ADDR", static_cast<int>(addr),
                       "OFF", static_cast<int>(off), "UNITS", sensorUnits,
                       "VALUETYPE", valueType, "RATE", rate, "MIN", minVal,
                       "MAX", maxVal, "PWRSTATE", pwrState);

            auto& sensor = sensors[name];
            sensor = nullptr;

            std::string path = "/dev/i2c-" + std::to_string(busId);

            sensor = std::make_unique<SmbpbiSensor>(
                dbusConnection, io, name, pathPair.first, objectType,
                objectServer, std::move(sensorThresholds), busId, addr, off,
                sensorUnits, valueType, rate, minVal, maxVal, path, pwrState);

            sensor->init();
        }
    }
}

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::unique_ptr<SmbpbiSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    if (!dbusConnection)
    {
        lg2::error("Connection not created");
        return;
    }

    dbusConnection->async_method_call(
        [&io, &objectServer, &dbusConnection, &sensors](
            boost::system::error_code ec, const ManagedObjectType& resp) {
            createSensorCallback(ec, resp, io, objectServer, dbusConnection,
                                 sensors);
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
    systemBus->request_name("xyz.openbmc_project.SMBPBI");

    boost::asio::post(io, [&]() {
        createSensors(io, objectServer, sensors, systemBus);
    });

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
                    lg2::error("timer error");
                    return;
                }
                createSensors(io, objectServer, sensors, systemBus);
                if (sensors.empty())
                {
                    lg2::info("Configuration not detected");
                }
            });
        };

    sdbusplus::bus::match_t configMatch(
        static_cast<sdbusplus::bus_t&>(*systemBus),
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
