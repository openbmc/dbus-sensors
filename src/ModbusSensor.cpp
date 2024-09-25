#include "ModbusSensor.hpp"

#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

static constexpr bool debug = false;

static constexpr const char* sensorType = "ModbusSensor";
static constexpr const char* modbusServiceName =
    "xyz.openbmc_project.ModbusSensor";
static constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";

// Supported protocol
static constexpr const char* PROTOCOL_TCP = "TCP";
static constexpr const char* PROTOCOL_RTU = "RTU";

// Supported function code
static constexpr const uint16_t READ_DISCRETE_INPUTS = 2;
static constexpr const uint16_t READ_HOLDING_REGISTER = 3;
static constexpr const uint16_t READ_INPUT_REGISTERS = 4;
static constexpr const uint16_t WRITE_SINGLE_REGISTER = 6;

// Timeout for Modbus connection response
static constexpr const uint32_t TIMEOUT_SEC = 2;       // in seconds
static constexpr const uint32_t TIMEOUT_USEC = 500000; // in microseconds

// Constants for signed readings
static constexpr double MAX_SIGNED_DUAL_WORD = 2147483647;
static constexpr double MIN_SIGNED_DUAL_WORD = -2147483648;
static constexpr double MAX_SIGNED_WORD = 32767;
static constexpr double MIN_SIGNED_WORD = -32768;

// Constants for unsigned readings
static constexpr double MAX_UNSIGNED_DUAL_WORD = 4294967295;
static constexpr double MAX_UNSIGNED_WORD = 65535;
static constexpr double MIN_UNSIGNED = 0;

static boost::container::flat_map<std::string, std::string> sensorTable = {
    {"DegreesC", sensor_paths::unitDegreesC},
    {"DegreesF", sensor_paths::unitDegreesF},
    {"CF", sensor_paths::unitCF},
    {"CM", sensor_paths::unitCM},
    {"CMH", sensor_paths::unitCMH},
    {"CMS", sensor_paths::unitCMS},
    {"Gallon", sensor_paths::unitGallon},
    {"GPM", sensor_paths::unitGPM},
    {"Liter", sensor_paths::unitLiter},
    {"LPH", sensor_paths::unitLPH},
    {"LPM", sensor_paths::unitLPM},
    {"LPS", sensor_paths::unitLPS},
    {"Percent", sensor_paths::unitPercent},
    {"Volts", sensor_paths::unitVolts}};

boost::container::flat_map<std::string, std::unique_ptr<ModbusSensor>> sensors;

ModbusSensor::ModbusSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const std::string& sensorName,
    const std::string& sensorConfiguration,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData,
    const std::pair<double, double>& limits, std::string& protocol,
    uint8_t deviceId, std::string& ipAddress, uint16_t portNumber,
    std::string& device, uint32_t baud, std::string& parity, uint8_t dataBit,
    uint8_t stopBit, uint16_t requestFunctionCode, uint16_t requestAddress,
    uint16_t requestQuantity, uint16_t requestByteCount, uint16_t requestValue,
    uint16_t readLength, double scaleFactor, bool signedInteger,
    bool readTwoWords, std::string sensorUnit) :
    Sensor(escapeName(sensorName), std::move(thresholdData),
           sensorConfiguration, "ModbusSensor", false, false, limits.second,
           limits.first, conn),
    protocol(protocol), deviceId(deviceId), ipAddress(ipAddress),
    portNumber(portNumber), device(device), baud(baud), parity(parity),
    dataBit(dataBit), stopBit(stopBit),
    requestFunctionCode(requestFunctionCode), requestAddress(requestAddress),
    requestQuantity(requestQuantity), requestByteCount(requestByteCount),
    requestValue(requestValue), readLength(readLength),
    scaleFactor(scaleFactor), signedInteger(signedInteger),
    readTwoWords(readTwoWords), sensorUnit(sensorUnit),
    objectServer(objectServer), waitTimer(io)
{
    std::string unitPath = sensor_paths::getPathForUnits(sensorUnit);
    if (unitPath == "")
        return;

    std::string dbusPath = sensorPathPrefix + unitPath + "/" + name;

    sensorInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface(dbusPath, interface);
    }

    auto findSensorUnit = sensorTable.find(sensorUnit);
    if (findSensorUnit == sensorTable.end())
    {
        std::cerr << "Unit " << sensorUnit << " does not exist.\n";
        return;
    }

    setInitialProperties(findSensorUnit->second);

    association = objectServer.add_interface(dbusPath, association::interface);
}

ModbusSensor::~ModbusSensor()
{
    modbus_close(mb);
    modbus_free(mb);

    waitTimer.cancel();
    for (const auto& iface : thresholdInterfaces)
    {
        objectServer.remove_interface(iface);
    }
    objectServer.remove_interface(sensorInterface);
    objectServer.remove_interface(association);
}

void ModbusSensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}

void ModbusSensor::setSensorValue()
{
    switch (requestFunctionCode)
    {
        // Function code: 2
        case (READ_DISCRETE_INPUTS):
            if constexpr (debug)
            {
                std::cout << "responseHalfword = ";
                for (const auto& data : responseHalfword)
                    std::cout
                        << data
                        << (&data != &responseHalfword.back() ? ", " : "\n");
            }

            updateValue((signedInteger ? static_cast<int16_t>(
                                             responseHalfword[readLength - 1])
                                       : responseHalfword[readLength - 1]) *
                        scaleFactor);
            break;

        // Function code: 3 or 4
        case (READ_HOLDING_REGISTER):
        case (READ_INPUT_REGISTERS):
            if constexpr (debug)
            {
                std::cout << "responseWord = ";
                for (const auto& data : responseWord)
                    std::cout << data
                              << (&data != &responseWord.back() ? ", " : "\n");
            }

            if (readTwoWords)
            {
                updateValue((static_cast<uint32_t>(responseWord[1]) << 16) |
                            static_cast<uint32_t>(responseWord[0]));
            }
            else
            {
                updateValue((signedInteger ? static_cast<int16_t>(
                                                 responseWord[readLength - 1])
                                           : responseWord[readLength - 1]) *
                            scaleFactor);
            }
            break;

        default:
            if constexpr (debug)
            {
                std::cerr << "No matching function available for reading."
                          << std::endl;
            }
            break;
    }
}

int ModbusSensor::request()
{
    switch (requestFunctionCode)
    {
        // Function code: 2
        case (READ_DISCRETE_INPUTS):
            if (modbus_read_input_bits(mb, requestAddress, readLength,
                                       responseHalfword.data()) == -1)
            {
                std::cerr << "Failed to read discrete inputs."
                          << modbus_strerror(errno) << std::endl;
            }
            break;

        // Function code: 3
        case (READ_HOLDING_REGISTER):
            if (modbus_read_registers(mb, requestAddress, readLength,
                                      responseWord.data()) == -1)
            {
                std::cerr << "Failed to read holding register."
                          << modbus_strerror(errno) << std::endl;
            }
            break;

        // Function code: 4
        case (READ_INPUT_REGISTERS):
            if (modbus_read_input_registers(mb, requestAddress, readLength,
                                            responseWord.data()) == -1)
            {
                std::cerr << "Failed to read input register."
                          << modbus_strerror(errno) << std::endl;
            }
            break;

        // Function code: 6
        case (WRITE_SINGLE_REGISTER):
            if (modbus_write_register(mb, requestAddress, requestValue) == -1)
            {
                std::cerr << "Failed to write single register."
                          << modbus_strerror(errno) << std::endl;
            }
            break;

        default:
            if constexpr (debug)
            {
                std::cerr << "No matching function code" << std::endl;
                return -1;
            }
            break;
    }

    return 0;
}

void ModbusSensor::routine()
{
    static constexpr size_t pollTime = 5; // in seconds

    waitTimer.expires_after(std::chrono::seconds(pollTime));
    waitTimer.async_wait([this](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
            return;
        if (ec)
        {
            std::cerr << "timer error\n";
            return;
        }

        if (request() == -1)
            return;

        setSensorValue();
        routine();
    });
}

int ModbusSensor::setupModbusConnection()
{
    if (protocol == PROTOCOL_RTU)
    {
        mb = modbus_new_rtu(device.data(), baud, parity.at(0), dataBit,
                            stopBit);
        if (mb == NULL)
            std::cerr << "The Modbus RTU object cannot be created.\n";
    }
    else if (protocol == PROTOCOL_TCP)
    {
        mb = modbus_new_tcp(ipAddress.data(), portNumber);

        if (mb == NULL)
            std::cerr << "The Modbus TCP object cannot be created.\n";
    }
    else
    {
        std::cerr << "No matching protocol.";
        return -1;
    }

    if (modbus_set_slave(mb, deviceId) == -1)
    {
        std::cerr << "Failed to set device ID.";
        return -1;
    }

    if (modbus_connect(mb) == -1)
    {
        std::cerr << "Connection failed.";
        return -1;
    }

    if (modbus_set_response_timeout(mb, TIMEOUT_SEC, TIMEOUT_USEC) == -1)
    {
        std::cerr << "Failed to set timeout.";
        return -1;
    }

    return 0;
}

int ModbusSensor::startConnection()
{
    size_t retryCount = 0;
    constexpr size_t maxRetries = 5;

    while (retryCount < maxRetries)
    {
        if (setupModbusConnection() != -1)
            break;

        std::cout << "Retry " << retryCount + 1
                  << ": Modbus connection, retrying in 5 seconds..."
                  << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(5));

        retryCount++;
    }

    if (retryCount == maxRetries)
    {
        std::cerr << "Connection failed: " << modbus_strerror(errno)
                  << std::endl;
        return -1;
    }

    return 0;
}

void ModbusSensor::init()
{
    if (startConnection() == -1)
    {
        modbus_free(mb);
        return;
    }

    if (readTwoWords)
        readLength = 2;

    switch (requestFunctionCode)
    {
        // Function code: 2
        case (READ_DISCRETE_INPUTS):
            responseHalfword.clear();
            responseHalfword.resize(readLength);
            break;

        // Function code: 3 or 4
        case (READ_HOLDING_REGISTER):
        case (READ_INPUT_REGISTERS):
            responseWord.clear();
            responseWord.resize(readLength);
            break;

        default:
            if constexpr (debug)
                std::cerr << "No matching function code" << std::endl;
            break;
    }

    routine();
}

template <typename T>
inline T loadVariantWithDefault(const SensorBaseConfigMap& cfg,
                                const std::string& key, T defaultValue)
{
    try
    {
        return loadVariant<T>(cfg, key);
    }
    catch (const std::invalid_argument& e)
    {
        if constexpr (debug)
            std::cout << key << ": " << e.what() << std::endl;
        return defaultValue;
    }
}

static double setMaxReading(bool signedInteger, bool readTwoWords)
{
    if (signedInteger)
        return readTwoWords ? MAX_SIGNED_DUAL_WORD : MAX_SIGNED_WORD;
    else
        return readTwoWords ? MAX_UNSIGNED_DUAL_WORD : MAX_UNSIGNED_WORD;
}

static double setMinReading(bool signedInteger, bool readTwoWords)
{
    if (signedInteger)
        return readTwoWords ? MIN_SIGNED_DUAL_WORD : MIN_SIGNED_WORD;
    else
        return MIN_UNSIGNED;
}

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::unique_ptr<ModbusSensor>>&
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
                    continue;

                std::string name = loadVariant<std::string>(cfg, "Name");

                std::string protocol =
                    loadVariantWithDefault<std::string>(cfg, "Protocol", "");
                uint8_t deviceId =
                    loadVariantWithDefault<uint8_t>(cfg, "DeviceID", 0);
                std::string ipAddress =
                    loadVariantWithDefault<std::string>(cfg, "IP", "");
                uint16_t portNumber =
                    loadVariantWithDefault<uint16_t>(cfg, "Port", 0);
                std::string device =
                    loadVariantWithDefault<std::string>(cfg, "Device", "");
                uint32_t baud = loadVariantWithDefault<uint32_t>(cfg, "Baud",
                                                                 0);
                std::string parity =
                    loadVariantWithDefault<std::string>(cfg, "Parity", "");
                uint8_t dataBit =
                    loadVariantWithDefault<uint16_t>(cfg, "DataBit", 0);
                uint8_t stopBit =
                    loadVariantWithDefault<uint16_t>(cfg, "StopBit", 0);
                uint16_t requestFunctionCode = loadVariantWithDefault<uint16_t>(
                    cfg, "RequestFunctionCode", 3);
                uint16_t requestAddress =
                    loadVariantWithDefault<uint16_t>(cfg, "RequestAddress", 0);
                uint16_t requestQuantity =
                    loadVariantWithDefault<uint16_t>(cfg, "RequestQuantity", 2);
                uint16_t requestByteCount = loadVariantWithDefault<uint16_t>(
                    cfg, "RequestByteCount", 1);
                uint16_t requestValue =
                    loadVariantWithDefault<uint16_t>(cfg, "RequestValue", 0);
                uint16_t readLength =
                    loadVariantWithDefault<uint16_t>(cfg, "ReadLength", 1);
                double scaleFactor =
                    loadVariantWithDefault<double>(cfg, "ScaleFactor", 1.0);
                bool signedInteger =
                    loadVariantWithDefault<double>(cfg, "Signed", false);
                bool readTwoWords =
                    loadVariantWithDefault<double>(cfg, "ReadTwoWords", false);
                std::string sensorUnit =
                    loadVariantWithDefault<std::string>(cfg, "Unit", "");

                std::vector<thresholds::Threshold> sensorThresholds;
                if (!parseThresholdsFromConfig(interfaces, sensorThresholds))
                {
                    std::cerr << "error populating thresholds for " << name
                              << "\n";
                }

                if constexpr (debug)
                {
                    std::cout
                        << "Configuration parsed for \n\t" << intf << "\n"
                        << "with\n"
                        << "\tName: " << name << "\n"
                        << "\tProtocol: " << protocol << "\n"
                        << "\tdeviceId: " << static_cast<int>(deviceId) << "\n"
                        << "\tIpAddress: " << ipAddress << "\n"
                        << "\tPortNumber: " << static_cast<int>(portNumber)
                        << "\n"
                        << "\tDevice: " << device << "\n"
                        << "\tBaud: " << static_cast<int>(baud) << "\n"
                        << "\tParity: " << parity << "\n"
                        << "\tDataBitr: " << static_cast<int>(dataBit) << "\n"
                        << "\tStopBit: " << static_cast<int>(stopBit) << "\n"
                        << "\tRequestFunctionCode: "
                        << static_cast<int>(requestFunctionCode) << "\n"
                        << "\tRequestAddress: "
                        << static_cast<int>(requestAddress) << "\n"
                        << "\tRequestQuantity: "
                        << static_cast<int>(requestQuantity) << "\n"
                        << "\tRequestByteCount: "
                        << static_cast<int>(requestByteCount) << "\n"
                        << "\tReadLength: " << static_cast<int>(readLength)
                        << "\n"
                        << "\tScaleFactor: " << scaleFactor << "\n"
                        << "\tSigned: " << signedInteger << "\n"
                        << "\tReadTwoWords: " << readTwoWords << "\n"
                        << "\tUnit: " << sensorUnit << "\n"
                        << "\tRequestValue: " << requestValue << "\n";
                }

                std::pair<double, double> limits =
                    std::make_pair(setMinReading(signedInteger, readTwoWords),
                                   setMaxReading(signedInteger, readTwoWords));

                findLimits(limits, &(*interfaces.find(intf)));

                auto& sensor = sensors[name];

                sensor = std::make_unique<ModbusSensor>(
                    dbusConnection, io, name, path, objectServer,
                    std::move(sensorThresholds), limits, protocol, deviceId,
                    ipAddress, portNumber, device, baud, parity, dataBit,
                    stopBit, requestFunctionCode, requestAddress,
                    requestQuantity, requestByteCount, requestValue, readLength,
                    scaleFactor, signedInteger, readTwoWords, sensorUnit);

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

    systemBus->request_name("xyz.openbmc_project.ModbusSensor");

    boost::asio::post(
        io, [&]() { createSensors(io, objectServer, sensors, systemBus); });

    boost::asio::steady_timer configTimer(io);

    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&](sdbusplus::message_t&) {
        configTimer.expires_after(std::chrono::seconds(1));
        configTimer.async_wait([&](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
                return;
            if (ec)
            {
                std::cerr << "timer error\n";
                return;
            }
            createSensors(io, objectServer, sensors, systemBus);
            if (sensors.empty())
                std::cout << "Configuration not detected\n";
        });
    };

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(
            *systemBus, std::to_array<const char*>({sensorType}), eventHandler);
    setupManufacturingModeMatch(*systemBus);
    io.run();
    return 0;
}
