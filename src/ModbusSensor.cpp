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
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

static constexpr bool debug = false;

static constexpr const char* sensorType = "ModbusSensor";
static constexpr const char* modbusServiceName =
    "xyz.openbmc_project.ModbusSensor";
static constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";

// Supported protocol
static constexpr const char* PROTOCOL_RTU = "RTU";
static constexpr const char* PROTOCOL_TCP = "TCP";

// Supported function code
static constexpr const uint16_t READ_DISCRETE_INPUTS = 2;
static constexpr const uint16_t READ_HOLDING_REGISTER = 3;
static constexpr const uint16_t READ_INPUT_REGISTERS = 4;
static constexpr const uint16_t WRITE_SINGLE_REGISTER = 6;

// Timeout for Modbus connection response
static constexpr const uint32_t TIMEOUT_SEC = 1;  // in seconds
static constexpr const uint32_t TIMEOUT_USEC = 0; // in microseconds

// Constants for signed readings
static constexpr double MAX_SIGNED_DUAL_WORD = 2147483647;
static constexpr double MIN_SIGNED_DUAL_WORD = -2147483648;
static constexpr double MAX_SIGNED_WORD = 32767;
static constexpr double MIN_SIGNED_WORD = -32768;

// Constants for unsigned readings
static constexpr double MAX_UNSIGNED_DUAL_WORD = 4294967295;
static constexpr double MAX_UNSIGNED_WORD = 65535;
static constexpr double MIN_UNSIGNED = 0;

static boost::container::flat_map<std::string, std::string> unitTable = {
    {"DegreesC", sensor_paths::unitDegreesC},
    {"LPM", sensor_paths::unitLPM},
    {"Percent", sensor_paths::unitPercent},
    {"Volts", sensor_paths::unitVolts}};

boost::container::flat_map<std::string, std::shared_ptr<ModbusSensor>> sensors;

std::map<std::string, std::pair<std::unique_ptr<RequestQueue>, std::thread>>
    requestQueueMap;

RequestQueue& getRequestQueue(const struct ModbusConfig& config)
{
    std::string target = "";
    if (config.protocol == PROTOCOL_RTU)
        target = config.device;
    else if (config.protocol == PROTOCOL_TCP)
        target = config.ipAddress;

    auto it = requestQueueMap.find(target);
    if (it != requestQueueMap.end())
        return *(it->second.first);

    auto queue = std::make_unique<RequestQueue>();
    std::thread processor(&RequestQueue::processRequests, queue.get());

    requestQueueMap[target] =
        std::make_pair(std::move(queue), std::move(processor));
    requestQueueMap[target].second.detach();

    return *(requestQueueMap[target].first);
}

ModbusSensor::ModbusSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const std::string& sensorName,
    const std::string& sensorConfiguration,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData,
    const std::pair<double, double>& limits,
    const struct ModbusConfig&& config) :
    Sensor(escapeName(sensorName), std::move(thresholdData),
           sensorConfiguration, "ModbusSensor", false, false, limits.second,
           limits.first, conn),
    config(std::move(config)), objectServer(objectServer), waitTimer(io)
{
    std::string unitPath = sensor_paths::getPathForUnits(config.sensorUnit);
    if (unitPath == "")
        unitPath = "unclassified";
    std::string dbusPath;

    switch (config.requestFunctionCode)
    {
        // Function code: 2, 3 or 4
        case (READ_DISCRETE_INPUTS):
        case (READ_HOLDING_REGISTER):
        case (READ_INPUT_REGISTERS):
            dbusPath = sensorPathPrefix + unitPath + "/" + name;

            sensorInterface = objectServer.add_interface(
                dbusPath, "xyz.openbmc_project.Sensor.Value");

            for (const auto& threshold : thresholds)
            {
                std::string interface =
                    thresholds::getInterface(threshold.level);
                thresholdInterfaces[static_cast<size_t>(threshold.level)] =
                    objectServer.add_interface(dbusPath, interface);
            }

            auto findSensorUnit = unitTable.find(config.sensorUnit);
            if (findSensorUnit == unitTable.end())
            {
                std::cout << "Unit " << config.sensorUnit << " does not exist."
                          << std::endl;
                setInitialProperties("");
            }
            else
                setInitialProperties(findSensorUnit->second);

            association =
                objectServer.add_interface(dbusPath, association::interface);
            break;
    }
}

ModbusSensor::~ModbusSensor()
{
    modbus_close(mb);
    modbus_free(mb);

    switch (config.requestFunctionCode)
    {
        // Function code: 2, 3 or 4
        case (READ_DISCRETE_INPUTS):
        case (READ_HOLDING_REGISTER):
        case (READ_INPUT_REGISTERS):
            waitTimer.cancel();
            for (const auto& iface : thresholdInterfaces)
            {
                objectServer.remove_interface(iface);
            }
            objectServer.remove_interface(association);
            objectServer.remove_interface(sensorInterface);
            break;

        // Function code: 6
        case (WRITE_SINGLE_REGISTER):
            objectServer.remove_interface(sensorInterface);
            break;

        default:
            if constexpr (debug)
            {
                std::cerr
                    << "No matching function code found for destructuring ModbusSensor."
                    << std::endl;
            }
            break;
    }
}

void ModbusSensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}

void ModbusSensor::setSensorValue()
{
    switch (config.requestFunctionCode)
    {
        // Function code: 2
        case (READ_DISCRETE_INPUTS):
            if constexpr (debug)
            {
                std::cout << name << ", responseHalfword = ";
                for (const auto& data : responseHalfword)
                    std::cout
                        << data
                        << (&data != &responseHalfword.back() ? ", " : "\n");
            }

            updateValue((config.signedInteger
                             ? static_cast<int16_t>(
                                   responseHalfword[config.readLength - 1])
                             : responseHalfword[config.readLength - 1]) *
                        config.scaleFactor);
            break;

        // Function code: 3 or 4
        case (READ_HOLDING_REGISTER):
        case (READ_INPUT_REGISTERS):
            if constexpr (debug)
            {
                std::cout << name << ", responseWord = ";
                for (const auto& data : responseWord)
                    std::cout << data
                              << (&data != &responseWord.back() ? ", " : "\n");
            }

            if (config.readTwoWords)
            {
                updateValue(((static_cast<uint32_t>(responseWord[1]) << 16) |
                             static_cast<uint32_t>(responseWord[0])));
            }
            else
            {
                updateValue((config.signedInteger
                                 ? static_cast<int16_t>(
                                       responseWord[config.readLength - 1])
                                 : responseWord[config.readLength - 1]) *
                            config.scaleFactor);
            }
            break;

        default:
            if constexpr (debug)
            {
                std::cerr << name
                          << ": No matching function code found for reading."
                          << std::endl;
            }
            break;
    }
}

void ModbusSensor::request()
{
    switch (config.requestFunctionCode)
    {
        // Function code: 2
        case (READ_DISCRETE_INPUTS):
            if (modbus_read_input_bits(mb, config.requestAddress,
                                       config.readLength,
                                       responseHalfword.data()) == -1)
            {
                std::cerr << name << ": Failed to read discrete inputs. "
                          << modbus_strerror(errno) << std::endl;
            }
            break;

        // Function code: 3
        case (READ_HOLDING_REGISTER):
            if (modbus_read_registers(mb, config.requestAddress,
                                      config.readLength, responseWord.data()) ==
                -1)
            {
                std::cerr << name << ": Failed to read holding register. "
                          << modbus_strerror(errno) << std::endl;
            }
            break;

        // Function code: 4
        case (READ_INPUT_REGISTERS):
            if (modbus_read_input_registers(mb, config.requestAddress,
                                            config.readLength,
                                            responseWord.data()) == -1)
            {
                std::cerr << name << ": Failed to read input register. "
                          << modbus_strerror(errno) << std::endl;
            }
            break;

        // Function code: 6
        case (WRITE_SINGLE_REGISTER):
            if (modbus_write_register(mb, config.requestAddress,
                                      config.requestValue) == -1)
            {
                std::cerr << name << ": Failed to write single register. "
                          << modbus_strerror(errno) << std::endl;
            }
            break;

        default:
            if constexpr (debug)
            {
                std::cerr << name
                          << ": No matching function code found for requesting"
                          << std::endl;
            }
            break;
    }
}

void ModbusSensor::routine()
{
    static constexpr size_t pollTime = 10; // in seconds

    waitTimer.expires_after(std::chrono::seconds(pollTime));
    waitTimer.async_wait([this](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
            return;
        if (ec)
        {
            std::cerr << "timer error" << std::endl;
            return;
        }

        if (connected)
        {
            RequestQueue& requestQueue = getRequestQueue(config);
            requestQueue.addRequest(this);

            switch (config.requestFunctionCode)
            {
                // Function code: 2, 3 or 4
                case (READ_DISCRETE_INPUTS):
                case (READ_HOLDING_REGISTER):
                case (READ_INPUT_REGISTERS):
                    setSensorValue();
                    break;

                // Function code: 6
                case (WRITE_SINGLE_REGISTER):
                    break;

                default:
                    if constexpr (debug)
                    {
                        std::cerr
                            << name
                            << ": No matching function code found for routine."
                            << std::endl;
                    }
                    break;
            }
        }
        else
            startConnection();

        routine();
    });
}

int ModbusSensor::setupModbusConnection()
{
    if (config.protocol == PROTOCOL_RTU)
    {
        mb = modbus_new_rtu(config.device.data(), config.baud,
                            config.parity.at(0), config.dataBit,
                            config.stopBit);
        if (mb == NULL)
        {
            std::cerr << name << ": The Modbus RTU object cannot be created."
                      << std::endl;
            return -1;
        }
    }
    else if (config.protocol == PROTOCOL_TCP)
    {
        mb = modbus_new_tcp(config.ipAddress.data(), config.portNumber);

        if (mb == NULL)
        {
            std::cerr << name << ": The Modbus TCP object cannot be created."
                      << std::endl;
            return -1;
        }
    }
    else
    {
        std::cerr << name << ": No matching protocol." << std::endl;
        return -1;
    }

    if (modbus_set_slave(mb, config.deviceId) == -1)
    {
        std::cerr << name << ": Failed to set device ID." << std::endl;
        return -1;
    }

    if (modbus_connect(mb) == -1)
    {
        std::cerr << name << ": Connection failed." << std::endl;
        return -1;
    }

    if (modbus_set_response_timeout(mb, TIMEOUT_SEC, TIMEOUT_USEC) == -1)
    {
        std::cerr << name << ": Failed to set timeout." << std::endl;
        return -1;
    }

    return 0;
}

void ModbusSensor::startConnection()
{
    connected = (setupModbusConnection() == 0);
}

void ModbusSensor::init()
{
    if (config.readTwoWords)
        config.readLength = 2;

    switch (config.requestFunctionCode)
    {
        // Function code: 2
        case (READ_DISCRETE_INPUTS):
            responseHalfword.clear();
            responseHalfword.resize(config.readLength);
            break;

        // Function code: 3 or 4
        case (READ_HOLDING_REGISTER):
        case (READ_INPUT_REGISTERS):
            responseWord.clear();
            responseWord.resize(config.readLength);
            break;

        default:
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
    boost::container::flat_map<std::string, std::shared_ptr<ModbusSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    if (!dbusConnection)
    {
        std::cerr << "Connection not created" << std::endl;
        return;
    }

    dbusConnection->async_method_call(
        [&io, &objectServer, &dbusConnection, &sensors](
            boost::system::error_code ec, const ManagedObjectType& resp) {
            if (ec)
            {
                std::cerr << "Error contacting entity manager" << std::endl;
                return;
            }

            for (const auto& [path, interfaces] : resp)
            {
                for (const auto& [intf, cfg] : interfaces)
                {
                    if (intf != configInterfaceName(sensorType))
                        continue;

                    std::string name = loadVariant<std::string>(cfg, "Name");

                    struct ModbusConfig config;

                    config.protocol = loadVariantWithDefault<std::string>(
                        cfg, "Protocol", "");
                    config.deviceId =
                        loadVariantWithDefault<uint8_t>(cfg, "DeviceID", 0);
                    config.ipAddress =
                        loadVariantWithDefault<std::string>(cfg, "IP", "");
                    config.portNumber =
                        loadVariantWithDefault<uint16_t>(cfg, "Port", 0);
                    config.device =
                        loadVariantWithDefault<std::string>(cfg, "Device", "");
                    config.baud =
                        loadVariantWithDefault<uint32_t>(cfg, "Baud", 0);
                    config.parity =
                        loadVariantWithDefault<std::string>(cfg, "Parity", "");
                    config.dataBit =
                        loadVariantWithDefault<uint16_t>(cfg, "DataBit", 0);
                    config.stopBit =
                        loadVariantWithDefault<uint16_t>(cfg, "StopBit", 0);
                    config.requestFunctionCode =
                        loadVariantWithDefault<uint16_t>(
                            cfg, "RequestFunctionCode", 3);
                    config.requestAddress = loadVariantWithDefault<uint16_t>(
                        cfg, "RequestAddress", 0);
                    config.requestQuantity = loadVariantWithDefault<uint16_t>(
                        cfg, "RequestQuantity", 2);
                    config.requestByteCount = loadVariantWithDefault<uint16_t>(
                        cfg, "RequestByteCount", 1);
                    config.requestValue = loadVariantWithDefault<uint16_t>(
                        cfg, "RequestValue", 0);
                    config.readLength =
                        loadVariantWithDefault<uint16_t>(cfg, "ReadLength", 1);
                    config.scaleFactor =
                        loadVariantWithDefault<double>(cfg, "ScaleFactor", 1.0);
                    config.signedInteger =
                        loadVariantWithDefault<bool>(cfg, "Signed", false);
                    config.readTwoWords = loadVariantWithDefault<bool>(
                        cfg, "ReadTwoWords", false);
                    config.sensorUnit =
                        loadVariantWithDefault<std::string>(cfg, "Unit", "");

                    std::vector<thresholds::Threshold> sensorThresholds;
                    if (!parseThresholdsFromConfig(interfaces,
                                                   sensorThresholds))
                    {
                        std::cerr << "error populating thresholds for " << name
                                  << std::endl;
                    }

                    if constexpr (debug)
                    {
                        std::cout
                            << "Configuration parsed for \n\t" << intf << "\n"
                            << "with\n"
                            << "\tName: " << name << "\n"
                            << "\tProtocol: " << config.protocol << "\n"
                            << "\tdeviceId: "
                            << static_cast<int>(config.deviceId) << "\n"
                            << "\tIpAddress: " << config.ipAddress << "\n"
                            << "\tPortNumber: "
                            << static_cast<int>(config.portNumber) << "\n"
                            << "\tDevice: " << config.device << "\n"
                            << "\tBaud: " << static_cast<int>(config.baud)
                            << "\n"
                            << "\tParity: " << config.parity << "\n"
                            << "\tDataBitr: "
                            << static_cast<int>(config.dataBit) << "\n"
                            << "\tStopBit: " << static_cast<int>(config.stopBit)
                            << "\n"
                            << "\tRequestFunctionCode: "
                            << static_cast<int>(config.requestFunctionCode)
                            << "\n"
                            << "\tRequestAddress: "
                            << static_cast<int>(config.requestAddress) << "\n"
                            << "\tRequestQuantity: "
                            << static_cast<int>(config.requestQuantity) << "\n"
                            << "\tRequestByteCount: "
                            << static_cast<int>(config.requestByteCount) << "\n"
                            << "\tReadLength: "
                            << static_cast<int>(config.readLength) << "\n"
                            << "\tScaleFactor: " << config.scaleFactor << "\n"
                            << "\tSigned: " << config.signedInteger << "\n"
                            << "\tReadTwoWords: " << config.readTwoWords << "\n"
                            << "\tUnit: " << config.sensorUnit << "\n"
                            << "\tRequestValue: " << config.requestValue
                            << "\n";
                    }

                    getRequestQueue(config);

                    std::pair<double, double> limits =
                        std::make_pair(setMinReading(config.signedInteger,
                                                     config.readTwoWords),
                                       setMaxReading(config.signedInteger,
                                                     config.readTwoWords));

                    findLimits(limits, &(*interfaces.find(intf)));

                    auto& sensor = sensors[name];

                    sensor = std::make_shared<ModbusSensor>(
                        dbusConnection, io, name, path, objectServer,
                        std::move(sensorThresholds), limits, std::move(config));

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

    boost::asio::post(io, [&]() {
        createSensors(io, objectServer, sensors, systemBus);
    });

    boost::asio::steady_timer configTimer(io);

    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&](sdbusplus::message_t&) {
            configTimer.expires_after(std::chrono::seconds(1));
            configTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                    return;
                if (ec)
                {
                    std::cerr << "timer error" << std::endl;
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
