#pragma once

#include <sensor.hpp>

#include <memory>
#include <string>
#include <vector>

extern "C"
{
#include <modbus.h>
}

using word = std::vector<uint16_t>;
using halfword = std::vector<uint8_t>;

struct ModbusConfig
{
    std::string protocol;
    uint8_t deviceId;
    std::string ipAddress;
    uint16_t portNumber;
    std::string device;
    uint32_t baud;
    std::string parity;
    uint8_t dataBit;
    uint8_t stopBit;
    uint16_t requestFunctionCode;
    uint16_t requestAddress;
    uint16_t requestQuantity;
    uint16_t requestByteCount;
    uint16_t requestValue;
    uint16_t readLength; // A readLength value of 1 represents one register.
    double scaleFactor;
    bool signedInteger;
    bool readTwoWords; // Combine two registers into a single value.
    std::string sensorUnit;
};

class ModbusSensor : public Sensor
{
  public:
    ModbusSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                 boost::asio::io_context& io, const std::string& name,
                 const std::string& sensorConfiguration,
                 sdbusplus::asio::object_server& objectServer,
                 std::vector<thresholds::Threshold>&& thresholdData,
                 const std::pair<double, double>& limits,
                 const struct ModbusConfig&& config);
    ~ModbusSensor() override;

    void checkThresholds() override;
    void init();

  private:
    void setSensorValue();
    int request();
    void routine();
    int setupModbusConnection();
    int startConnection();

    modbus_t* mb;

    ModbusConfig config;

    word responseWord;
    halfword responseHalfword;

    sdbusplus::asio::object_server& objectServer;
    boost::asio::steady_timer waitTimer;
};
