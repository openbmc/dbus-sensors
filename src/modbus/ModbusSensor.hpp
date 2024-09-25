#pragma once

#include <boost/asio/io_context.hpp>
#include <sensor.hpp>

#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

extern "C"
{
#include <modbus.h>
}

using word = std::vector<uint16_t>;
using halfword = std::vector<uint8_t>;

class RequestQueue;

struct ModbusConfig
{
    /**
     * @brief Modbus Configurations
     *
     * @param protocol
     *  RTU or TCP. RTU is used with RS-485 serial connections, while TCP is
     * used over TCP/IP networks.
     * @param deviceId
     *  It is a unique identifier assigned to each device on a Modbus network.
     * @param ipAddress
     *  The setup for TCP communication with Modbus devices.
     * @param portNumber
     *  The setup for TCP communication with Modbus devices.
     * @param device
     *  The device is where the Modbus device file is stored under the /dev
     * folder for RTU communication.
     * @param baud
     *  The setup for RTU communication with Modbus devices.
     * @param parity
     *  The setup for RTU communication with Modbus devices.
     * @param dataBit
     *  The setup for RTU communication with Modbus devices.
     * @param stopBit
     *  The setup for RTU communication with Modbus devices.
     * @param requestFunctionCode
     *  It specifies the type of action the client wants to perform on the
     * server.
     * @param requestAddress
     *  It specifies the starting register address for the request.
     * @param requestQuantity
     *  It specifies the quantity of consecutive data registers being read or
     * written in a single request.
     * @param requestByteCount
     *  It specifies the quantity of complete bytes of data for the request.
     * @param requestValue
     *  It specifies the data to be stored within a Modbus register in order to
     * write a value.
     * @param readLength
     *  It specifies the number of registers to read; a value of 1 means reading
     * the value of a single register.
     * @param scaleFactor
     *  It specifies the scaling factor that maps the retrieved value to its
     * corresponding unit.
     * @param signedInteger
     *  It specifies whether to explicitly cast uint16_t to int16_t.
     * @param readTwoWords
     *  It specifies whether to combine two registers into a single value.
     * @param sensorUnit
     *  It specifies the unit of the sensor value.
     */

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
    uint16_t readLength;
    double scaleFactor;
    bool signedInteger;
    bool readTwoWords;
    std::string sensorUnit;
};

class ModbusSensor :
    public Sensor,
    public std::enable_shared_from_this<ModbusSensor>
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
    void request();
    void init();

  private:
    void setSensorValue();
    void routine();
    int setupModbusConnection();
    void startConnection();
    int newModbusConnection();

    modbus_t* mb;
    ModbusConfig config;
    bool connected = false;
    word responseWord;
    halfword responseHalfword;

    sdbusplus::asio::object_server& objectServer;
    boost::asio::steady_timer waitTimer;
};

class RequestQueue
{
  public:
    void addRequest(std::shared_ptr<ModbusSensor> instance)
    {
        std::unique_lock<std::mutex> lock(mtx);
        queue.push(instance);
        cv.notify_one();
    }

    void processRequests()
    {
        while (true)
        {
            std::shared_ptr<ModbusSensor> instance = nullptr;
            {
                std::unique_lock<std::mutex> lock(mtx);
                cv.wait(lock, [this]() { return !queue.empty(); });
                instance = queue.front();
                queue.pop();
            }

            if (instance)
                instance->request();
        }
    }

  private:
    std::queue<std::shared_ptr<ModbusSensor>> queue;
    std::mutex mtx;
    std::condition_variable cv;
};
