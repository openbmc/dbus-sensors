#pragma once

#include <boost/asio/io_context.hpp>
#include <sensor.hpp>

class NVMeSensor : public Sensor
{
  public:
    static constexpr const char* sensorType = "NVME1000";

    NVMeSensor(sdbusplus::asio::object_server& objectServer,
               boost::asio::io_context& io,
               std::shared_ptr<sdbusplus::asio::connection>& conn,
               const std::string& sensorName,
               std::vector<thresholds::Threshold>&& thresholds,
               const std::string& sensorConfiguration, int busNumber,
               uint8_t slaveAddr);
    ~NVMeSensor() override;

    NVMeSensor& operator=(const NVMeSensor& other) = delete;

    bool sample();

    int bus;
    uint8_t address;

  private:
    const unsigned int scanDelayTicks = 5 * 60;
    sdbusplus::asio::object_server& objServer;
    unsigned int scanDelay{0};

    void checkThresholds(void) override;
};
