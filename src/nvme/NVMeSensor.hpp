#pragma once

#include "Thresholds.hpp"

#include <boost/asio/io_context.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

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
               uint8_t slaveAddr, bool smbusPEC);
    ~NVMeSensor() override;

    NVMeSensor& operator=(const NVMeSensor& other) = delete;

    bool sample();

    const int bus;
    const uint8_t address;
    const bool smbusPEC;

  private:
    const unsigned int scanDelayTicks = 5 * 60;
    sdbusplus::asio::object_server& objServer;
    unsigned int scanDelay{0};

    void checkThresholds() override;
};
