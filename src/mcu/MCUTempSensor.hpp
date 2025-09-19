#pragma once
#include "asio/Thresholds.hpp"
#include "asio/sensor.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

struct MCUTempSensor : public Sensor
{
    MCUTempSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                  boost::asio::io_context& io, const std::string& name,
                  const std::string& sensorConfiguration,
                  sdbusplus::asio::object_server& objectServer,
                  std::vector<thresholds::Threshold>&& thresholdData,
                  uint8_t busId, uint8_t mcuAddress, uint8_t tempReg);
    ~MCUTempSensor() override;

    void checkThresholds() override;
    void read();
    void init();

    uint8_t busId;
    uint8_t mcuAddress;
    uint8_t tempReg;

  private:
    int getMCURegsInfoWord(uint8_t regs, int32_t* pu32data) const;
    sdbusplus::asio::object_server& objectServer;
    boost::asio::steady_timer waitTimer;
};
