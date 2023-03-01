#pragma once
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <sensor.hpp>

#include <chrono>
#include <limits>
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

    void checkThresholds(void) override;
    void read(void);
    void init(void);

    uint8_t busId;
    uint8_t mcuAddress;
    uint8_t tempReg;

  private:
    int getMCURegsInfoWord(uint8_t regs, int16_t* pu16data) const;
    sdbusplus::asio::object_server& objectServer;
    boost::asio::steady_timer waitTimer;
};
