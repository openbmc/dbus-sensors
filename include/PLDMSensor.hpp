#pragma once
#include <boost/asio/deadline_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <sensor.hpp>

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

struct PLDMSensor : public Sensor
{
    PLDMSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                  boost::asio::io_service& io, const std::string& name,
                  const std::string& sensorConfiguration,
                  sdbusplus::asio::object_server& objectServer,
                  std::vector<thresholds::Threshold>&& thresholds,
                  const std::string& sensorType,
                  const std::string& sensorUnit,
                  const std::string& DataSize,
                  uint8_t eid, uint16_t sensorId, uint8_t instanceId);
    ~PLDMSensor() override;

    void checkThresholds(void) override;
    void read(void);
    void init(void);

private:
    int pldmSendRecv(uint8_t eid, std::vector<uint8_t>& requestMsg, std::vector<uint8_t>& responseMsg);
    int getSensorReading(uint8_t sensorId, int32_t* reading);
    uint8_t eid;
    uint8_t sensorId;
    uint8_t sensorDataSize;
    uint8_t instanceId;
    std::string sensorPath;
    sdbusplus::asio::object_server& objectServer;
    boost::asio::deadline_timer waitTimer;
};
