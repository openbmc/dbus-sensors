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
  public:
    PLDMSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
               boost::asio::io_service& io, const std::string& name,
               const std::string& sensorPath, const std::string& sensorType,
               sdbusplus::asio::object_server& objectServer,
               std::vector<thresholds::Threshold>&& thresholds,
               const std::string& sensorUnit, uint8_t eid, uint16_t sensorId,
               uint8_t instanceId, const float pollRate, float resolution,
               float offset, double maxValue, double minValue);
    ~PLDMSensor() override;

    void checkThresholds(void) override;
    void read(void);
    void init(void);

  private:
    uint8_t eid;
    uint8_t sensorId;
    uint8_t instanceId;
    uint8_t sensorDataSize;
    int sensorPollMs;
    float resolution;
    float offset;
    sdbusplus::asio::object_server& objectServer;
    boost::asio::deadline_timer waitTimer;
};

bool getSensorReading(uint8_t eid, uint8_t instanceId, uint8_t sensorId,
                      int32_t* reading, uint8_t* dataSize);