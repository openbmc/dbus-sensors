#pragma once

#include "Thresholds.hpp"
#include "sensor.hpp"

#include <sdbusplus/asio/object_server.hpp>

#include <string>
#include <vector>

class ExternalSensor :
    public Sensor,
    public std::enable_shared_from_this<ExternalSensor>
{
  public:
    ExternalSensor(const std::string& objectType,
                   sdbusplus::asio::object_server& objectServer,
                   std::shared_ptr<sdbusplus::asio::connection>& conn,
                   const std::string& sensorName,
                   std::vector<thresholds::Threshold>&& thresholds,
                   const std::string& sensorConfiguration,
                   const double& maxReading, const double& minReading,
                   const std::string_view& sensorUnits,
                   const PowerState& powerState);
    virtual ~ExternalSensor();

  private:
    sdbusplus::asio::object_server& objServer;

    void checkThresholds(void) override;
};
