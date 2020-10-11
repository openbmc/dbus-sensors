#pragma once

#include "Thresholds.hpp"
#include "sensor.hpp"

#include <sdbusplus/asio/object_server.hpp>

#include <string>
#include <vector>

class ExternalSensor
{
  public:
    ExternalSensor(const std::string& objectType,
                   sdbusplus::asio::object_server& objectServer,
                   std::shared_ptr<sdbusplus::asio::connection>& conn,
                   const std::string& sensorName,
                   const std::string& sensorMeasure,
                   std::vector<thresholds::Threshold>&& thresholds,
                   const std::string& sensorConfiguration,
                   const double& maxReading, const double& minReading,
                   const PowerState& powerState);
    virtual ~ExternalSensor();

    Sensor sensor;

  private:
    sdbusplus::asio::object_server& objServer;

    void checkThresholds();
};
