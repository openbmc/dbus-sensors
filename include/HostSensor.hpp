#pragma once

#include "Thresholds.hpp"
#include "sensor.hpp"

#include <gpiod.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

class HostSensor :
    public Sensor,
    public std::enable_shared_from_this<HostSensor>
{
  public:
    HostSensor(const std::string& path,
               sdbusplus::asio::object_server& objectServer,
               std::shared_ptr<sdbusplus::asio::connection>& conn,
               boost::asio::io_service& io, const std::string& sensorName,
               std::vector<thresholds::Threshold>&& _thresholds,
               const std::string& sensorConfiguration);
    ~HostSensor();

  private:
    sdbusplus::asio::object_server& objServer;
    std::string path;
    void checkThresholds(void);
};
