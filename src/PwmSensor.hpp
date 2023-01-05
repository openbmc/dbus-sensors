#pragma once

#include "sensor.hpp"

#include <sdbusplus/asio/object_server.hpp>

#include <memory>
#include <string>

class PwmSensor
{
  public:
    PwmSensor(const std::string& name, const std::vector<std::string>& sysPaths,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              sdbusplus::asio::object_server& objectServer,
              const std::string& sensorConfiguration,
              const std::string& sensorType, bool isValueMutable = false);
    ~PwmSensor();

  private:
    std::vector<std::string> sysPaths;
    sdbusplus::asio::object_server& objectServer;
    std::string name;
    std::string sensorType;
    std::shared_ptr<sdbusplus::asio::dbus_interface> sensorInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> controlInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> association;
    std::shared_ptr<sdbusplus::asio::dbus_interface> valueMutabilityInterface;
    double pwmMax;
    void setValue(uint32_t value);
    uint32_t getValue(bool errThrow = true);
};
