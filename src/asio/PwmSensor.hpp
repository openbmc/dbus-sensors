#pragma once

#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <string>

class PwmSensor
{
  public:
    PwmSensor(const std::string& pwmname, const std::string& sysPath,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              sdbusplus::asio::object_server& objectServer,
              const std::string& sensorConfiguration,
              const std::string& sensorType, bool isValueMutable = false);
    ~PwmSensor();

  private:
    std::string sysPath;
    sdbusplus::asio::object_server& objectServer;
    std::string name;
    std::shared_ptr<sdbusplus::asio::dbus_interface> sensorInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> controlInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> association;
    std::shared_ptr<sdbusplus::asio::dbus_interface> valueMutabilityInterface;
    double pwmMax;
    void setValue(uint32_t value);
    uint32_t getValue(bool errThrow = true);
};
