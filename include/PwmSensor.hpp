#pragma once

#include <sdbusplus/asio/object_server.hpp>

class PwmSensor
{
  public:
    PwmSensor(const std::string& sysPath,
              sdbusplus::asio::object_server& objectServer);
    ~PwmSensor();

  private:
    std::string sysPath;
    sdbusplus::asio::object_server& objectServer;
    std::string name;
    std::shared_ptr<sdbusplus::asio::dbus_interface> sensorInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> controlInterface;
    void setValue(uint32_t value);
    uint32_t getValue(bool errThrow = true);
};
