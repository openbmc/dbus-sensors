#pragma once

#include <Thresholds.hpp>
#include <sdbusplus/asio/object_server.hpp>

struct Sensor
{
    virtual ~Sensor() = default;
    std::vector<thresholds::Threshold> thresholds;
    std::shared_ptr<sdbusplus::asio::dbus_interface> sensorInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> thresholdInterfaceWarning;
    std::shared_ptr<sdbusplus::asio::dbus_interface> thresholdInterfaceCritical;
    double value = std::numeric_limits<double>::quiet_NaN();
};