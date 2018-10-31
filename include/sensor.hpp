#pragma once

#include <Thresholds.hpp>
#include <sdbusplus/asio/object_server.hpp>

constexpr size_t sensorFailedPollTimeMs = 5000;
struct Sensor
{
    Sensor(const std::string& name, const std::string& path,
           std::vector<thresholds::Threshold>&& thresholdData) :
        name(name),
        path(path), thresholds(std::move(thresholdData))
    {
    }
    virtual ~Sensor() = default;
    std::string name;
    std::string path;
    std::vector<thresholds::Threshold> thresholds;
    std::shared_ptr<sdbusplus::asio::dbus_interface> sensorInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> thresholdInterfaceWarning;
    std::shared_ptr<sdbusplus::asio::dbus_interface> thresholdInterfaceCritical;
    double value = std::numeric_limits<double>::quiet_NaN();
};