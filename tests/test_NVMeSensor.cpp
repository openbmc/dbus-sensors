#include <NVMeSensor.hpp>
#include <i2c.h>
#include <smbus.hpp>
#include <dbus/connection.hpp>
#include <nlohmann/json.hpp>

#include <fstream>

#include "gtest/gtest.h"

TEST(NVMeSensor, TestNVMeSensor)
{
    boost::asio::io_service io;
    auto system_bus =
        std::make_shared<dbus::connection>(io, dbus::bus::session);
    dbus::DbusObjectServer object_server(system_bus);

    std::vector<thresholds::Threshold> sensor_thresholds;
    auto t = thresholds::Threshold(thresholds::Level::CRITICAL,
                                   thresholds::Direction::HIGH, 60);
    sensor_thresholds.emplace_back(t);
}
