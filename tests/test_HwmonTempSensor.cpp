#include "../src/HwmonTempSensor.hpp"

#include <boost/asio/read_until.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

// #include "dbus/connection.hpp"

#include <nlohmann/json.hpp>

#include <fstream>

#include <gtest/gtest.h>

TEST(HwmonTempSensor, TestTMP75)
{
    boost::asio::io_context io;
    auto system_bus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server object_server(system_bus, true);

    std::vector<thresholds::Threshold> sensor_thresholds;
    auto t = thresholds::Threshold(thresholds::Level::CRITICAL,
                                   thresholds::Direction::LOW, 80);
    sensor_thresholds.emplace_back(t);

    std::ofstream test_file("test0.txt");
    test_file << "28\n";
    test_file.close();
    auto filename = std::string("test0.txt");
    auto tempsensname = std::string("test sensor");
    auto object_type = std::string("");
    struct SensorParams sp = {.minValue = -100,
                              .maxValue = 100,
                              .offsetValue = 0.0,
                              .scaleValue = 1.0,
                              .units = sensor_paths::unitDegreesC,
                              .typeName = "temperature"};
    HwmonTempSensor test(filename, object_type, object_server, system_bus, io,
                         tempsensname, std::move(sensor_thresholds), sp, 1.0,
                         "", PowerState::on, nullptr);

    std::remove("test0.txt");
}

TEST(HwmonTempSensor, TestTMP421)
{
    boost::asio::io_context io;
    auto system_bus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server object_server(system_bus, true);

    std::vector<thresholds::Threshold> sensor_thresholds;
    auto t = thresholds::Threshold(thresholds::Level::WARNING,
                                   thresholds::Direction::HIGH, 80);
    sensor_thresholds.emplace_back(t);

    std::ofstream test_file("test1.txt");
    test_file << "28\n";
    test_file.close();
    auto filename = std::string("test1.txt");
    auto tempsensname = std::string("test sensor");
    auto object_type = std::string("");
    struct SensorParams sp = {.minValue = -100,
                              .maxValue = 100,
                              .offsetValue = 0.0,
                              .scaleValue = 1.0,
                              .units = sensor_paths::unitDegreesC,
                              .typeName = "temperature"};
    HwmonTempSensor test(filename, object_type, object_server, system_bus, io,
                         tempsensname, std::move(sensor_thresholds), sp, 1.0,
                         "", PowerState::on, nullptr);

    std::remove("test1.txt");
}
