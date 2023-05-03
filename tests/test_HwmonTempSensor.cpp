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
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus, true);

    std::vector<thresholds::Threshold> sensorThresholds;
    auto t = thresholds::Threshold(thresholds::Level::CRITICAL,
                                   thresholds::Direction::LOW, 80);
    sensorThresholds.emplace_back(t);

    std::ofstream testFile("test0.txt");
    testFile << "28\n";
    testFile.close();
    auto filename = std::string("test0.txt");
    auto tempsensname = std::string("test sensor");
    auto objectType = std::string("");
    struct SensorParams sp = {.minValue = -100,
                              .maxValue = 100,
                              .offsetValue = 0.0,
                              .scaleValue = 1.0,
                              .units = sensor_paths::unitDegreesC,
                              .typeName = "temperature"};
    HwmonTempSensor test(filename, objectType, objectServer, systemBus, io,
                         tempsensname, std::move(sensorThresholds), sp, 1.0, "",
                         PowerState::on, nullptr);

    std::remove("test0.txt");
}

TEST(HwmonTempSensor, TestTMP421)
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus, true);

    std::vector<thresholds::Threshold> sensorThresholds;
    auto t = thresholds::Threshold(thresholds::Level::WARNING,
                                   thresholds::Direction::HIGH, 80);
    sensorThresholds.emplace_back(t);

    std::ofstream testFile("test1.txt");
    testFile << "28\n";
    testFile.close();
    auto filename = std::string("test1.txt");
    auto tempsensname = std::string("test sensor");
    auto objectType = std::string("");
    struct SensorParams sp = {.minValue = -100,
                              .maxValue = 100,
                              .offsetValue = 0.0,
                              .scaleValue = 1.0,
                              .units = sensor_paths::unitDegreesC,
                              .typeName = "temperature"};
    HwmonTempSensor test(filename, objectType, objectServer, systemBus, io,
                         tempsensname, std::move(sensorThresholds), sp, 1.0, "",
                         PowerState::on, nullptr);

    std::remove("test1.txt");
}
