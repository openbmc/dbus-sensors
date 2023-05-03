#include "../src/TachSensor.hpp"
#include "../src/Thresholds.hpp"

#include <boost/asio/read_until.hpp>

#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

//#include <dbus/connection.hpp>
#include <nlohmann/json.hpp>

#include <fstream>

#include <gtest/gtest.h>

TEST(TachSensor, TestTachSensor)
{
    boost::asio::io_context io;
    auto system_bus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server object_server(system_bus, true);

    std::vector<thresholds::Threshold> sensor_thresholds;
    auto t = thresholds::Threshold(thresholds::Level::CRITICAL,
                                   thresholds::Direction::LOW, 1000);
    sensor_thresholds.emplace_back(t);

    std::ofstream test_file("test.txt");
    test_file << "10000\n";
    test_file.close();
    auto filename = std::string("test.txt");
    auto fanname = std::string("test fan");
    std::pair<double,double> limits;
    limits.first = 1.0;
    limits.second = 1.0;

    TachSensor test(filename, "", object_server, system_bus, nullptr, nullptr, io, fanname,
                    std::move(sensor_thresholds), "",
                    limits, PowerState::on, std::nullopt);

    std::remove("test.txt");
}
