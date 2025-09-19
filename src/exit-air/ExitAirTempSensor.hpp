#pragma once
#include "asio/Thresholds.hpp"
#include "asio/sensor.hpp"

#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

struct ExitAirTempSensor;
struct CFMSensor : public Sensor, std::enable_shared_from_this<CFMSensor>
{
    std::vector<std::string> tachs;
    double c1 = 0.0;
    double c2 = 0.0;
    double maxCFM = 0.0;
    double tachMinPercent = 0.0;
    double tachMaxPercent = 0.0;

    std::shared_ptr<ExitAirTempSensor> parent;

    CFMSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
              const std::string& name, const std::string& sensorConfiguration,
              sdbusplus::asio::object_server& objectServer,
              std::vector<thresholds::Threshold>&& thresholdData,
              std::shared_ptr<ExitAirTempSensor>& parent);
    ~CFMSensor() override;

    bool calculate(double& /*value*/);
    void updateReading();
    void setupMatches();
    void createMaxCFMIface();
    void addTachRanges(const std::string& serviceName, const std::string& path);
    void checkThresholds() override;
    uint64_t getMaxRpm(uint64_t cfmMax) const;

  private:
    std::vector<sdbusplus::bus::match_t> matches;
    boost::container::flat_map<std::string, double> tachReadings;
    boost::container::flat_map<std::string, std::pair<double, double>>
        tachRanges;
    std::shared_ptr<sdbusplus::asio::dbus_interface> pwmLimitIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> cfmLimitIface;
    sdbusplus::asio::object_server& objServer;
};

struct ExitAirTempSensor :
    public Sensor,
    std::enable_shared_from_this<ExitAirTempSensor>
{
    double powerFactorMin = 0.0;
    double powerFactorMax = 0.0;
    double qMin = 0.0;
    double qMax = 0.0;
    double alphaS = 0.0;
    double alphaF = 0.0;
    double pOffset = 0.0;

    ExitAirTempSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                      const std::string& name,
                      const std::string& sensorConfiguration,
                      sdbusplus::asio::object_server& objectServer,
                      std::vector<thresholds::Threshold>&& thresholdData);
    ~ExitAirTempSensor() override;

    void checkThresholds() override;
    void updateReading();
    void setupMatches();

  private:
    double lastReading = 0.0;

    std::vector<sdbusplus::bus::match_t> matches;
    double inletTemp = std::numeric_limits<double>::quiet_NaN();
    boost::container::flat_map<std::string, double> powerReadings;

    sdbusplus::asio::object_server& objServer;
    std::chrono::time_point<std::chrono::steady_clock> lastTime;
    static double getTotalCFM();
    bool calculate(double& val);
};
