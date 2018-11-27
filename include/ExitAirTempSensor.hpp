#pragma once
#include "sensor.hpp"

#include <boost/container/flat_map.hpp>
#include <chrono>
#include <limits>
#include <vector>

struct CFMInfo
{
    std::vector<std::string> pwm;
    int32_t c1;
    int32_t c2;
    int32_t maxCFM;
    double pwmMin;
    double pwmMax;
};

struct ExitAirTempSensor : public Sensor
{
    boost::container::flat_map<std::string, double> pwmReadings;
    boost::container::flat_map<std::string, double> powerReadings;
    std::vector<sdbusplus::bus::match::match> matches;

    double inletTemp = std::numeric_limits<double>::quiet_NaN();

    double powerFactorMin;
    double powerFactorMax;
    double qMin;
    double qMax;
    double alphaS;
    double alphaF;
    double pOffset = 0;

    std::vector<CFMInfo> cfmData;
    ExitAirTempSensor(std::shared_ptr<sdbusplus::asio::connection> &conn,
                      const std::string &sensorConfiguration,
                      sdbusplus::asio::object_server &objectServer,
                      std::vector<thresholds::Threshold> &&thresholds);
    ~ExitAirTempSensor();

    void checkThresholds(void) override;
    void updateReading(void);

  private:
    double lastReading;

    std::shared_ptr<sdbusplus::asio::connection> dbusConnection;
    std::chrono::time_point<std::chrono::system_clock> lastTime;
    int32_t getTotalCFM(void);
    bool calculate(double &val);
    void setupMatches(void);
};