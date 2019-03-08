#pragma once
#include "sensor.hpp"

#include <boost/container/flat_map.hpp>
#include <chrono>
#include <limits>
#include <vector>

struct ExitAirTempSensor;
struct CFMSensor : public Sensor
{
    std::vector<std::string> tachs;
    int32_t c1;
    int32_t c2;
    int32_t maxCFM;
    double tachMinPercent;
    double tachMaxPercent;

    std::shared_ptr<ExitAirTempSensor> parent;

    CFMSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
              const std::string& name, const std::string& sensorConfiguration,
              sdbusplus::asio::object_server& objectServer,
              std::vector<thresholds::Threshold>&& thresholds,
              std::shared_ptr<ExitAirTempSensor>& parent);
    ~CFMSensor();

    bool calculate(double&);
    void updateReading(void);
    void createMaxCFMIface(void);
    void checkThresholds(void) override;
    uint64_t getMaxRpm(uint64_t cfmMax);

  private:
    std::vector<sdbusplus::bus::match::match> matches;
    boost::container::flat_map<std::string, double> tachReadings;
    boost::container::flat_map<std::string, std::pair<double, double>>
        tachRanges;
    std::shared_ptr<sdbusplus::asio::connection> dbusConnection;
    std::shared_ptr<sdbusplus::asio::dbus_interface> pwmLimitIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> cfmLimitIface;
    sdbusplus::asio::object_server& objServer;
    void addTachRanges(const std::string& serviceName, const std::string& path);
};

struct ExitAirTempSensor : public Sensor
{

    double powerFactorMin;
    double powerFactorMax;
    double qMin;
    double qMax;
    double alphaS;
    double alphaF;
    double pOffset = 0;

    // todo: make this private once we don't have to hack in a reading
    boost::container::flat_map<std::string, double> powerReadings;

    std::vector<std::unique_ptr<CFMSensor>> cfmSensors;
    ExitAirTempSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                      const std::string& name,
                      const std::string& sensorConfiguration,
                      sdbusplus::asio::object_server& objectServer,
                      std::vector<thresholds::Threshold>&& thresholds);
    ~ExitAirTempSensor();

    void checkThresholds(void) override;
    void updateReading(void);

  private:
    double lastReading;

    std::vector<sdbusplus::bus::match::match> matches;
    double inletTemp = std::numeric_limits<double>::quiet_NaN();

    std::shared_ptr<sdbusplus::asio::connection> dbusConnection;
    sdbusplus::asio::object_server& objServer;
    std::chrono::time_point<std::chrono::system_clock> lastTime;
    double getTotalCFM(void);
    bool calculate(double& val);
    void setupMatches(void);
};
