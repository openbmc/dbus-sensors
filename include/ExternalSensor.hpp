#pragma once

#include "Thresholds.hpp"
#include "sensor.hpp"

#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <string>
#include <vector>

class ExternalSensor :
    public Sensor,
    public std::enable_shared_from_this<ExternalSensor>
{
  public:
    ExternalSensor(
        const std::string& objectType,
        sdbusplus::asio::object_server& objectServer,
        std::shared_ptr<sdbusplus::asio::connection>& conn,
        const std::string& sensorName, const std::string& sensorMeasure,
        std::vector<thresholds::Threshold>&& _thresholds,
        const std::string& sensorConfiguration, const double& maxReading,
        const double& minReading, const double& timeoutSecs,
        const PowerState& powerState,
        std::function<void(const std::chrono::steady_clock::time_point& now)>&&
            _writeHook);
    virtual ~ExternalSensor();

    bool isPerishable(void) const;
    bool isViable(const std::chrono::steady_clock::time_point& now) const;
    void writeBegin(const std::chrono::steady_clock::time_point& now);
    void writeInvalidate(void);
    std::chrono::steady_clock::duration
        ageElapsed(const std::chrono::steady_clock::time_point& now) const;
    std::chrono::steady_clock::duration
        ageRemaining(const std::chrono::steady_clock::time_point& now) const;

  private:
    sdbusplus::asio::object_server& objServer;

    std::chrono::steady_clock::time_point writeLast;
    std::chrono::steady_clock::duration writeTimeout;
    bool writeAlive;
    bool writePerishable;
    std::function<void(const std::chrono::steady_clock::time_point& now)>
        writeHook;

    void checkThresholds(void) override;
    void externalSetTrigger(void);
};
