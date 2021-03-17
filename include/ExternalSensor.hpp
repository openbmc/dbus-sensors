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
        std::vector<thresholds::Threshold>&& thresholdsIn,
        const std::string& sensorConfiguration, const double& maxReading,
        const double& minReading, const double& timeoutSecs,
        const PowerState& powerState,
        std::function<void(const std::chrono::steady_clock::time_point& now)>&&
            writeHookIn);
    virtual ~ExternalSensor();

    // Returns true if sensor Value is eligible to become "NaN" upon timeout
    bool isPerishable(void) const;

    // Returns true if sensor is Perishable and timeout has not yet happened
    bool isViable(const std::chrono::steady_clock::time_point& now) const;

    // Marks the time when Value successfully received from external source
    void writeBegin(const std::chrono::steady_clock::time_point& now);

    // Marks sensor as timed out, replacing Value with floating-point "NaN"
    void writeInvalidate(void);

    // Returns amount of time elapsed since last writeBegin() happened
    std::chrono::steady_clock::duration
        ageElapsed(const std::chrono::steady_clock::time_point& now) const;

    // Returns amount of time remaining until sensor timeout will happen
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
