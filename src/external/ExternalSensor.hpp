#pragma once

#include "asio/Thresholds.hpp"
#include "asio/sensor.hpp"
#include "utils/Utils.hpp"

#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <functional>
#include <memory>
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
        const std::string& sensorName, const std::string& sensorUnits,
        std::vector<thresholds::Threshold>&& thresholdsIn,
        const std::string& sensorConfiguration, double maxReading,
        double minReading, double timeoutSecs, const PowerState& powerState);
    ~ExternalSensor() override;

    // Call this immediately after calling the constructor
    void initWriteHook(
        std::function<void(std::chrono::steady_clock::time_point now)>&&
            writeHookIn);

    // Returns true if sensor has external Value that is subject to timeout
    bool isAliveAndPerishable() const;

    // Returns true if AliveAndPerishable and timeout has not yet happened
    bool isAliveAndFresh(
        const std::chrono::steady_clock::time_point& now) const;

    // Marks the time when Value successfully received from external source
    void writeBegin(const std::chrono::steady_clock::time_point& now);

    // Marks sensor as timed out, replacing Value with floating-point "NaN"
    void writeInvalidate();

    // Returns amount of time elapsed since last writeBegin() happened
    std::chrono::steady_clock::duration ageElapsed(
        const std::chrono::steady_clock::time_point& now) const;

    // Returns amount of time remaining until sensor timeout will happen
    std::chrono::steady_clock::duration ageRemaining(
        const std::chrono::steady_clock::time_point& now) const;

  private:
    sdbusplus::asio::object_server& objServer;

    std::chrono::steady_clock::time_point writeLast;
    std::chrono::steady_clock::duration writeTimeout;
    bool writeAlive{false};
    bool writePerishable;
    std::function<void(const std::chrono::steady_clock::time_point& now)>
        writeHook;

    void checkThresholds() override;
    void externalSetTrigger();
};
