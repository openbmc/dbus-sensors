#pragma once

#include "Thresholds.hpp"

#include <sdbusplus/asio/object_server.hpp>

#include <limits>
#include <memory>
#include <string>
#include <vector>

constexpr size_t sensorFailedPollTimeMs = 5000;

constexpr const char* sensorValueInterface = "xyz.openbmc_project.Sensor.Value";
struct Sensor
{
    Sensor(const std::string& name,
           std::vector<thresholds::Threshold>&& thresholdData,
           const std::string& configurationPath, const std::string& objectType,
           const double max, const double min) :
        name(std::regex_replace(name, std::regex("[^a-zA-Z0-9_/]+"), "_")),
        configurationPath(configurationPath), objectType(objectType),
        maxValue(max), minValue(min), thresholds(std::move(thresholdData)),
        hysteresisTrigger((max - min) * 0.01),
        hysteresisPublish((max - min) * 0.0001)
    {}
    virtual ~Sensor() = default;
    virtual void checkThresholds(void) = 0;
    std::string name;
    std::string configurationPath;
    std::string objectType;
    double maxValue;
    double minValue;
    std::vector<thresholds::Threshold> thresholds;
    std::shared_ptr<sdbusplus::asio::dbus_interface> sensorInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> thresholdInterfaceWarning;
    std::shared_ptr<sdbusplus::asio::dbus_interface> thresholdInterfaceCritical;
    std::shared_ptr<sdbusplus::asio::dbus_interface> association;
    double value = std::numeric_limits<double>::quiet_NaN();
    bool overriddenState = false;
    bool internalSet = false;
    double hysteresisTrigger;
    double hysteresisPublish;

    int setSensorValue(const double& newValue, double& oldValue)
    {
        if (!internalSet)
        {
            oldValue = newValue;
            overriddenState = true;
            // check thresholds for external set
            value = newValue;
            checkThresholds();
        }
        else if (!overriddenState)
        {
            oldValue = newValue;
        }
        return 1;
    }

    void
        setInitialProperties(std::shared_ptr<sdbusplus::asio::connection>& conn,
                             const std::string label = std::string(),
                             size_t thresholdSize = 0)
    {
        createAssociation(association, configurationPath);

        sensorInterface->register_property("MaxValue", maxValue);
        sensorInterface->register_property("MinValue", minValue);
        sensorInterface->register_property(
            "Value", value, [&](const double& newValue, double& oldValue) {
                return setSensorValue(newValue, oldValue);
            });
        for (auto& threshold : thresholds)
        {
            std::shared_ptr<sdbusplus::asio::dbus_interface> iface;
            std::string level;
            std::string alarm;
            if (threshold.level == thresholds::Level::CRITICAL)
            {
                iface = thresholdInterfaceCritical;
                if (threshold.direction == thresholds::Direction::HIGH)
                {
                    level = "CriticalHigh";
                    alarm = "CriticalAlarmHigh";
                }
                else
                {
                    level = "CriticalLow";
                    alarm = "CriticalAlarmLow";
                }
            }
            else if (threshold.level == thresholds::Level::WARNING)
            {
                iface = thresholdInterfaceWarning;
                if (threshold.direction == thresholds::Direction::HIGH)
                {
                    level = "WarningHigh";
                    alarm = "WarningAlarmHigh";
                }
                else
                {
                    level = "WarningLow";
                    alarm = "WarningAlarmLow";
                }
            }
            else
            {
                std::cerr << "Unknown threshold level" << threshold.level
                          << "\n";
                continue;
            }
            if (!iface)
            {
                std::cout << "trying to set uninitialized interface\n";
                continue;
            }

            size_t thresSize =
                label.empty() ? thresholds.size() : thresholdSize;
            iface->register_property(
                level, threshold.value,
                [&, label, thresSize](const double& request, double& oldValue) {
                    oldValue = request; // todo, just let the config do this?
                    threshold.value = request;
                    thresholds::persistThreshold(configurationPath, objectType,
                                                 threshold, conn, thresSize,
                                                 label);
                    // Invalidate previously remembered value,
                    // so new thresholds will be checked during next update,
                    // even if sensor reading remains unchanged.
                    value = std::numeric_limits<double>::quiet_NaN();

                    // Although tempting, don't call checkThresholds() from here
                    // directly. Let the regular sensor monitor call the same
                    // using updateValue(), which can check conditions like
                    // poweron, etc., before raising any event.
                    return 1;
                });
            iface->register_property(alarm, false);
        }
        if (!sensorInterface->initialize())
        {
            std::cerr << "error initializing value interface\n";
        }
        if (thresholdInterfaceWarning &&
            !thresholdInterfaceWarning->initialize(true))
        {
            std::cerr << "error initializing warning threshold interface\n";
        }

        if (thresholdInterfaceCritical &&
            !thresholdInterfaceCritical->initialize(true))
        {
            std::cerr << "error initializing critical threshold interface\n";
        }
    }

    void updateValue(const double& newValue)
    {
        // Ignore if overriding is enabled
        if (overriddenState)
        {
            return;
        }

        bool isChanged = false;

        // Avoid floating-point equality comparison,
        // by instead comparing against a very small hysteresis range.
        if (std::isnan(value) || std::isnan(newValue))
        {
            // If one or the other is NAN,
            // either we are intentionally invalidating a sensor reading,
            // or initializing for the very first time,
            // either way we should always publish this.
            isChanged = true;
        }
        else
        {
            // This essentially does "if (value != newValue)",
            // but safely against floating-point background noise.
            double diff = std::abs(value - newValue);
            if (diff > hysteresisPublish)
            {
                isChanged = true;
            }
        }

        // Ignore if the change is so small as to be deemed unchanged
        if (!isChanged)
        {
            return;
        }

        // The value will be changed, keep track of it for next time
        value = newValue;

        // Indicate that it is internal set call
        internalSet = true;
        if (!(sensorInterface->set_property("Value", newValue)))
        {
            std::cerr << "error setting property to " << newValue << "\n";
        }
        internalSet = false;

        // Always check thresholds after changing the value,
        // as the test against hysteresisTrigger now takes place in
        // the thresholds::checkThresholds() method,
        // which is called by checkThresholds() below,
        // in all current implementations of sensors that have thresholds.
        checkThresholds();
    }
};
