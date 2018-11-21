#pragma once

#include <Thresholds.hpp>
#include <sdbusplus/asio/object_server.hpp>

constexpr size_t sensorFailedPollTimeMs = 5000;
struct Sensor
{
    Sensor(const std::string &name, const std::string &path,
           std::vector<thresholds::Threshold> &&thresholdData,
           const std::string &configurationPath, const std::string &objectType,
           const double max, const double min) :
        name(name),
        path(path), thresholds(std::move(thresholdData)),
        configurationPath(configurationPath), maxValue(max), minValue(min)
    {
    }
    virtual ~Sensor() = default;
    virtual void checkThresholds(void) = 0;
    std::string name;
    std::string path;
    std::string configurationPath;
    std::string objectType;
    double maxValue;
    double minValue;
    std::vector<thresholds::Threshold> thresholds;
    std::shared_ptr<sdbusplus::asio::dbus_interface> sensorInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> thresholdInterfaceWarning;
    std::shared_ptr<sdbusplus::asio::dbus_interface> thresholdInterfaceCritical;
    double value = std::numeric_limits<double>::quiet_NaN();
    double overriddenValue = std::numeric_limits<double>::quiet_NaN();
    bool internalSet = false;

    int setSensorValue(const double &newValue, double &oldValue)
    {
        if (internalSet)
        {
            internalSet = false;
            oldValue = newValue;
            return 1;
        }
        overriddenValue = newValue;
        return 1;
    }

    void
        setInitialProperties(std::shared_ptr<sdbusplus::asio::connection> &conn)
    {
        sensorInterface->register_property("MaxValue", maxValue);
        sensorInterface->register_property("MinValue", minValue);
        sensorInterface->register_property(
            "Value", value, [&](const double &newValue, double &oldValue) {
                return setSensorValue(newValue, oldValue);
            });

        for (auto &threshold : thresholds)
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
            iface->register_property(
                level, threshold.value,
                [&](const double &request, double &oldValue) {
                    oldValue = request; // todo, just let the config do this?
                    threshold.value = request;
                    thresholds::persistThreshold(configurationPath, objectType,
                                                 threshold, conn);
                    return 1;
                });
            iface->register_property(alarm, false);
        }
        if (!sensorInterface->initialize())
        {
            std::cerr << "error initializing value interface\n";
        }
        if (thresholdInterfaceWarning &&
            !thresholdInterfaceWarning->initialize())
        {
            std::cerr << "error initializing warning threshold interface\n";
        }

        if (thresholdInterfaceCritical &&
            !thresholdInterfaceCritical->initialize())
        {
            std::cerr << "error initializing critical threshold interface\n";
        }
    }

    void updateValue(const double &newValue)
    {
        // Indicate that it is internal set call
        internalSet = true;
        sensorInterface->set_property("Value", newValue);
        internalSet = false;
        value = newValue;
        checkThresholds();
    }
};