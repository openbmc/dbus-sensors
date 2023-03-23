#pragma once

#include "dbus-sensor_config.h"

#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"

#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/exception.hpp>

#include <limits>
#include <memory>
#include <string>
#include <vector>

constexpr size_t sensorFailedPollTimeMs = 5000;

// Enable useful logging with sensor instrumentation
// This is intentionally not DEBUG, avoid clash with usage in .cpp files
constexpr bool enableInstrumentation = false;

constexpr const char* sensorValueInterface = "xyz.openbmc_project.Sensor.Value";
constexpr const char* valueMutabilityInterfaceName =
    "xyz.openbmc_project.Sensor.ValueMutability";
constexpr const char* availableInterfaceName =
    "xyz.openbmc_project.State.Decorator.Availability";
constexpr const char* operationalInterfaceName =
    "xyz.openbmc_project.State.Decorator.OperationalStatus";
constexpr const size_t errorThreshold = 5;

struct SensorInstrumentation
{
    // These are for instrumentation for debugging
    int numCollectsGood = 0;
    int numCollectsMiss = 0;
    int numStreakGreats = 0;
    int numStreakMisses = 0;
    double minCollected = 0.0;
    double maxCollected = 0.0;
};

struct SetSensorError : sdbusplus::exception_t
{
    const char* name() const noexcept override
    {
        return "xyz.openbmc_project.Common.Errors.NotAllowed";
    }
    const char* description() const noexcept override
    {
        return "Not allowed to set property value.";
    }
    int get_errno() const noexcept override
    {
        return EACCES;
    }
};

struct Sensor
{
    Sensor(const std::string& name,
           std::vector<thresholds::Threshold>&& thresholdData,
           const std::string& configurationPath, const std::string& objectType,
           bool isSettable, bool isMutable, const double max, const double min,
           std::shared_ptr<sdbusplus::asio::connection>& conn,
           PowerState readState = PowerState::always) :
        name(sensor_paths::escapePathForDbus(name)),
        configurationPath(configurationPath),
        objectType(configInterfaceName(objectType)),
        isSensorSettable(isSettable), isValueMutable(isMutable), maxValue(max),
        minValue(min), thresholds(std::move(thresholdData)),
        hysteresisTrigger((max - min) * 0.01),
        hysteresisPublish((max - min) * 0.0001), dbusConnection(conn),
        readState(readState),
        instrumentation(enableInstrumentation
                            ? std::make_unique<SensorInstrumentation>()
                            : nullptr)
    {}
    virtual ~Sensor() = default;
    virtual void checkThresholds(void) = 0;
    std::string name;
    std::string configurationPath;
    std::string objectType;
    bool isSensorSettable;

    /* A flag indicates if properties of xyz.openbmc_project.Sensor.Value
     * interface are mutable. If mutable, then
     * xyz.openbmc_project.Sensor.ValueMutability interface will be
     * instantiated.
     */
    bool isValueMutable;
    double maxValue;
    double minValue;
    std::vector<thresholds::Threshold> thresholds;
    std::shared_ptr<sdbusplus::asio::dbus_interface> sensorInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> association;
    std::shared_ptr<sdbusplus::asio::dbus_interface> availableInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> operationalInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> valueMutabilityInterface;
    double value = std::numeric_limits<double>::quiet_NaN();
    double rawValue = std::numeric_limits<double>::quiet_NaN();
    bool overriddenState = false;
    bool internalSet = false;
    double hysteresisTrigger;
    double hysteresisPublish;
    std::shared_ptr<sdbusplus::asio::connection> dbusConnection;
    PowerState readState;
    size_t errCount{0};
    std::unique_ptr<SensorInstrumentation> instrumentation;

    // This member variable provides a hook that can be used to receive
    // notification whenever this Sensor's value is externally set via D-Bus.
    // If interested, assign your own lambda to this variable, during
    // construction of your Sensor subclass. See ExternalSensor for example.
    std::function<void()> externalSetHook;

    using Level = thresholds::Level;
    using Direction = thresholds::Direction;

    std::array<std::shared_ptr<sdbusplus::asio::dbus_interface>,
               thresholds::thresProp.size()>
        thresholdInterfaces;

    std::shared_ptr<sdbusplus::asio::dbus_interface>
        getThresholdInterface(Level lev)
    {
        size_t index = static_cast<size_t>(lev);
        if (index >= thresholdInterfaces.size())
        {
            std::cout << "Unknown threshold level \n";
            return nullptr;
        }
        std::shared_ptr<sdbusplus::asio::dbus_interface> interface =
            thresholdInterfaces[index];
        return interface;
    }

    void updateInstrumentation(double readValue) const
    {
        // Do nothing if this feature is not enabled
        if constexpr (!enableInstrumentation)
        {
            return;
        }
        if (!instrumentation)
        {
            return;
        }

        // Save some typing
        auto& inst = *instrumentation;

        // Show constants if first reading (even if unsuccessful)
        if ((inst.numCollectsGood == 0) && (inst.numCollectsMiss == 0))
        {
            std::cerr << "Sensor " << name << ": Configuration min=" << minValue
                      << ", max=" << maxValue << ", type=" << objectType
                      << ", path=" << configurationPath << "\n";
        }

        // Sensors can use "nan" to indicate unavailable reading
        if (!std::isfinite(readValue))
        {
            // Only show this if beginning a new streak
            if (inst.numStreakMisses == 0)
            {
                std::cerr << "Sensor " << name
                          << ": Missing reading, Reading counts good="
                          << inst.numCollectsGood
                          << ", miss=" << inst.numCollectsMiss
                          << ", Prior good streak=" << inst.numStreakGreats
                          << "\n";
            }

            inst.numStreakGreats = 0;
            ++(inst.numCollectsMiss);
            ++(inst.numStreakMisses);

            return;
        }

        // Only show this if beginning a new streak and not the first time
        if ((inst.numStreakGreats == 0) && (inst.numCollectsGood != 0))
        {
            std::cerr << "Sensor " << name
                      << ": Recovered reading, Reading counts good="
                      << inst.numCollectsGood
                      << ", miss=" << inst.numCollectsMiss
                      << ", Prior miss streak=" << inst.numStreakMisses << "\n";
        }

        // Initialize min/max if the first successful reading
        if (inst.numCollectsGood == 0)
        {
            std::cerr << "Sensor " << name << ": First reading=" << readValue
                      << "\n";

            inst.minCollected = readValue;
            inst.maxCollected = readValue;
        }

        inst.numStreakMisses = 0;
        ++(inst.numCollectsGood);
        ++(inst.numStreakGreats);

        // Only provide subsequent output if new min/max established
        if (readValue < inst.minCollected)
        {
            std::cerr << "Sensor " << name << ": Lowest reading=" << readValue
                      << "\n";

            inst.minCollected = readValue;
        }

        if (readValue > inst.maxCollected)
        {
            std::cerr << "Sensor " << name << ": Highest reading=" << readValue
                      << "\n";

            inst.maxCollected = readValue;
        }
    }

    int setSensorValue(const double& newValue, double& oldValue)
    {
        if (!internalSet)
        {
            if (insecureSensorOverride == 0 && !isSensorSettable &&
                !getManufacturingMode())
            {
                throw SetSensorError();
            }

            oldValue = newValue;
            overriddenState = true;
            // check thresholds for external set
            value = newValue;
            checkThresholds();

            // Trigger the hook, as an external set has just happened
            if (externalSetHook)
            {
                externalSetHook();
            }
        }
        else if (!overriddenState)
        {
            oldValue = newValue;
        }
        return 1;
    }

    void setInitialProperties(const std::string& unit,
                              const std::string& label = std::string(),
                              size_t thresholdSize = 0)
    {
        if (readState == PowerState::on || readState == PowerState::biosPost ||
            readState == PowerState::chassisOn)
        {
            setupPowerMatch(dbusConnection);
        }

        createAssociation(association, configurationPath);

        sensorInterface->register_property("Unit", unit);
        sensorInterface->register_property("MaxValue", maxValue);
        sensorInterface->register_property("MinValue", minValue);
        sensorInterface->register_property(
            "Value", value, [this](const double& newValue, double& oldValue) {
                return setSensorValue(newValue, oldValue);
            });

        fillMissingThresholds();

        for (auto& threshold : thresholds)
        {
            if (std::isnan(threshold.hysteresis))
            {
                threshold.hysteresis = hysteresisTrigger;
            }

            std::shared_ptr<sdbusplus::asio::dbus_interface> iface =
                getThresholdInterface(threshold.level);

            if (!iface)
            {
                std::cout << "trying to set uninitialized interface\n";
                continue;
            }

            std::string level =
                propertyLevel(threshold.level, threshold.direction);
            std::string alarm =
                propertyAlarm(threshold.level, threshold.direction);

            if ((level.empty()) || (alarm.empty()))
            {
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
                                             threshold, dbusConnection,
                                             thresSize, label);
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

        for (auto& thresIface : thresholdInterfaces)
        {
            if (thresIface)
            {
                if (!thresIface->initialize(true))
                {
                    std::cerr << "Error initializing threshold interface \n";
                }
            }
        }

        if (isValueMutable)
        {
            valueMutabilityInterface =
                std::make_shared<sdbusplus::asio::dbus_interface>(
                    dbusConnection, sensorInterface->get_object_path(),
                    valueMutabilityInterfaceName);
            valueMutabilityInterface->register_property("Mutable", true);
            if (!valueMutabilityInterface->initialize())
            {
                std::cerr
                    << "error initializing sensor value mutability interface\n";
                valueMutabilityInterface = nullptr;
            }
        }

        if (!availableInterface)
        {
            availableInterface =
                std::make_shared<sdbusplus::asio::dbus_interface>(
                    dbusConnection, sensorInterface->get_object_path(),
                    availableInterfaceName);
            availableInterface->register_property(
                "Available", true, [this](const bool propIn, bool& old) {
                    if (propIn == old)
                    {
                        return 1;
                    }
                    old = propIn;
                    if (!propIn)
                    {
                        updateValue(std::numeric_limits<double>::quiet_NaN());
                    }
                    return 1;
                });
            availableInterface->initialize();
        }
        if (!operationalInterface)
        {
            operationalInterface =
                std::make_shared<sdbusplus::asio::dbus_interface>(
                    dbusConnection, sensorInterface->get_object_path(),
                    operationalInterfaceName);
            operationalInterface->register_property("Functional", true);
            operationalInterface->initialize();
        }
    }

    static std::string propertyLevel(const Level lev, const Direction dir)
    {
        for (const thresholds::ThresholdDefinition& prop :
             thresholds::thresProp)
        {
            if (prop.level == lev)
            {
                if (dir == Direction::HIGH)
                {
                    return std::string(prop.levelName) + "High";
                }
                if (dir == Direction::LOW)
                {
                    return std::string(prop.levelName) + "Low";
                }
            }
        }
        return "";
    }

    static std::string propertyAlarm(const Level lev, const Direction dir)
    {
        for (const thresholds::ThresholdDefinition& prop :
             thresholds::thresProp)
        {
            if (prop.level == lev)
            {
                if (dir == Direction::HIGH)
                {
                    return std::string(prop.levelName) + "AlarmHigh";
                }
                if (dir == Direction::LOW)
                {
                    return std::string(prop.levelName) + "AlarmLow";
                }
            }
        }
        return "";
    }

    bool readingStateGood() const
    {
        return ::readingStateGood(readState);
    }

    void markFunctional(bool isFunctional)
    {
        if (operationalInterface)
        {
            operationalInterface->set_property("Functional", isFunctional);
        }
        if (isFunctional)
        {
            errCount = 0;
        }
        else
        {
            updateValue(std::numeric_limits<double>::quiet_NaN());
        }
    }

    void markAvailable(bool isAvailable)
    {
        if (availableInterface)
        {
            availableInterface->set_property("Available", isAvailable);
            errCount = 0;
        }
    }

    void incrementError()
    {
        if (!readingStateGood())
        {
            markAvailable(false);
            return;
        }

        if (errCount >= errorThreshold)
        {
            return;
        }

        errCount++;
        if (errCount == errorThreshold)
        {
            std::cerr << "Sensor " << name << " reading error!\n";
            markFunctional(false);
        }
    }

    bool inError() const
    {
        return errCount >= errorThreshold;
    }

    void updateValue(const double& newValue)
    {
        // Ignore if overriding is enabled
        if (overriddenState)
        {
            return;
        }

        if (!readingStateGood())
        {
            markAvailable(false);
            for (auto& threshold : thresholds)
            {
                assertThresholds(this, value, threshold.level,
                                 threshold.direction, false);
            }
            updateValueProperty(std::numeric_limits<double>::quiet_NaN());
            return;
        }

        updateValueProperty(newValue);
        updateInstrumentation(newValue);

        // Always check thresholds after changing the value,
        // as the test against hysteresisTrigger now takes place in
        // the thresholds::checkThresholds() method,
        // which is called by checkThresholds() below,
        // in all current implementations of sensors that have thresholds.
        checkThresholds();
        if (!std::isnan(newValue))
        {
            markFunctional(true);
            markAvailable(true);
        }
    }

    void updateProperty(
        std::shared_ptr<sdbusplus::asio::dbus_interface>& interface,
        double& oldValue, const double& newValue,
        const char* dbusPropertyName) const
    {
        if (requiresUpdate(oldValue, newValue))
        {
            oldValue = newValue;
            if (interface &&
                !(interface->set_property(dbusPropertyName, newValue)))
            {
                std::cerr << "error setting property " << dbusPropertyName
                          << " to " << newValue << "\n";
            }
        }
    }

    bool requiresUpdate(const double& lVal, const double& rVal) const
    {
        const auto lNan = std::isnan(lVal);
        const auto rNan = std::isnan(rVal);
        if (lNan || rNan)
        {
            return (lNan != rNan);
        }
        return std::abs(lVal - rVal) > hysteresisPublish;
    }

  private:
    // If one of the thresholds for a dbus interface is provided
    // we have to set the other one as dbus properties are never
    // optional.
    void fillMissingThresholds()
    {
        for (thresholds::Threshold& thisThreshold : thresholds)
        {
            bool foundOpposite = false;
            thresholds::Direction opposite = thresholds::Direction::HIGH;
            if (thisThreshold.direction == thresholds::Direction::HIGH)
            {
                opposite = thresholds::Direction::LOW;
            }
            for (thresholds::Threshold& otherThreshold : thresholds)
            {
                if (thisThreshold.level != otherThreshold.level)
                {
                    continue;
                }
                if (otherThreshold.direction != opposite)
                {
                    continue;
                }
                foundOpposite = true;
                break;
            }
            if (foundOpposite)
            {
                continue;
            }
            thresholds.emplace_back(thisThreshold.level, opposite,
                                    std::numeric_limits<double>::quiet_NaN());
        }
    }

    void updateValueProperty(const double& newValue)
    {
        // Indicate that it is internal set call, not an external overwrite
        internalSet = true;
        updateProperty(sensorInterface, value, newValue, "Value");
        internalSet = false;
    }
};
