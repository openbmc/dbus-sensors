#include "Thresholds.hpp"

#include "Utils.hpp"
#include "VariantVisitors.hpp"
#include "sensor.hpp"

#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/commit.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/exception.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/event.hpp>
#include <xyz/openbmc_project/Sensor/Value/common.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

namespace thresholds
{
Level findThresholdLevel(uint8_t sev)
{
    for (const ThresholdDefinition& prop : thresProp)
    {
        if (prop.sevOrder == sev)
        {
            return prop.level;
        }
    }
    return Level::ERROR;
}

Direction findThresholdDirection(const std::string& direct)
{
    if (direct == "greater than")
    {
        return Direction::HIGH;
    }
    if (direct == "less than")
    {
        return Direction::LOW;
    }
    return Direction::ERROR;
}

bool parseThresholdsFromConfig(
    const SensorData& sensorData,
    std::vector<thresholds::Threshold>& thresholdVector,
    const std::string* matchLabel, const int* sensorIndex)
{
    for (const auto& [intf, cfg] : sensorData)
    {
        if (intf.find("Thresholds") == std::string::npos)
        {
            continue;
        }
        if (matchLabel != nullptr)
        {
            auto labelFind = cfg.find("Label");
            if (labelFind == cfg.end())
            {
                continue;
            }
            if (std::visit(VariantToStringVisitor(), labelFind->second) !=
                *matchLabel)
            {
                continue;
            }
        }

        if (sensorIndex != nullptr)
        {
            auto indexFind = cfg.find("Index");

            // If we're checking for index 1, a missing Index is OK.
            if ((indexFind == cfg.end()) && (*sensorIndex != 1))
            {
                continue;
            }

            if ((indexFind != cfg.end()) &&
                (std::visit(VariantToIntVisitor(), indexFind->second) !=
                 *sensorIndex))
            {
                continue;
            }
        }

        double hysteresis = std::numeric_limits<double>::quiet_NaN();
        auto hysteresisFind = cfg.find("Hysteresis");
        if (hysteresisFind != cfg.end())
        {
            hysteresis =
                std::visit(VariantToDoubleVisitor(), hysteresisFind->second);
        }

        auto directionFind = cfg.find("Direction");
        auto severityFind = cfg.find("Severity");
        auto valueFind = cfg.find("Value");
        if (valueFind == cfg.end() || severityFind == cfg.end() ||
            directionFind == cfg.end())
        {
            lg2::error(
                "Malformed threshold on configuration interface: '{INTERFACE}'",
                "INTERFACE", intf);
            return false;
        }
        unsigned int severity =
            std::visit(VariantToUnsignedIntVisitor(), severityFind->second);

        std::string directions =
            std::visit(VariantToStringVisitor(), directionFind->second);

        Level level = findThresholdLevel(severity);
        Direction direction = findThresholdDirection(directions);

        if ((level == Level::ERROR) || (direction == Direction::ERROR))
        {
            continue;
        }
        double val = std::visit(VariantToDoubleVisitor(), valueFind->second);

        thresholdVector.emplace_back(level, direction, val, hysteresis);
    }
    return true;
}

void persistThreshold(const std::string& path, const std::string& baseInterface,
                      const thresholds::Threshold& threshold,
                      std::shared_ptr<sdbusplus::asio::connection>& conn,
                      size_t thresholdCount, const std::string& labelMatch)
{
    for (size_t ii = 0; ii < thresholdCount; ii++)
    {
        std::string thresholdInterface =
            baseInterface + ".Thresholds" + std::to_string(ii);
        conn->async_method_call(
            [&, path, threshold, thresholdInterface,
             labelMatch](const boost::system::error_code& ec,
                         const SensorBaseConfigMap& result) {
                if (ec)
                {
                    return; // threshold not supported
                }

                if (!labelMatch.empty())
                {
                    auto labelFind = result.find("Label");
                    if (labelFind == result.end())
                    {
                        lg2::error("No label in threshold configuration");
                        return;
                    }
                    std::string label =
                        std::visit(VariantToStringVisitor(), labelFind->second);
                    if (label != labelMatch)
                    {
                        return;
                    }
                }

                auto directionFind = result.find("Direction");
                auto severityFind = result.find("Severity");
                auto valueFind = result.find("Value");
                if (valueFind == result.end() || severityFind == result.end() ||
                    directionFind == result.end())
                {
                    lg2::error("Malformed threshold in configuration");
                    return;
                }
                unsigned int severity = std::visit(
                    VariantToUnsignedIntVisitor(), severityFind->second);

                std::string dir =
                    std::visit(VariantToStringVisitor(), directionFind->second);
                if ((findThresholdLevel(severity) != threshold.level) ||
                    (findThresholdDirection(dir) != threshold.direction))
                {
                    return; // not the droid we're looking for
                }

                std::variant<double> value(threshold.value);
                conn->async_method_call(
                    [](const boost::system::error_code& ec) {
                        if (ec)
                        {
                            lg2::error(
                                "Error setting threshold: '{ERROR_MESSAGE}'",
                                "ERROR_MESSAGE", ec.message());
                        }
                    },
                    entityManagerName, path, "org.freedesktop.DBus.Properties",
                    "Set", thresholdInterface, "Value", value);
            },
            entityManagerName, path, "org.freedesktop.DBus.Properties",
            "GetAll", thresholdInterface);
    }
}

void updateThresholds(Sensor* sensor)
{
    for (const auto& threshold : sensor->thresholds)
    {
        std::shared_ptr<sdbusplus::asio::dbus_interface> interface =
            sensor->getThresholdInterface(threshold.level);

        if (!interface)
        {
            continue;
        }

        std::string property =
            Sensor::propertyLevel(threshold.level, threshold.direction);
        if (property.empty())
        {
            continue;
        }
        interface->set_property(property, threshold.value);
    }
}

// Debugging counters
static int cHiTrue = 0;
static int cHiFalse = 0;
static int cHiMidstate = 0;
static int cLoTrue = 0;
static int cLoFalse = 0;
static int cLoMidstate = 0;
static int cDebugThrottle = 0;
static constexpr int assertLogCount = 10;
using Unit = sdbusplus::common::xyz::openbmc_project::sensor::Value::Unit;

struct ChangeParam
{
    ChangeParam(Threshold whichThreshold, bool status, double value) :
        threshold(std::move(whichThreshold)), asserted(status),
        assertValue(value)
    {}

    Threshold threshold;
    bool asserted;
    double assertValue;
};

static std::vector<ChangeParam> checkThresholds(Sensor* sensor, double value)
{
    std::vector<ChangeParam> thresholdChanges;
    if (sensor->thresholds.empty())
    {
        return thresholdChanges;
    }

    for (auto& threshold : sensor->thresholds)
    {
        // Use "Schmitt trigger" logic to avoid threshold trigger spam,
        // if value is noisy while hovering very close to a threshold.
        // When a threshold is crossed, indicate true immediately,
        // but require more distance to be crossed the other direction,
        // before resetting the indicator back to false.
        if (threshold.direction == thresholds::Direction::HIGH)
        {
            if (value >= threshold.value)
            {
                thresholdChanges.emplace_back(threshold, true, value);
                if (++cHiTrue < assertLogCount)
                {
                    lg2::info(
                        "Sensor name: {NAME}, high threshold: {THRESHOLD}, "
                        "assert value: {VALUE}, raw data: {RAW_DATA}",
                        "NAME", sensor->name, "THRESHOLD", threshold.value,
                        "VALUE", value, "RAW_DATA", sensor->rawValue);
                }
            }
            else if (value < (threshold.value - threshold.hysteresis))
            {
                thresholdChanges.emplace_back(threshold, false, value);
                ++cHiFalse;
            }
            else
            {
                ++cHiMidstate;
            }
        }
        else if (threshold.direction == thresholds::Direction::LOW)
        {
            if (value <= threshold.value)
            {
                thresholdChanges.emplace_back(threshold, true, value);
                if (++cLoTrue < assertLogCount)
                {
                    lg2::info(
                        "Sensor name: {NAME}, low threshold: {THRESHOLD}, "
                        "assert value: {VALUE}, raw data: {RAW_DATA}",
                        "NAME", sensor->name, "THRESHOLD", threshold.value,
                        "VALUE", value, "RAW_DATA", sensor->rawValue);
                }
            }
            else if (value > (threshold.value + threshold.hysteresis))
            {
                thresholdChanges.emplace_back(threshold, false, value);
                ++cLoFalse;
            }
            else
            {
                ++cLoMidstate;
            }
        }
        else
        {
            lg2::error("Error determining threshold direction");
        }
    }

    // Throttle debug output, so that it does not continuously spam
    ++cDebugThrottle;
    if (cDebugThrottle >= 1000)
    {
        cDebugThrottle = 0;
        lg2::debug("checkThresholds: High T= {HIGH_TRUE}, F= {HIGH_FALSE},"
                   " M= {HIGH_MIDSTATE}, Low T= {LOW_TRUE}, F= {LOW_FALSE},"
                   " M= {LOW_MIDSTATE}",
                   "HIGH_TRUE", cHiTrue, "HIGH_FALSE", cHiFalse,
                   "HIGH_MIDSTATE", cHiMidstate, "LOW_TRUE", cLoTrue,
                   "LOW_FALSE", cLoFalse, "LOW_MIDSTATE", cLoMidstate);
    }

    return thresholdChanges;
}

void ThresholdTimer::startTimer(const std::weak_ptr<Sensor>& weakSensor,
                                const Threshold& threshold, bool assert,
                                double assertValue)
{
    struct TimerUsed timerUsed = {};
    constexpr const size_t waitTime = 5;
    TimerPair* pair = nullptr;

    for (TimerPair& timer : timers)
    {
        if (!timer.first.used)
        {
            pair = &timer;
            break;
        }
    }
    if (pair == nullptr)
    {
        pair = &timers.emplace_back(timerUsed, boost::asio::steady_timer(io));
    }

    pair->first.used = true;
    pair->first.level = threshold.level;
    pair->first.direction = threshold.direction;
    pair->first.assert = assert;
    pair->second.expires_after(std::chrono::seconds(waitTime));
    pair->second.async_wait([weakSensor, pair, threshold, assert,
                             assertValue](boost::system::error_code ec) {
        auto sensorPtr = weakSensor.lock();
        if (!sensorPtr)
        {
            return; // owner sensor has been destructed
        }
        // pair is valid as long as sensor is valid
        pair->first.used = false;

        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        if (ec)
        {
            lg2::error("timer error: '{ERROR_MESSAGE}'", "ERROR_MESSAGE",
                       ec.message());
            return;
        }
        if (sensorPtr->readingStateGood())
        {
            assertThresholds(sensorPtr.get(), assertValue, threshold.level,
                             threshold.direction, assert);
        }
    });
}

bool checkThresholds(Sensor* sensor)
{
    bool status = true;
    std::vector<ChangeParam> changes = checkThresholds(sensor, sensor->value);
    for (const auto& change : changes)
    {
        assertThresholds(sensor, change.assertValue, change.threshold.level,
                         change.threshold.direction, change.asserted);
        if (change.threshold.level == thresholds::Level::CRITICAL &&
            change.asserted)
        {
            status = false;
        }
    }

    return status;
}

void checkThresholdsPowerDelay(const std::weak_ptr<Sensor>& weakSensor,
                               ThresholdTimer& thresholdTimer)
{
    auto sensorPtr = weakSensor.lock();
    if (!sensorPtr)
    {
        return; // sensor is destructed, should never be here
    }

    Sensor* sensor = sensorPtr.get();
    std::vector<ChangeParam> changes = checkThresholds(sensor, sensor->value);
    for (const auto& change : changes)
    {
        // When CPU is powered off, some volatges are expected to
        // go below low thresholds. Filter these events with thresholdTimer.
        // 1. always delay the assertion of low events to see if they are
        //   caused by power off event.
        // 2. conditional delay the de-assertion of low events if there is
        //   an existing timer for assertion.
        // 3. no delays for de-assert of low events if there is an existing
        //   de-assert for low event. This means 2nd de-assert would happen
        //   first and when timer expires for the previous one, no additional
        //   signal will be logged.
        // 4. no delays for all high events.
        if (change.threshold.direction == thresholds::Direction::LOW)
        {
            if (change.asserted || thresholdTimer.hasActiveTimer(
                                       change.threshold, !change.asserted))
            {
                thresholdTimer.startTimer(weakSensor, change.threshold,
                                          change.asserted, change.assertValue);
                continue;
            }
        }
        assertThresholds(sensor, change.assertValue, change.threshold.level,
                         change.threshold.direction, change.asserted);
    }
}

void logDeassertThresholds(Sensor* sensor, double value,
                           thresholds::Level level,
                           thresholds::Direction direction)
{
    namespace events =
        sdbusplus::event::xyz::openbmc_project::sensor::Threshold;

    auto thresholdIt =
        std::find_if(sensor->thresholds.begin(), sensor->thresholds.end(),
                     [level, direction](auto& th) {
                         return th.level == level && th.direction == direction;
                     });
    if (thresholdIt != sensor->thresholds.end())
    {
        auto& threshold = *thresholdIt;
        if (threshold.assertedLog)
        {
            try
            {
                /* empty log entries are returned by commit() if the
                requested log is being filtered out */
                if (!threshold.assertedLog->str.empty())
                {
                    lg2::resolve(*threshold.assertedLog);
                }
            }
            catch (std::exception& ec)
            {
                lg2::error("Unable to resolve {LOG} : {ERROR}", "LOG",
                           std::string(*threshold.assertedLog), "ERROR",
                           ec.what());
            }
            threshold.assertedLog.reset();
        }
    }
    auto it = std::find_if(
        sensor->thresholds.begin(), sensor->thresholds.end(),
        [](auto& th) -> bool { return th.assertedLog.has_value(); });
    // Return if there are outstanding asserts.
    if (it != sensor->thresholds.end())
    {
        return;
    }
    auto objPath = sensor->sensorInterface->get_object_path();
    lg2::commit(events::SensorReadingNormalRange(
        "SENSOR_NAME", objPath, "READING_VALUE", value, "UNITS",
        sensor->units));
}

template <typename errorObj>
auto logAssertThresholdHelper(const std::string& objPath, double assertValue,
                              Unit unit, double thresholdValue)
    -> sdbusplus::message::object_path
{
    return lg2::commit(
        errorObj("SENSOR_NAME", objPath, "READING_VALUE", assertValue, "UNITS",
                 unit, "THRESHOLD_VALUE", thresholdValue));
}

void logAssertThresholds(Sensor* sensor, double assertValue,
                         thresholds::Level level,
                         thresholds::Direction direction)
{
    namespace errors =
        sdbusplus::error::xyz::openbmc_project::sensor::Threshold;
    static const std::map<std::tuple<thresholds::Level, thresholds::Direction>,
                          std::function<sdbusplus::message::object_path(
                              const std::string&, double, Unit, double)>>
        thresholdLogMap = {
            {{Level::WARNING, Direction::HIGH},
             &logAssertThresholdHelper<
                 errors::ReadingAboveUpperWarningThreshold>},
            {{Level::WARNING, Direction::LOW},
             &logAssertThresholdHelper<
                 errors::ReadingBelowLowerWarningThreshold>},
            {{Level::PERFORMANCELOSS, Direction::HIGH},
             &logAssertThresholdHelper<
                 errors::ReadingAboveUpperPerformanceLossThreshold>},
            {{Level::PERFORMANCELOSS, Direction::LOW},
             &logAssertThresholdHelper<
                 errors::ReadingBelowLowerPerformanceLossThreshold>},
            {{Level::CRITICAL, Direction::HIGH},
             &logAssertThresholdHelper<
                 errors::ReadingAboveUpperCriticalThreshold>},
            {{Level::CRITICAL, Direction::LOW},
             &logAssertThresholdHelper<
                 errors::ReadingBelowLowerCriticalThreshold>},
            {{Level::SOFTSHUTDOWN, Direction::HIGH},
             &logAssertThresholdHelper<
                 errors::ReadingAboveUpperSoftShutdownThreshold>},
            {{Level::SOFTSHUTDOWN, Direction::LOW},
             &logAssertThresholdHelper<
                 errors::ReadingBelowLowerSoftShutdownThreshold>},
            {{Level::HARDSHUTDOWN, Direction::HIGH},
             &logAssertThresholdHelper<
                 errors::ReadingAboveUpperHardShutdownThreshold>},
            {{Level::HARDSHUTDOWN, Direction::LOW},
             &logAssertThresholdHelper<
                 errors::ReadingBelowLowerHardShutdownThreshold>}};
    auto objPath = sensor->sensorInterface->get_object_path();
    auto thresholdIter =
        std::find_if(sensor->thresholds.begin(), sensor->thresholds.end(),
                     [level, direction](auto& th) {
                         return th.level == level && th.direction == direction;
                     });
    if ((thresholdIter == sensor->thresholds.end()))
    {
        lg2::error("Unable to find threshold {SENSOR}", "SENSOR", objPath);
        return;
    }
    auto& threshold = *thresholdIter;
    if (threshold.assertedLog)
    {
        // Technically we should never get here. But handle anyway.
        lg2::error("Ignoring new log with unresolved outstanding entry: {LOG}",
                   "LOG", std::string(*(threshold.assertedLog)));
        return;
    }
    try
    {
        threshold.assertedLog = thresholdLogMap.at({level, direction})(
            objPath, assertValue, sensor->units, threshold.value);
    }
    catch (std::out_of_range& e)
    {
        lg2::error("Unknown threshold level/direction assertion for {SENSOR}",
                   "SENSOR", objPath);
    }
    catch (std::exception& e)
    {
        lg2::error("Could not create threshold log entry for {SENSOR}",
                   "SENSOR", objPath);
    }
}

void assertThresholds(Sensor* sensor, double assertValue,
                      thresholds::Level level, thresholds::Direction direction,
                      bool assert)
{
    std::shared_ptr<sdbusplus::asio::dbus_interface> interface =
        sensor->getThresholdInterface(level);

    if (!interface)
    {
        lg2::info("trying to set uninitialized interface");
        return;
    }

    std::string property = Sensor::propertyAlarm(level, direction);
    if (property.empty())
    {
        lg2::info("Alarm property is empty");
        return;
    }
    if (interface->set_property<bool, true>(property, assert))
    {
        try
        {
            // msg.get_path() is interface->get_object_path()
            sdbusplus::message_t msg =
                interface->new_signal("ThresholdAsserted");

            msg.append(sensor->name, interface->get_interface_name(), property,
                       assert, assertValue);
            msg.signal_send();
        }
        catch (const sdbusplus::exception_t& e)
        {
            lg2::error(
                "Failed to send thresholdAsserted signal with assertValue");
        }
        if (assert)
        {
            logAssertThresholds(sensor, assertValue, level, direction);
        }
        else
        {
            logDeassertThresholds(sensor, assertValue, level, direction);
        }
    }
}

bool parseThresholdsFromAttr(
    std::vector<thresholds::Threshold>& thresholdVector,
    const std::string& inputPath, const double& scaleFactor,
    const double& offset, const double& hysteresis)
{
    const boost::container::flat_map<
        std::string, std::vector<std::tuple<const char*, thresholds::Level,
                                            thresholds::Direction, double>>>
        map = {
            {"average",
             {
                 std::make_tuple("average_min", Level::WARNING, Direction::LOW,
                                 0.0),
                 std::make_tuple("average_max", Level::WARNING, Direction::HIGH,
                                 0.0),
             }},
            {"input",
             {
                 std::make_tuple("min", Level::WARNING, Direction::LOW, 0.0),
                 std::make_tuple("max", Level::WARNING, Direction::HIGH, 0.0),
                 std::make_tuple("lcrit", Level::CRITICAL, Direction::LOW, 0.0),
                 std::make_tuple("crit", Level::CRITICAL, Direction::HIGH,
                                 offset),
             }},
        };

    if (auto fileParts = splitFileName(inputPath))
    {
        auto& [type, nr, item] = *fileParts;
        if (map.contains(item))
        {
            for (const auto& t : map.at(item))
            {
                const auto& [suffix, level, direction, offset] = t;
                auto attrPath =
                    boost::replace_all_copy(inputPath, item, suffix);
                if (auto val = readFile(attrPath, scaleFactor))
                {
                    *val += offset;
                    lg2::debug("Threshold: '{PATH}': '{VALUE}'", "PATH",
                               attrPath, "VALUE", *val);
                    thresholdVector.emplace_back(level, direction, *val,
                                                 hysteresis);
                }
            }
        }
    }
    return true;
}

std::string getInterface(const Level thresholdLevel)
{
    for (const ThresholdDefinition& thresh : thresProp)
    {
        if (thresh.level == thresholdLevel)
        {
            return std::string("xyz.openbmc_project.Sensor.Threshold.") +
                   thresh.levelName;
        }
    }
    return "";
}
} // namespace thresholds
