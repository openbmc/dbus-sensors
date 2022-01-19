#include <Thresholds.hpp>
#include <VariantVisitors.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_map.hpp>
#include <sensor.hpp>

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

static constexpr bool debug = false;
namespace thresholds
{
Level findThresholdLevel(uint8_t sev, const std::string& direct)
{
    for (Sensor::ThresholdProperty prop : Sensor::thresProp)
    {
        if ((prop.sevOrder == sev) && (prop.dirOrder == direct))
        {
            return prop.level;
        }
    }
    return Level::ERROR;
}

Direction findThresholdDirection(uint8_t sev, const std::string& direct)
{
    for (Sensor::ThresholdProperty prop : Sensor::thresProp)
    {
        if ((prop.sevOrder == sev) && (prop.dirOrder == direct))
        {
            return prop.direction;
        }
    }
    return Direction::ERROR;
}

bool findOrder(Level lev, Direction dir)
{
    for (Sensor::ThresholdProperty prop : Sensor::thresProp)
    {
        if ((prop.level == lev) && (prop.direction == dir))
        {
            return true;
        }
    }
    return false;
}

bool parseThresholdsFromConfig(
    const SensorData& sensorData,
    std::vector<thresholds::Threshold>& thresholdVector,
    const std::string* matchLabel, const int* sensorIndex)
{
    for (const auto& item : sensorData)
    {
        if (item.first.find("Thresholds") == std::string::npos)
        {
            continue;
        }
        if (matchLabel != nullptr)
        {
            auto labelFind = item.second.find("Label");
            if (labelFind == item.second.end())
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
            auto indexFind = item.second.find("Index");

            // If we're checking for index 1, a missing Index is OK.
            if ((indexFind == item.second.end()) && (*sensorIndex != 1))
            {
                continue;
            }

            if ((indexFind != item.second.end()) &&
                (std::visit(VariantToIntVisitor(), indexFind->second) !=
                 *sensorIndex))
            {
                continue;
            }
        }

        double hysteresis = std::numeric_limits<double>::quiet_NaN();
        auto hysteresisFind = item.second.find("Hysteresis");
        if (hysteresisFind != item.second.end())
        {
            hysteresis =
                std::visit(VariantToDoubleVisitor(), hysteresisFind->second);
        }

        auto directionFind = item.second.find("Direction");
        auto severityFind = item.second.find("Severity");
        auto valueFind = item.second.find("Value");
        if (valueFind == item.second.end() ||
            severityFind == item.second.end() ||
            directionFind == item.second.end())
        {
            std::cerr << "Malformed threshold on configuration interface "
                      << item.first << "\n";
            return false;
        }
        unsigned int severity =
            std::visit(VariantToUnsignedIntVisitor(), severityFind->second);

        std::string directions =
            std::visit(VariantToStringVisitor(), directionFind->second);

        Level level = findThresholdLevel(severity, directions);
        Direction direction = findThresholdDirection(severity, directions);

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
            [&, path, threshold, thresholdInterface, labelMatch](
                const boost::system::error_code& ec,
                const boost::container::flat_map<std::string, BasicVariantType>&
                    result) {
                if (ec)
                {
                    return; // threshold not supported
                }

                if (!labelMatch.empty())
                {
                    auto labelFind = result.find("Label");
                    if (labelFind == result.end())
                    {
                        std::cerr << "No label in threshold configuration\n";
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
                    std::cerr << "Malformed threshold in configuration\n";
                    return;
                }
                unsigned int severity = std::visit(
                    VariantToUnsignedIntVisitor(), severityFind->second);

                std::string dir =
                    std::visit(VariantToStringVisitor(), directionFind->second);
                if (((findThresholdLevel(severity, dir)) != threshold.level) ||
                    ((findThresholdDirection(severity, dir)) !=
                     threshold.direction))
                {
                    return; // not the droid we're looking for
                }

                std::variant<double> value(threshold.value);
                conn->async_method_call(
                    [](const boost::system::error_code& ec) {
                        if (ec)
                        {
                            std::cerr << "Error setting threshold " << ec
                                      << "\n";
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
    if (sensor->thresholds.empty())
    {
        return;
    }

    for (const auto& threshold : sensor->thresholds)
    {
        if (!findOrder(threshold.level, threshold.direction))
        {
            continue;
        }
        std::shared_ptr<sdbusplus::asio::dbus_interface> interface =
            sensor->getThresholdInterface(threshold.level);

        if (!interface)
        {
            continue;
        }

        std::string property =
            sensor->propertyLevel(threshold.level, threshold.direction);
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

struct ChangeParam
{
    ChangeParam(Threshold whichThreshold, bool status, double value) :
        threshold(whichThreshold), asserted(status), assertValue(value)
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
                    std::cerr << "Sensor " << sensor->name << " high threshold "
                              << threshold.value << " assert: value " << value
                              << " raw data " << sensor->rawValue << "\n";
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
                    std::cerr << "Sensor " << sensor->name << " low threshold "
                              << threshold.value << " assert: value "
                              << sensor->value << " raw data "
                              << sensor->rawValue << "\n";
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
            std::cerr << "Error determining threshold direction\n";
        }
    }

    // Throttle debug output, so that it does not continuously spam
    ++cDebugThrottle;
    if (cDebugThrottle >= 1000)
    {
        cDebugThrottle = 0;
        if constexpr (debug)
        {
            std::cerr << "checkThresholds: High T=" << cHiTrue
                      << " F=" << cHiFalse << " M=" << cHiMidstate
                      << ", Low T=" << cLoTrue << " F=" << cLoFalse
                      << " M=" << cLoMidstate << "\n";
        }
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
        pair = &timers.emplace_back(timerUsed, boost::asio::deadline_timer(io));
    }

    pair->first.used = true;
    pair->first.level = threshold.level;
    pair->first.direction = threshold.direction;
    pair->first.assert = assert;
    pair->second.expires_from_now(boost::posix_time::seconds(waitTime));
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
            std::cerr << "timer error: " << ec.message() << "\n";
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

void assertThresholds(Sensor* sensor, double assertValue,
                      thresholds::Level level, thresholds::Direction direction,
                      bool assert)
{
    if (!findOrder(level, direction))
    {
        return;
    }

    std::shared_ptr<sdbusplus::asio::dbus_interface> interface =
        sensor->getThresholdInterface(level);

    if (!interface)
    {
        std::cout << "trying to set uninitialized interface\n";
        return;
    }

    std::string property = sensor->propertyAlarm(level, direction);
    if (property.empty())
    {
        std::cout << "Alarm property is empty \n";
        return;
    }
    if (interface->set_property<bool, true>(property, assert))
    {
        try
        {
            // msg.get_path() is interface->get_object_path()
            sdbusplus::message::message msg =
                interface->new_signal("ThresholdAsserted");

            msg.append(sensor->name, interface->get_interface_name(), property,
                       assert, assertValue);
            msg.signal_send();
        }
        catch (const sdbusplus::exception::exception& e)
        {
            std::cerr
                << "Failed to send thresholdAsserted signal with assertValue\n";
        }
    }
}

bool parseThresholdsFromAttr(
    std::vector<thresholds::Threshold>& thresholdVector,
    const std::string& inputPath, const double& scaleFactor,
    const double& offset)
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
        if (map.count(item) != 0)
        {
            for (const auto& t : map.at(item))
            {
                auto& [suffix, level, direction, offset] = t;
                auto attrPath =
                    boost::replace_all_copy(inputPath, item, suffix);
                if (auto val = readFile(attrPath, scaleFactor))
                {
                    *val += offset;
                    if (debug)
                    {
                        std::cout << "Threshold: " << attrPath << ": " << *val
                                  << "\n";
                    }
                    thresholdVector.emplace_back(level, direction, *val);
                }
            }
        }
    }
    return true;
}

std::string getInterface(const Level thresholdLevel)
{
    std::string level;
    switch (thresholdLevel)
    {
        case Level::WARNING:
            level = "Warning";
            break;
        case Level::CRITICAL:
            level = "Critical";
            break;
        case Level::SOFTSHUTDOWN:
            level = "SoftShutdown";
            break;
        case Level::HARDSHUTDOWN:
            level = "HardShutdown";
            break;
        case Level::ERROR:
            level = "Error";
            break;
    }
    std::string interface = "xyz.openbmc_project.Sensor.Threshold." + level;
    return interface;
}
} // namespace thresholds
