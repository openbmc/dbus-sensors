#include <Thresholds.hpp>
#include <VariantVisitors.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/lexical_cast.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sensor.hpp>

static constexpr bool DEBUG = false;
namespace thresholds
{
unsigned int toBusValue(const Level& level)
{
    switch (level)
    {
        case (Level::WARNING):
        {
            return 0;
        }
        case (Level::CRITICAL):
        {
            return 1;
        }
        default:
        {
            return -1;
        }
    }
}

std::string toBusValue(const Direction& direction)
{
    switch (direction)
    {
        case (Direction::LOW):
        {
            return "less than";
        }
        case (Direction::HIGH):
        {
            return "greater than";
        }
        default:
        {
            return "err";
        }
    }
}

bool parseThresholdsFromConfig(
    const SensorData& sensorData,
    std::vector<thresholds::Threshold>& thresholdVector,
    const std::string* matchLabel)
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
                continue;
            if (std::visit(VariantToStringVisitor(), labelFind->second) !=
                *matchLabel)
                continue;
        }
        auto directionFind = item.second.find("Direction");
        auto severityFind = item.second.find("Severity");
        auto valueFind = item.second.find("Value");
        if (valueFind == item.second.end() ||
            severityFind == item.second.end() ||
            directionFind == item.second.end())
        {
            std::cerr << "Malformed threshold in configuration\n";
            return false;
        }
        Level level;
        Direction direction;
        if (std::visit(VariantToUnsignedIntVisitor(), severityFind->second) ==
            0)
        {
            level = Level::WARNING;
        }
        else
        {
            level = Level::CRITICAL;
        }
        if (std::visit(VariantToStringVisitor(), directionFind->second) ==
            "less than")
        {
            direction = Direction::LOW;
        }
        else
        {
            direction = Direction::HIGH;
        }
        double val = std::visit(VariantToDoubleVisitor(), valueFind->second);

        thresholdVector.emplace_back(level, direction, val);
    }
    return true;
}

void persistThreshold(const std::string& path, const std::string& baseInterface,
                      const thresholds::Threshold& threshold,
                      std::shared_ptr<sdbusplus::asio::connection>& conn,
                      size_t thresholdCount)
{
    for (int ii = 0; ii < thresholdCount; ii++)
    {
        std::string thresholdInterface =
            baseInterface + ".Thresholds" + std::to_string(ii);
        conn->async_method_call(
            [&, path, threshold, thresholdInterface](
                const boost::system::error_code& ec,
                const boost::container::flat_map<std::string, BasicVariantType>&
                    result) {
                if (ec)
                {
                    return; // threshold not supported
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
                unsigned int level = std::visit(VariantToUnsignedIntVisitor(),
                                                severityFind->second);

                std::string dir =
                    std::visit(VariantToStringVisitor(), directionFind->second);
                if ((toBusValue(threshold.level) != level) ||
                    (toBusValue(threshold.direction) != dir))
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
        std::shared_ptr<sdbusplus::asio::dbus_interface> interface;
        std::string property;
        if (threshold.level == thresholds::Level::CRITICAL)
        {
            interface = sensor->thresholdInterfaceCritical;
            if (threshold.direction == thresholds::Direction::HIGH)
            {
                property = "CriticalHigh";
            }
            else
            {
                property = "CriticalLow";
            }
        }
        else if (threshold.level == thresholds::Level::WARNING)
        {
            interface = sensor->thresholdInterfaceWarning;
            if (threshold.direction == thresholds::Direction::HIGH)
            {
                property = "WarningHigh";
            }
            else
            {
                property = "WarningLow";
            }
        }
        else
        {
            continue;
        }
        if (!interface)
        {
            continue;
        }
        interface->set_property(property, threshold.value);
    }
}

static std::vector<std::pair<Threshold, bool>> checkThresholds(Sensor* sensor,
                                                               double value)
{
    std::vector<std::pair<Threshold, bool>> thresholdChanges;
    if (sensor->thresholds.empty())
    {
        return thresholdChanges;
    }

    for (auto& threshold : sensor->thresholds)
    {
        if (threshold.direction == thresholds::Direction::HIGH)
        {
            if (value > threshold.value)
            {
                thresholdChanges.push_back(std::make_pair(threshold, true));
            }
            else
            {
                thresholdChanges.push_back(std::make_pair(threshold, false));
            }
        }
        else
        {
            if (value < threshold.value)
            {
                thresholdChanges.push_back(std::make_pair(threshold, true));
            }
            else
            {
                thresholdChanges.push_back(std::make_pair(threshold, false));
            }
        }
    }
    return thresholdChanges;
}

bool checkThresholds(Sensor* sensor)
{
    bool status = true;
    std::vector<std::pair<Threshold, bool>> changes =
        checkThresholds(sensor, sensor->value);
    for (const auto& [threshold, asserted] : changes)
    {
        assertThresholds(sensor, threshold.level, threshold.direction,
                         asserted);
        if (threshold.level == thresholds::Level::CRITICAL && asserted)
        {
            status = false;
        }
    }

    return status;
}

void checkThresholdsPowerDelay(Sensor* sensor, ThresholdTimer& thresholdTimer)
{

    std::vector<std::pair<Threshold, bool>> changes =
        checkThresholds(sensor, sensor->value);
    for (const auto& [threshold, asserted] : changes)
    {
        if (asserted)
        {
            thresholdTimer.startTimer(threshold);
        }
        else
        {
            assertThresholds(sensor, threshold.level, threshold.direction,
                             false);
        }
    }
}

void assertThresholds(Sensor* sensor, thresholds::Level level,
                      thresholds::Direction direction, bool assert)
{
    std::string property;
    std::shared_ptr<sdbusplus::asio::dbus_interface> interface;
    if (level == thresholds::Level::WARNING &&
        direction == thresholds::Direction::HIGH)
    {
        property = "WarningAlarmHigh";
        interface = sensor->thresholdInterfaceWarning;
    }
    else if (level == thresholds::Level::WARNING &&
             direction == thresholds::Direction::LOW)
    {
        property = "WarningAlarmLow";
        interface = sensor->thresholdInterfaceWarning;
    }
    else if (level == thresholds::Level::CRITICAL &&
             direction == thresholds::Direction::HIGH)
    {
        property = "CriticalAlarmHigh";
        interface = sensor->thresholdInterfaceCritical;
    }
    else if (level == thresholds::Level::CRITICAL &&
             direction == thresholds::Direction::LOW)
    {
        property = "CriticalAlarmLow";
        interface = sensor->thresholdInterfaceCritical;
    }
    else
    {
        std::cerr << "Unknown threshold, level " << level << "direction "
                  << direction << "\n";
        return;
    }
    if (!interface)
    {
        std::cout << "trying to set uninitialized interface\n";
        return;
    }
    interface->set_property(property, assert);
}

static constexpr std::array<const char*, 4> attrTypes = {"lcrit", "min", "max",
                                                         "crit"};

bool parseThresholdsFromAttr(
    std::vector<thresholds::Threshold>& thresholdVector,
    const std::string& inputPath, const double& scaleFactor,
    const double& offset)
{
    for (const std::string& type : attrTypes)
    {
        auto attrPath = boost::replace_all_copy(inputPath, "input", type);
        std::ifstream attrFile(attrPath);
        if (!attrFile.good())
        {
            continue;
        }
        std::string attr;
        std::getline(attrFile, attr);
        attrFile.close();

        Level level;
        Direction direction;
        double val;
        try
        {
            val = std::stod(attr) / scaleFactor;
        }
        catch (const std::invalid_argument&)
        {
            return false;
        }

        if (type == "min" || type == "max")
        {
            level = Level::WARNING;
        }
        else
        {
            level = Level::CRITICAL;
        }
        if (type == "min" || type == "lcrit")
        {
            direction = Direction::LOW;
        }
        else
        {
            direction = Direction::HIGH;
        }

        if (type == "crit")
        {
            val += offset;
        }

        if (DEBUG)
        {
            std::cout << "Threshold: " << attrPath << ": " << val << "\n";
        }

        thresholdVector.emplace_back(level, direction, val);
    }
    // no thresholds is allowed, not an error so return true.
    return true;
}

bool hasCriticalInterface(
    const std::vector<thresholds::Threshold>& thresholdVector)
{
    for (auto& threshold : thresholdVector)
    {
        if (threshold.level == Level::CRITICAL)
        {
            return true;
        }
    }
    return false;
}

bool hasWarningInterface(
    const std::vector<thresholds::Threshold>& thresholdVector)
{
    for (auto& threshold : thresholdVector)
    {
        if (threshold.level == Level::WARNING)
        {
            return true;
        }
    }
    return false;
}
} // namespace thresholds
