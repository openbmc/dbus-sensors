#include <Thresholds.hpp>
#include <VariantVisitors.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <iostream>
#include <sensor.hpp>

static constexpr bool DEBUG = false;
constexpr size_t MAX_THRESHOLDS = 4;

namespace thresholds
{
unsigned int toBusValue(const Level &level)
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

std::string toBusValue(const Direction &direction)
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

bool ParseThresholdsFromConfig(
    const SensorData &sensorData,
    std::vector<thresholds::Threshold> &thresholdVector,
    const std::string *matchLabel)
{
    for (const auto &item : sensorData)
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
            if (mapbox::util::apply_visitor(VariantToStringVisitor(),
                                            labelFind->second) != *matchLabel)
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
        if (mapbox::util::apply_visitor(VariantToUnsignedIntVisitor(),
                                        severityFind->second) == 0)
        {
            level = Level::WARNING;
        }
        else
        {
            level = Level::CRITICAL;
        }
        if (mapbox::util::apply_visitor(VariantToStringVisitor(),
                                        directionFind->second) == "less than")
        {
            direction = Direction::LOW;
        }
        else
        {
            direction = Direction::HIGH;
        }
        float val = mapbox::util::apply_visitor(VariantToFloatVisitor(),
                                                valueFind->second);

        thresholdVector.emplace_back(level, direction, val);
    }
    return true;
}

void persistThreshold(const std::string &path, const std::string &baseInterface,
                      const thresholds::Threshold &threshold,
                      std::shared_ptr<sdbusplus::asio::connection> &conn)
{
    for (int ii = 0; ii < MAX_THRESHOLDS; ii++)
    {
        std::string thresholdInterface =
            baseInterface + ".Thresholds" + std::to_string(ii);
        conn->async_method_call(
            [&, path, threshold, thresholdInterface](
                const boost::system::error_code &ec,
                const boost::container::flat_map<std::string, BasicVariantType>
                    &result) {
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
                unsigned int level = mapbox::util::apply_visitor(
                    VariantToUnsignedIntVisitor(), severityFind->second);

                std::string dir = mapbox::util::apply_visitor(
                    VariantToStringVisitor(), directionFind->second);
                if ((toBusValue(threshold.level) != level) ||
                    (toBusValue(threshold.direction) != dir))
                {
                    return; // not the droid we're looking for
                }

                sdbusplus::message::variant<double> value(threshold.value);
                conn->async_method_call(
                    [](const boost::system::error_code &ec) {
                        if (ec)
                        {
                            std::cerr << "Error setting threshold " << ec
                                      << "\n";
                        }
                    },
                    ENTITY_MANAGER_NAME, path,
                    "org.freedesktop.DBus.Properties", "Set",
                    thresholdInterface, "Value", value);
            },
            ENTITY_MANAGER_NAME, path, "org.freedesktop.DBus.Properties",
            "GetAll", thresholdInterface);
    }
}

void checkThresholds(Sensor *sensor)
{

    if (sensor->thresholds.empty())
    {
        return;
    }
    for (auto &threshold : sensor->thresholds)
    {
        if (threshold.direction == thresholds::Direction::HIGH)
        {
            if (sensor->value > threshold.value && !threshold.asserted)
            {
                assertThresholds(sensor, threshold.level, threshold.direction,
                                 true);
                threshold.asserted = true;
            }
            else if (sensor->value <= threshold.value && threshold.asserted)
            {
                assertThresholds(sensor, threshold.level, threshold.direction,
                                 false);
                threshold.asserted = false;
            }
        }
        else
        {
            if (sensor->value < threshold.value && !threshold.asserted)
            {
                assertThresholds(sensor, threshold.level, threshold.direction,
                                 true);
                threshold.asserted = true;
            }
            else if (sensor->value >= threshold.value && threshold.asserted)
            {
                assertThresholds(sensor, threshold.level, threshold.direction,
                                 false);
                threshold.asserted = false;
            }
        }
    }
}

void assertThresholds(Sensor *sensor, thresholds::Level level,
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

static constexpr std::array<const char *, 4> ATTR_TYPES = {"lcrit", "min",
                                                           "max", "crit"};

bool ParseThresholdsFromAttr(
    std::vector<thresholds::Threshold> &threshold_vector,
    const std::string &input_path, const double &scale_factor)
{
    for (auto &type : ATTR_TYPES)
    {
        auto attr_path = boost::replace_all_copy(input_path, "input", type);
        std::ifstream attr_file(attr_path);
        if (!attr_file.good())
            continue;
        std::string attr;
        std::getline(attr_file, attr);
        attr_file.close();

        Level level;
        Direction direction;
        double val = std::stod(attr) / scale_factor;
        if (type == "min" || type == "max")
            level = Level::WARNING;
        else
            level = Level::CRITICAL;
        if (type == "min" || type == "lcrit")
            direction = Direction::LOW;
        else
            direction = Direction::HIGH;

        if (DEBUG)
            std::cout << "Threshold: " << attr_path << ": " << val << "\n";

        threshold_vector.emplace_back(level, direction, val);
    }
    // no thresholds is allowed, not an error so return true always
    return true;
}

bool HasCriticalInterface(
    const std::vector<thresholds::Threshold> &threshold_vector)
{
    for (auto &threshold : threshold_vector)
    {
        if (threshold.level == Level::CRITICAL)
            return true;
    }
    return false;
}

bool HasWarningInterface(
    const std::vector<thresholds::Threshold> &threshold_vector)
{
    for (auto &threshold : threshold_vector)
    {
        if (threshold.level == Level::WARNING)
            return true;
    }
    return false;
}
} // namespace thresholds
