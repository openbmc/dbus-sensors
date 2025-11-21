#include "SensorPaths.hpp"

#include <map>
#include <regex>
#include <string>
#include <string_view>

namespace sensor_paths
{

// This is an allowlist of the units a sensor can measure. Should be in sync
// with
// phosphor-dbus-interfaces/blob/master/yaml/xyz/openbmc_project/Sensor/Value.interface.yaml#L38

std::string getPathForUnits(const std::string& units)
{
    if (units == "DegreesC" || units == unitDegreesC)
    {
        return "temperature";
    }
    if (units == "RPMS" || units == unitRPMs)
    {
        return "fan_tach";
    }
    if (units == "Volts" || units == unitVolts)
    {
        return "voltage";
    }
    if (units == "Meters" || units == unitMeters)
    {
        return "altitude";
    }
    if (units == "Amperes" || units == unitAmperes)
    {
        return "current";
    }
    if (units == "Watts" || units == unitWatts)
    {
        return "power";
    }
    if (units == "Joules" || units == unitJoules)
    {
        return "energy";
    }
    if (units == "Percent" || units == unitPercent)
    {
        return "utilization";
    }
    if (units == "Pascals" || units == unitPascals)
    {
        return "pressure";
    }
    return "";
}

std::string escapePathForDbus(const std::string& name)
{
    return std::regex_replace(name, std::regex("[^a-zA-Z0-9_/]+"), "_");
}

std::string convertToFullUnits(const std::string& units)
{
    static const std::map<std::string_view, std::string_view> unitsMap = {
        {"DegreesC", sensor_paths::unitDegreesC},
        {"RPMS", sensor_paths::unitRPMs},
        {"Volts", sensor_paths::unitVolts},
        {"Meters", sensor_paths::unitMeters},
        {"Amperes", sensor_paths::unitAmperes},
        {"Joules", sensor_paths::unitJoules},
        {"Percent", sensor_paths::unitPercent},
        {"CFM", sensor_paths::unitCFM},
        {"Pascals", sensor_paths::unitPascals},
        {"PercentRH", sensor_paths::unitPercentRH},
    };

    auto it = unitsMap.find(units);
    if (it != unitsMap.end())
    {
        return std::string(it->second);
    }
    return "";
}

} // namespace sensor_paths
