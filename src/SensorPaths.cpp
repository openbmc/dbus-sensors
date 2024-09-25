#include "SensorPaths.hpp"

#include <regex>
#include <string>
#include <unordered_map>

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
    if (units == "LPM" || units == unitLPM)
    {
        return "liquidflow";
    }
    if (units == "Unclassified" || units == unitUnclassified)
    {
        return "unclassified";
    }
    return "";
}

static const std::unordered_map<std::string, std::string> unitMap = {
    {"DegreesC", unitDegreesC},
    {"RPMS", unitRPMs},
    {"Volts", unitVolts},
    {"Meters", unitMeters},
    {"Amperes", unitAmperes},
    {"Watts", unitWatts},
    {"Joules", unitJoules},
    {"Percent", unitPercent},
    {"Pascals", unitPascals},
    {"LPM", unitLPM},
    {"Unclassified", unitUnclassified}};

std::string getUnitPath(const std::string& unitName)
{
    auto it = unitMap.find(unitName);
    if (it != unitMap.end())
    {
        return it->second;
    }
    return "";
}

std::string escapePathForDbus(const std::string& name)
{
    return std::regex_replace(name, std::regex("[^a-zA-Z0-9_/]+"), "_");
}

} // namespace sensor_paths
