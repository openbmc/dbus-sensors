#include "SensorPaths.hpp"

#include <regex>
#include <string>

namespace sensor_paths
{

// This is an allowlist of the units a sensor can measure.

std::string getPathForUnits(const std::string& units)
{
    if (units == "DegreesC" || units == unitDegreesC)
    {
        return SensorValue::namespace_path::temperature;
    }
    if (units == "RPMS" || units == unitRPMs)
    {
        return SensorValue::namespace_path::fan_tach;
    }
    if (units == "Volts" || units == unitVolts)
    {
        return SensorValue::namespace_path::voltage;
    }
    if (units == "Meters" || units == unitMeters)
    {
        return SensorValue::namespace_path::altitude;
    }
    if (units == "Amperes" || units == unitAmperes)
    {
        return SensorValue::namespace_path::current;
    }
    if (units == "Watts" || units == unitWatts)
    {
        return SensorValue::namespace_path::power;
    }
    if (units == "Joules" || units == unitJoules)
    {
        return SensorValue::namespace_path::energy;
    }
    if (units == "Percent" || units == unitPercent)
    {
        return SensorValue::namespace_path::utilization;
    }
    if (units == "Pascals" || units == unitPascals)
    {
        return SensorValue::namespace_path::pressure;
    }
    return "";
}

std::string escapePathForDbus(const std::string& name)
{
    return std::regex_replace(name, std::regex("[^a-zA-Z0-9_/]+"), "_");
}

std::string convertToFullUnits(const std::string& units)
{
    for (const auto& unit : unitsMap)
    {
        if (unit.dbusUnitName == units)
        {
            return std::string(unit.dbusPathName);
        }
    }
    return "";
}

} // namespace sensor_paths
