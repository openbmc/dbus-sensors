#include "SensorPaths.hpp"

#include <regex>
#include <string>

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
    if (units == "DegreesF" || units == unitDegreesF)
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
        return "Utilization";
    }
    if (units == "Pascals" || units == unitPascals)
    {
        return "pressure";
    }
    if (units == "CF" || units == unitCF)
    {
        return "volume";
    }
    if (units == "CM" || units == unitCM)
    {
        return "volume";
    }
    if (units == "CMH" || units == unitCMH)
    {
        return "volume";
    }
    if (units == "CMS" || units == unitCMS)
    {
        return "volume";
    }
    if (units == "Gallon" || units == unitGallon)
    {
        return "volume";
    }
    if (units == "GPM" || units == unitGPM)
    {
        return "volume";
    }
    if (units == "Litre" || units == unitLitre)
    {
        return "volume";
    }
    if (units == "LPH" || units == unitLPH)
    {
        return "volume";
    }
    if (units == "LPM" || units == unitLPM)
    {
        return "volume";
    }
    if (units == "LPS" || units == unitLPS)
    {
        return "volume";
    }
    return "";
}

std::string escapePathForDbus(const std::string& name)
{
    return std::regex_replace(name, std::regex("[^a-zA-Z0-9_/]+"), "_");
}

} // namespace sensor_paths
