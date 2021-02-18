#include <cstring>
#include <regex>
#include <string>

namespace sensor_paths
{

// This is an allowlist of the units a sensor can measure. Should be in sync
// with
// phosphor-dbus-interfaces/blob/master/xyz/openbmc_project/Sensor/Value.interface.yaml#L35

std::string getPathForUnits(const std::string& units)
{
    if (units == "DegreesC")
    {
        return "temperature";
    }
    if (units == "RPMS")
    {
        return "fan_tach";
    }
    if (units == "Volts")
    {
        return "voltage";
    }
    if (units == "Meters")
    {
        return "altitude";
    }
    if (units == "Amperes")
    {
        return "current";
    }
    if (units == "Watts")
    {
        return "power";
    }
    if (units == "Joules")
    {
        return "energy";
    }
    if (units == "Percent")
    {
        return "Utilization";
    }
    return "";
}

std::string escapePathForDbus(const std::string& name)
{
    return std::regex_replace(name, std::regex("[^a-zA-Z0-9_/]+"), "_");
}

} // namespace sensor_paths
