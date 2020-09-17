#include <SensorPaths.hpp>
#include <Utils.hpp>

#include <cstring>
#include <regex>
#include <string>

namespace sensor_paths
{

// This is an allowlist of the units a sensor can measure. Should be in sync
// with
// phosphor-dbus-interfaces/blob/master/xyz/openbmc_project/Sensor/Value.interface.yaml#L35

std::string getPathForUnits(const std::string_view& units)
{
    std::string sensorType = "";
    if (units == Unit::DegreesC)
    {
        sensorType = "temperature";
    }
    if (units == Unit::RPMS)
    {
        sensorType = "fan_tach";
    }
    if (units == Unit::Volts)
    {
        sensorType = "voltage";
    }
    if (units == Unit::Meters)
    {
        sensorType = "altitude";
    }
    if (units == Unit::Amperes)
    {
        sensorType = "current";
    }
    if (units == Unit::Watts)
    {
        sensorType = "power";
    }
    if (units == Unit::Joules)
    {
        sensorType = "energy";
    }
    if (units == Unit::Percent)
    {
        sensorType = "Utilization";
    }
    if (units == Unit::CFM)
    {
        sensorType = "cfm";
    }
    if (sensorType.empty())
    {
        return "";
    }
    return sensor_paths::prefix + sensorType + "/";
}

std::string_view getUnits(const std::string& units)
{
    if (units == "DegreesC")
    {
        return Unit::DegreesC;
    }
    if (units == "RPMS")
    {
        return Unit::RPMS;
    }
    if (units == "Volts")
    {
        return Unit::Volts;
    }
    if (units == "Meters")
    {
        return Unit::Meters;
    }
    if (units == "Amperes")
    {
        return Unit::Amperes;
    }
    if (units == "Watts")
    {
        return Unit::Watts;
    }
    if (units == "Joules")
    {
        return Unit::Joules;
    }
    if (units == "Percent")
    {
        return Unit::Percent;
    }
    if (units == "CFM")
    {
        return Unit::CFM;
    }
    return "";
}

std::string escapePathForDbus(const std::string& name)
{
    return std::regex_replace(name, std::regex("[^a-zA-Z0-9_/]+"), "_");
}

} // namespace sensor_paths
