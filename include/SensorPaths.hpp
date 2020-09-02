#pragma once

#include <cstring>
#include <string>

namespace sensors
{

// This is the leading prefix of the object path for sensors.
// The full path is /xyz/openbmc_project/sensors/<Measure>/<Name>
// Note C++20 would allow this to be "constexpr std::string"
const char* objectPathPrefix = "/xyz/openbmc_project/sensors/";

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

} // namespace sensors
