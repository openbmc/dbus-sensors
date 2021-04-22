#include <cstring>
#include <regex>
#include <string>

namespace sensor_paths
{

// This is an allowlist of the units a sensor can measure. Should be in sync
// with
// phosphor-dbus-interfaces/blob/master/xyz/openbmc_project/Sensor/Value.interface.yaml#L35

std::string getPathForUnits(const std::string& fullUnits)
{
    // So we can accept either "xyz.openbmc_project.Sensor.Value.Unit.Foo" or
    // just a bare "Foo"
    size_t lastDot = fullUnits.find_last_of('.');
    size_t unitStart = (lastDot == std::string::npos) ? 0 : lastDot + 1;
    const std::string& units = fullUnits.substr(unitStart);

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
