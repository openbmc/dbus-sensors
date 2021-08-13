#include <SensorPaths.hpp>
#include <sdbusplus/message/types.hpp>

#include <cstring>
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
    return "";
}

std::string escapePathForDbus(const std::string& name)
{
    return (sdbusplus::message::object_path("/") / name).str.substr(1);
}

} // namespace sensor_paths
