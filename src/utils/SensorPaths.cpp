#include "SensorPaths.hpp"

#include <regex>
#include <string>
#include <string_view>

namespace sensor_paths
{

// This is an allowlist of the units a sensor can measure. Should be in sync
// with
// phosphor-dbus-interfaces/blob/master/yaml/xyz/openbmc_project/Sensor/Value.interface.yaml#L38

std::string getPathForUnits(std::string_view units)
{
    for (const auto& unit : unitsMap)
    {
        if (unit.dbusUnitName == units || unit.dbusPathName == units)
        {
            return std::string(unit.emUnitName);
        }
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
