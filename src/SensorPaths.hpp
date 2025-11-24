#pragma once

#include <array>
#include <string>

namespace sensor_paths
{

struct UnitMap {
    std::string_view dbusPathName;
    std::string_view dbusUnitName;
    std::string_view emUnitName;
};

constexpr std::array<UnitMap, 10> UNITS_MAP = {{
    {"xyz.openbmc_project.Sensor.Value.Unit.DegreesC", "DegreesC",
     "temperature"},
    {"xyz.openbmc_project.Sensor.Value.Unit.RPMS", "RPMS", "fan_tach"},
    {"xyz.openbmc_project.Sensor.Value.Unit.Volts", "Volts", "voltage"},
    {"xyz.openbmc_project.Sensor.Value.Unit.Meters", "Meters", "distance"},
    {"xyz.openbmc_project.Sensor.Value.Unit.Amperes", "Amperes", "current"},
    {"xyz.openbmc_project.Sensor.Value.Unit.Joules", "Joules", "energy"},
    {"xyz.openbmc_project.Sensor.Value.Unit.Percent", "Percent", "percent"},
    {"xyz.openbmc_project.Sensor.Value.Unit.CFM", "CFM", "airflow"},
    {"xyz.openbmc_project.Sensor.Value.Unit.Pascals", "Pascals", "pressure"},
    {"xyz.openbmc_project.Sensor.Value.Unit.PercentRH", "PercentRH",
     "humidity"},
}};
// This is an allowlist of the units a sensor can measure. Should be in sync
// with
// phosphor-dbus-interfaces/blob/master/yaml/xyz/openbmc_project/Sensor/Value.interface.yaml#L38

constexpr const char* unitDegreesC =
    "xyz.openbmc_project.Sensor.Value.Unit.DegreesC";
constexpr const char* unitRPMs = "xyz.openbmc_project.Sensor.Value.Unit.RPMS";
constexpr const char* unitVolts = "xyz.openbmc_project.Sensor.Value.Unit.Volts";
constexpr const char* unitMeters =
    "xyz.openbmc_project.Sensor.Value.Unit.Meters";
constexpr const char* unitAmperes =
    "xyz.openbmc_project.Sensor.Value.Unit.Amperes";
constexpr const char* unitWatts = "xyz.openbmc_project.Sensor.Value.Unit.Watts";
constexpr const char* unitJoules =
    "xyz.openbmc_project.Sensor.Value.Unit.Joules";
constexpr const char* unitPercent =
    "xyz.openbmc_project.Sensor.Value.Unit.Percent";
constexpr const char* unitCFM = "xyz.openbmc_project.Sensor.Value.Unit.CFM";
constexpr const char* unitPascals =
    "xyz.openbmc_project.Sensor.Value.Unit.Pascals";
constexpr const char* unitPercentRH =
    "xyz.openbmc_project.Sensor.Value.Unit.PercentRH";

std::string getPathForUnits(const std::string& units);

std::string escapePathForDbus(const std::string& name);

std::string convertToFullUnits(const std::string& units);

} // namespace sensor_paths
