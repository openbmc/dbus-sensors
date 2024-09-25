#pragma once

#include <cstring>
#include <regex>
#include <string>

namespace sensor_paths
{

// This is an allowlist of the units a sensor can measure. Should be in sync
// with
// phosphor-dbus-interfaces/blob/master/yaml/xyz/openbmc_project/Sensor/Value.interface.yaml#L38

constexpr const char* unitDegreesC =
    "xyz.openbmc_project.Sensor.Value.Unit.DegreesC";
constexpr const char* unitDegreesF =
    "xyz.openbmc_project.Sensor.Value.Unit.DegreesF";
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
constexpr const char* unitCF = "xyz.openbmc_project.Sensor.Value.Unit.CF";
constexpr const char* unitCFM = "xyz.openbmc_project.Sensor.Value.Unit.CFM";
constexpr const char* unitPascals =
    "xyz.openbmc_project.Sensor.Value.Unit.Pascals";
constexpr const char* unitPercentRH =
    "xyz.openbmc_project.Sensor.Value.Unit.PercentRH";
constexpr const char* unitCM =
    "xyz.openbmc_project.Sensor.Value.Unit.CM";
constexpr const char* unitCMH =
    "xyz.openbmc_project.Sensor.Value.Unit.CMH";
constexpr const char* unitCMS =
    "xyz.openbmc_project.Sensor.Value.Unit.CMS";
constexpr const char* unitGallon =
    "xyz.openbmc_project.Sensor.Value.Unit.Gallon";
constexpr const char* unitGPM =
    "xyz.openbmc_project.Sensor.Value.Unit.GPM";
constexpr const char* unitLiter =
    "xyz.openbmc_project.Sensor.Value.Unit.Liter";
constexpr const char* unitLPH =
    "xyz.openbmc_project.Sensor.Value.Unit.LPH";
constexpr const char* unitLPM =
    "xyz.openbmc_project.Sensor.Value.Unit.LPM";
constexpr const char* unitLPS =
    "xyz.openbmc_project.Sensor.Value.Unit.LPS";

std::string getPathForUnits(const std::string& units);

std::string escapePathForDbus(const std::string& name);

} // namespace sensor_paths
