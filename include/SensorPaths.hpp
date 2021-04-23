#pragma once

#include <cstring>
#include <regex>
#include <string>

namespace sensor_paths
{

// This is an allowlist of the units a sensor can measure. Should be in sync
// with
// phosphor-dbus-interfaces/blob/master/xyz/openbmc_project/Sensor/Value.interface.yaml#L35

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

std::string getPathForUnits(const std::string& units);

std::string escapePathForDbus(const std::string& name);

} // namespace sensor_paths
