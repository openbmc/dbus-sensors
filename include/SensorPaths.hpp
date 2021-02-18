#pragma once

#include <cstring>
#include <regex>
#include <string>

namespace sensor_paths
{

// This is an allowlist of the units a sensor can measure. Should be in sync
// with
// phosphor-dbus-interfaces/blob/master/xyz/openbmc_project/Sensor/Value.interface.yaml#L35

std::string getPathForUnits(const std::string& units);

std::string escapePathForDbus(const std::string& name);

} // namespace sensor_paths
