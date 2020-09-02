#pragma once

#include <set>
#include <string>

namespace sensors
{

// Sensors are at /xyz/openbmc_project/sensors/<Measure>/<Name>
const std::string objectPath{"/xyz/openbmc_project/sensors/"};

// This is an allowlist of the various physical characteristics
// that a sensor can measure. When developing a new sensor type,
// add to this vector, as needed. Keep this vector in sync with
// other usages, as needed, such as your IPMI or Redfish services.
const std::set<std::string> allowedMeasures{
    "cfm", "current", "fan_pwm", "fan_tach", "power", "temperature", "voltage",
};

bool InAllowedMeasures(const std::string& measure)
{
    return (allowedMeasures.find(measure) != allowedMeasures.end());
}

} // namespace sensors
