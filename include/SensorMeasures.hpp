#pragma once

#include <string>
#include <unordered_set>

namespace sensors
{

// Sensors are at /xyz/openbmc_project/sensors/<Measure>/<Name>
// C++20 would allow this to be "constexpr std::string"
const char* objectPath{"/xyz/openbmc_project/sensors/"};

bool InAllowedMeasures(const std::string& measure)
{
    // This is an allowlist of the various physical characteristics
    // that a sensor can measure. When developing a new sensor type,
    // add to this vector, as needed. Keep this vector in sync with
    // other usages, as needed, such as your IPMI or Redfish services.
    const std::unordered_set<std::string> allowedMeasures{
        "cfm",   "current",     "fan_pwm", "fan_tach",
        "power", "temperature", "voltage",
    };

    return (allowedMeasures.find(measure) != allowedMeasures.end());
}

} // namespace sensors
