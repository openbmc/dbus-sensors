#pragma once

#include <cstring>
#include <string>

namespace sensors
{

// This is the leading prefix of the object path for sensors.
// The full path is /xyz/openbmc_project/sensors/<Measure>/<Name>
// Note C++20 would allow this to be "constexpr std::string"
const char* objectPath{"/xyz/openbmc_project/sensors/"};

// This is an allowlist of the various physical characteristics that a sensor
// can measure. When developing a new sensor type, add to this vector,
// as needed. Keep this vector in sync with other usages, as needed, such as
// your IPMI or Redfish services.
// Note C++20 would allow this to be "constexpr std::vector<std::string>"
const char* allowedMeasures[]{"cfm",   "current",     "fan_pwm", "fan_tach",
                              "power", "temperature", "voltage", nullptr};

bool InAllowedMeasures(const std::string& measure)
{
    for (const char** p = allowedMeasures; *p != nullptr; ++p)
    {
        if (std::strcmp(measure.c_str(), *p) == 0)
        {
            return true;
        }
    }
    return false;
}

} // namespace sensors
