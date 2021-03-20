#include <systemd/sd-bus.h>

#include <cstring>
#include <string>

namespace sensor_paths
{

// This is an allowlist of the units a sensor can measure. Should be in sync
// with
// phosphor-dbus-interfaces/blob/master/xyz/openbmc_project/Sensor/Value.interface.yaml#L35

std::string getPathForUnits(const std::string& units)
{
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

// Escapes unacceptable characters in an object name, making it safe for use
// in a D-Bus object path. The prefix is also required here, although it
// remains unchanged, to meet the API requirements of sd_bus_path_encode():
// https://www.freedesktop.org/software/systemd/man/sd_bus_path_encode.html
// https://github.com/systemd/systemd/tree/main/src/libsystemd/sd-bus/sd-bus.c
// Returns: String, concatenation of the (unchanged) prefix, the '/' (slash)
// character, and the escaped object name. Returns empty string if error.
std::string escapePathForDbus(const std::string& prefix,
                              const std::string& name)
{
    char* outbuf = nullptr;
    int ret = sd_bus_path_encode(prefix.c_str(), name.c_str(), &outbuf);

    // Error checking
    if ((ret < 0) || !outbuf)
    {
        return {};
    }

    // Convert C string to C++ string
    std::string outstr(outbuf);
    free(outbuf);

    return outstr;
}

} // namespace sensor_paths
