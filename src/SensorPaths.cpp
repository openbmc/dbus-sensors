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

std::string escapePathForDbus(const std::string& name)
{
    // This is only here to meet the API of sd_bus_path_encode()
    // https://www.freedesktop.org/software/systemd/man/sd_bus_path_encode.html
    std::string prefix = "/x";

    char* outbuf = nullptr;
    int ret = sd_bus_path_encode(prefix.c_str(), name.c_str(), &outbuf);

    // Error checking
    std::string result(name);
    if (ret < 0)
    {
        return result;
    }
    if (!outbuf)
    {
        return result;
    }

    // Convert C string to C++ string
    std::string outstr(outbuf);
    free(outbuf);

    // Ensure prefix and added slash are as expected
    prefix += '/';
    auto prefixsize = prefix.size();
    if (outstr.size() <= prefixsize)
    {
        return result;
    }
    if (outstr.compare(0, prefixsize, prefix) != 0)
    {
        return result;
    }

    // Return only the name
    result = outstr.substr(prefixsize);
    return result;
}

} // namespace sensor_paths
