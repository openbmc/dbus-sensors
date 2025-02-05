#include "DeviceMgmt.hpp"

#include "Utils.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <variant>

namespace fs = std::filesystem;

std::optional<I2CDeviceParams> getI2CDeviceParams(
    const I2CDeviceTypeMap& dtmap, const SensorBaseConfigMap& cfg)
{
    auto findType = cfg.find("Type");
    auto findBus = cfg.find("Bus");
    auto findAddr = cfg.find("Address");

    if (findType == cfg.end() || findBus == cfg.end() || findAddr == cfg.end())
    {
        return std::nullopt;
    }

    const std::string* type = std::get_if<std::string>(&findType->second);
    const uint64_t* bus = std::get_if<uint64_t>(&findBus->second);
    const uint64_t* addr = std::get_if<uint64_t>(&findAddr->second);

    if (type == nullptr || bus == nullptr || addr == nullptr)
    {
        return std::nullopt;
    }

    auto findDevType = dtmap.find(type->c_str());
    if (findDevType == dtmap.end())
    {
        return std::nullopt;
    }

    return I2CDeviceParams(findDevType->second, *bus, *addr);
}

static fs::path i2cBusPath(uint64_t bus)
{
    return {"/sys/bus/i2c/devices/i2c-" + std::to_string(bus)};
}

static std::string deviceDirName(uint64_t bus, uint64_t address)
{
    std::ostringstream name;
    name << bus << "-" << std::hex << std::setw(4) << std::setfill('0')
         << address;
    return name.str();
}

// check if device has already been added to bus
bool I2CDeviceParams::devicePresent() const
{
    fs::path path = i2cBusPath(bus) / deviceDirName(bus, address);

    // Ignore errors; anything but a clean 'true' is fine as 'false'
    std::error_code ec;
    return fs::exists(path, ec);
}

// check if bindable-devices have been bound
bool I2CDeviceParams::deviceReady() const
{
    fs::path path = i2cBusPath(bus) / deviceDirName(bus, address);

    if (type->createsHWMon)
    {
        path /= "hwmon";
    }

    // Ignore errors; anything but a clean 'true' is fine as 'false'
    std::error_code ec;
    return fs::exists(path, ec);
}

bool I2CDeviceParams::deviceStatic() const
{
    if (!deviceReady())
    {
        return false;
    }

    fs::path ofNode = i2cBusPath(bus) / deviceDirName(bus, address) / "of_node";

    // Ignore errors -- if of_node is present the device is a static DT node;
    // otherwise we can assume we created it from userspace.
    std::error_code ec;
    return fs::exists(ofNode, ec);
}

I2CDevice::I2CDevice(I2CDeviceParams params) : params(params)
{
    if (create() != 0)
    {
        throw std::runtime_error("failed to instantiate i2c device");
    }
}

I2CDevice::~I2CDevice()
{
    destroy();
}

int I2CDevice::create() const
{
    // If it's already instantiated, there's nothing we need to do.
    if (params.deviceReady())
    {
        return 0;
    }
    // If device is already added to the bus, but an expected driver binding
    // hasn't happened yet
    if (!params.deviceReady() &&
        params.devicePresent()) // if device is 'present' (handle exists on
                                // bus) but not 'ready' (missing an
                                // expected hwmon node)
    {
        fs::path ctorPath = "/sys/bus/i2c/drivers_probe";
        std::ofstream ctor(ctorPath);

        if (!ctor.good())
        {
            std::cerr << "Failed to open " << ctorPath << "\n";
            return -1;
        }

        ctor << deviceDirName(params.bus, params.address) << "\n";

        ctor.flush();
        if (!ctor.good())
        {
            std::cerr << "Failed to write to " << ctorPath << "\n";
            return -1;
        }
    }

    else // if device does not exist at all
    {
        // Try to create it: 'echo $devtype $addr > .../i2c-$bus/new_device'
        fs::path ctorPath = i2cBusPath(params.bus) / "new_device";
        std::ofstream ctor(ctorPath);
        if (!ctor.good())
        {
            std::cerr << "Failed to open " << ctorPath << "\n";
            return -1;
        }

        ctor << params.type->name << " " << params.address << "\n";
        ctor.flush();
        if (!ctor.good())
        {
            std::cerr << "Failed to write to " << ctorPath << "\n";
            return -1;
        }
    }

    // Check if that created the requisite sysfs directory
    if (!params.deviceReady())
    {
        destroy();
        return -1;
    }

    return 0;
}

int I2CDevice::destroy() const
{
    // No params.deviceReady() check on this like in create(), since it
    // might be used to clean up after a device instantiation that was only
    // partially successful (i.e. when params.deviceReady() would return
    // false but there's still a dummy i2c client device to remove)

    fs::path dtorPath = i2cBusPath(params.bus) / "delete_device";
    std::ofstream dtor(dtorPath);
    if (!dtor.good())
    {
        std::cerr << "Failed to open " << dtorPath << "\n";
        return -1;
    }

    dtor << params.address << "\n";
    dtor.flush();

    if (!dtor.good())
    {
        // Writing to delete_device will fail for devices added to bus by a
        // driver instead a daemon In that case, unbind any driver for the
        // device instead

        fs::path unbindPath =
            i2cBusPath(params.bus) / deviceDirName(params.bus, params.address) /
            "driver/unbind";
        std::ofstream unbind(unbindPath);
        if (!unbind.good())
        {
            // indicates there is no bound driver to unbind
            return 0;
        }

        dtor << deviceDirName(params.bus, params.address) << "\n";
        unbind.flush();
        if (!unbind.good())
        {
            std::cerr << "Failed to writes to " << unbindPath << " and to "
                      << dtorPath << "\n";
            return -1;
        }

        return 0;
    }

    return 0;
}
