#pragma once
#include <Utils.hpp>

#include <boost/container/flat_map.hpp>

#include <functional>
#include <optional>
#include <string_view>

struct I2CDeviceType
{
    const char* name;
    bool createsHWMon;
};

using I2CDeviceTypeMap =
    boost::container::flat_map<std::string_view, I2CDeviceType, std::less<>>;

struct I2CDevice
{
    I2CDevice(const I2CDeviceType& type, uint64_t bus, uint64_t address) :
        type(&type), bus(bus), address(address){};

    const I2CDeviceType* type;
    uint64_t bus;
    uint64_t address;

    bool present(void) const;
    int create(void) const;
    int destroy(void) const;
};

std::optional<I2CDevice> getI2CDevice(const I2CDeviceTypeMap& dtmap,
                                      const SensorBaseConfigMap& cfg);
