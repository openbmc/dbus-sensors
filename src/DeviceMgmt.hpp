#pragma once

#include "Utils.hpp"

#include <boost/container/flat_map.hpp>

#include <functional>
#include <optional>
#include <string_view>

struct I2CDeviceType
{
    const char* name;
    bool createsHWMon;
};

struct I2CDeviceComparator
{
    bool operator()(const std::string& a, const std::string& b) const noexcept
    {
        return strcasecmp(a.c_str(), b.c_str()) < 0;
    }
};

using I2CDeviceTypeMap =
    boost::container::flat_map<std::string, I2CDeviceType, I2CDeviceComparator>;

struct I2CDeviceParams
{
    I2CDeviceParams(const I2CDeviceType& type, uint64_t bus, uint64_t address) :
        type(&type), bus(bus), address(address){};

    const I2CDeviceType* type;
    uint64_t bus;
    uint64_t address;

    bool devicePresent(void) const;
    bool deviceStatic(void) const;
};

std::optional<I2CDeviceParams>
    getI2CDeviceParams(const I2CDeviceTypeMap& dtmap,
                       const SensorBaseConfigMap& cfg);

class I2CDevice
{
  public:
    explicit I2CDevice(I2CDeviceParams params);
    ~I2CDevice();

  private:
    I2CDeviceParams params;

    int create(void) const;
    int destroy(void) const;
};

// HACK: this declaration "should" live in Utils.hpp, but that leads to a
// tangle of header-dependency hell because each header needs types declared
// in the other.
std::vector<std::unique_ptr<sdbusplus::bus::match_t>>
    setupPropertiesChangedMatches(
        sdbusplus::asio::connection& bus, const I2CDeviceTypeMap& typeMap,
        const std::function<void(sdbusplus::message_t&)>& handler);
