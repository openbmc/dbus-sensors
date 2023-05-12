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

template <class T>
boost::container::flat_map<std::string,
                           std::pair<std::shared_ptr<I2CDevice>, bool>>
    instantiateDevices(
        const ManagedObjectType& sensorConfigs,
        const boost::container::flat_map<std::string, std::shared_ptr<T>>&
            sensors,
        const I2CDeviceTypeMap& sensorTypes)
{
    boost::container::flat_map<std::string,
                               std::pair<std::shared_ptr<I2CDevice>, bool>>
        devices;
    for (const auto& [path, sensor] : sensorConfigs)
    {
        for (const auto& [name, cfg] : sensor)
        {
            PowerState powerState = getPowerState(cfg);
            if (!readingStateGood(powerState))
            {
                continue;
            }

            auto findSensorName = cfg.find("Name");
            if (findSensorName == cfg.end())
            {
                continue;
            }

            std::string sensorName;
            try
            {
                sensorName = std::get<std::string>(findSensorName->second);
            }
            catch (const std::bad_variant_access& e)
            {
                std::cerr << e.what() << ": Unable to find sensor name.\n";
                continue;
            }

            boost::replace_all(sensorName, " ", "_");

            std::shared_ptr<T> findSensor(nullptr);
            for (const auto& sensor : sensors)
            {
                if (sensor.first.find(sensorName) != std::string::npos)
                {
                    findSensor = sensor.second;
                }
            }

            if (findSensor != nullptr && findSensor->isActive())
            {
                devices.emplace(
                    path.str,
                    std::make_pair(findSensor->getI2CDevice(), false));
                continue;
            }

            std::optional<I2CDeviceParams> params =
                getI2CDeviceParams(sensorTypes, cfg);
            if (params.has_value() && !params->deviceStatic())
            {
                // There exist error cases in which a sensor device that we
                // need is already instantiated, but needs to be destroyed and
                // re-created in order to be useful (for example if we crash
                // after instantiating a device and the sensor device's power
                // is cut before we get restarted, leaving it "present" but
                // not really usable).  To be on the safe side, instantiate a
                // temporary device that's immediately destroyed so as to
                // ensure that we end up with a fresh instance of it.
                if (params->devicePresent())
                {
                    std::cerr << "Clearing out previous instance for "
                              << path.str << "\n";
                    I2CDevice tmp(*params);
                }

                try
                {
                    devices.emplace(
                        path.str,
                        std::make_pair(std::make_shared<I2CDevice>(*params),
                                       true));
                }
                catch (std::runtime_error&)
                {
                    std::cerr << "Failed to instantiate " << params->type->name
                              << " at address " << params->address << " on bus "
                              << params->bus << "\n";
                }
            }
        }
    }
    return devices;
}
