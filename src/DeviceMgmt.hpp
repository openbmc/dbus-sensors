#pragma once

#include "Utils.hpp"

#include <strings.h>

#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace fs = std::filesystem;

// the /dev/i2c-mux/ directory should contain the symbolic links between the
// ChannelNames of the mux and the logical bus number as created in
// entity-manager linkMux function
// https://github.com/openbmc/entity-manager/blob/master/src/overlay.cpp#L71
static constexpr const char* defaultMuxDir = "/dev/i2c-mux/";

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
        type(&type), bus(bus), address(address) {};

    const I2CDeviceType* type;
    uint64_t bus;
    uint64_t address;

    bool devicePresent() const;
    bool deviceStatic() const;
};

std::optional<I2CDeviceParams> getI2CDeviceParams(
    const I2CDeviceTypeMap& dtmap, const SensorBaseConfigMap& cfg);

class I2CDevice
{
  public:
    explicit I2CDevice(I2CDeviceParams params);
    ~I2CDevice();

  private:
    I2CDeviceParams params;

    int create() const;
    int destroy() const;
};

class I2CBus
{
  public:
    explicit I2CBus() = default;
    explicit I2CBus(int logicalBus) : logicalBus(logicalBus) {}

    int getBus() const
    {
        return logicalBus;
    }

    ~I2CBus() = default;

  private:
    int logicalBus;
};

class I2CMux
{
  public:
    I2CMux() = delete;

    explicit I2CMux(std::string muxName,
                    const std::string muxDir = defaultMuxDir)
    {
        muxName = std::regex_replace(muxName, illegalDbusRegex, "_");
        muxPath = fs::path(muxDir) / muxName;
        if (muxPath.empty())
        {
            throw std::runtime_error(muxPath.string() + "does not exist");
        }
    }

    std::optional<I2CBus> getLogicalBus(std::string chName) const
    {
        fs::path busLink(muxPath / chName);
        if (busLink.empty() || !fs::is_symlink(busLink))
        {
            std::cerr << "ChannelName symlink is missing" << std::endl;
            return std::nullopt;
        }
        // retrieve the i2c-#
        std::string busPath(fs::read_symlink(busLink).filename());

        // remove "i2c-"
        std::string i2cPrefix = "i2c-";
        auto findPrefix = busPath.find(i2cPrefix);
        busPath.erase(findPrefix, i2cPrefix.size());
        I2CBus bus(std::stoi(busPath));
        return bus;
    }
    ~I2CMux() = default;

    static std::optional<I2CMux> findMux(
        const std::string& baseIntf, const SensorData& cfgData,
        const std::string& path, std::string& channelName,
        const std::string muxDir = defaultMuxDir)
    {
        const auto& muxChannelBase = cfgData.find(baseIntf + ".MuxChannel");
        std::string muxName;
        try
        {
            if (muxChannelBase == cfgData.end())
            {
                throw std::logic_error(
                    "No Bus or MuxChannel config for " + path);
            }
            const SensorBaseConfigMap& muxChannelIntf = muxChannelBase->second;
            auto findMuxName = muxChannelIntf.find("MuxName");
            auto findChName = muxChannelIntf.find("ChannelName");
            if (findMuxName == muxChannelIntf.end() ||
                findChName == muxChannelIntf.end())
            {
                throw std::logic_error(
                    "Can't find Mux or Channel name for " + path);
            }
            if (std::get_if<std::string>(&findMuxName->second) == nullptr ||
                std::get_if<std::string>(&findChName->second) == nullptr)
            {
                throw std::logic_error(
                    "Mux or Channel name invalid for " + path);
            }
            channelName = std::get<std::string>(findChName->second);
            muxName = std::get<std::string>(findMuxName->second);
            I2CMux mux(muxName, muxDir);
            return mux;
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            return std::nullopt;
        }
    }

  private:
    fs::path muxPath;
};

// HACK: this declaration "should" live in Utils.hpp, but that leads to a
// tangle of header-dependency hell because each header needs types declared
// in the other.
std::vector<std::unique_ptr<sdbusplus::bus::match_t>>
    setupPropertiesChangedMatches(
        sdbusplus::asio::connection& bus, const I2CDeviceTypeMap& typeMap,
        const std::function<void(sdbusplus::message_t&)>& handler);

// Helper find function because some sensors use underscores in their names
// while others use spaces. Ignore this discrepancy by changing all spaces to
// underscores.
inline size_t sensorNameFind(const std::string& fullName,
                             const std::string& partialName)
{
    return boost::replace_all_copy(fullName, " ", "_")
        .find(boost::replace_all_copy(partialName, " ", "_"));
}

// returns a {path: <I2CDevice, is_new>} map.  is_new indicates if the I2CDevice
// is newly instantiated by this call (true) or was already there (false).
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

            const auto* sensorName =
                std::get_if<std::string>(&findSensorName->second);
            if (sensorName == nullptr)
            {
                std::cerr << "Unable to find sensor name " << name
                          << " on path " << path.str << "\n";
                continue;
            }

            std::shared_ptr<T> findSensor(nullptr);
            for (const auto& sensor : sensors)
            {
                if (sensorNameFind(sensor.first, *sensorName) !=
                    std::string::npos)
                {
                    findSensor = sensor.second;
                    break;
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
