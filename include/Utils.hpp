#pragma once
#include "VariantVisitors.hpp"
#include "filesystem.hpp"

#include <boost/container/flat_map.hpp>
#include <iostream>
#include <regex>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/message/types.hpp>

const constexpr char* jsonStore = "/var/configuration/flattened.json";
const constexpr char* inventoryPath = "/xyz/openbmc_project/inventory";
const constexpr char* entityManagerName = "xyz.openbmc_project.EntityManager";
const std::regex illegalDbusRegex("[^A-Za-z0-9_]");

using BasicVariantType =
    sdbusplus::message::variant<std::vector<std::string>, std::string, int64_t,
                                uint64_t, double, int32_t, uint32_t, int16_t,
                                uint16_t, uint8_t, bool>;

using ManagedObjectType = boost::container::flat_map<
    sdbusplus::message::object_path,
    boost::container::flat_map<
        std::string,
        boost::container::flat_map<std::string, BasicVariantType>>>;
using SensorData = boost::container::flat_map<
    std::string, boost::container::flat_map<std::string, BasicVariantType>>;

using SensorBaseConfiguration =
    std::pair<std::string,
              boost::container::flat_map<std::string, BasicVariantType>>;

bool findFiles(const std::filesystem::path dirPath,
               const std::string& matchString,
               std::vector<std::filesystem::path>& foundPaths,
               unsigned int symlinkDepth = 1);
bool isPowerOn(void);
void setupPowerMatch(const std::shared_ptr<sdbusplus::asio::connection>& conn);
bool getSensorConfiguration(
    const std::string& type,
    const std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    ManagedObjectType& resp, bool useCache = false);

// replaces limits if MinReading and MaxReading are found.
void findLimits(std::pair<double, double>& limits,
                const SensorBaseConfiguration* data);

enum class PowerState : bool
{
    on,
    always
};

template <typename T>
inline T loadVariant(
    const boost::container::flat_map<std::string, BasicVariantType>& data,
    const std::string& key)
{
    auto it = data.find(key);
    if (it == data.end())
    {
        std::cerr << "Configuration missing " << key << "\n";
        throw std::invalid_argument("Key Missing");
    }
    if constexpr (std::is_same_v<T, double>)
    {
        return sdbusplus::message::variant_ns::visit(VariantToDoubleVisitor(),
                                                     it->second);
    }
    else if constexpr (std::is_unsigned_v<T>)
    {
        return sdbusplus::message::variant_ns::visit(
            VariantToUnsignedIntVisitor(), it->second);
    }
    else if constexpr (std::is_same_v<T, std::string>)
    {
        return sdbusplus::message::variant_ns::visit(VariantToStringVisitor(),
                                                     it->second);
    }
    else
    {
        static_assert("Type Not Implemented");
    }
}
