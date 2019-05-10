#pragma once
#include "VariantVisitors.hpp"

#include <systemd/sd-journal.h>

#include <boost/container/flat_map.hpp>
#include <filesystem>
#include <iostream>
#include <regex>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/types.hpp>

const constexpr char* jsonStore = "/var/configuration/flattened.json";
const constexpr char* inventoryPath = "/xyz/openbmc_project/inventory";
const constexpr char* entityManagerName = "xyz.openbmc_project.EntityManager";
const std::regex illegalDbusRegex("[^A-Za-z0-9_]");

using BasicVariantType =
    std::variant<std::vector<std::string>, std::string, int64_t, uint64_t,
                 double, int32_t, uint32_t, int16_t, uint16_t, uint8_t, bool>;

using ManagedObjectType = boost::container::flat_map<
    sdbusplus::message::object_path,
    boost::container::flat_map<
        std::string,
        boost::container::flat_map<std::string, BasicVariantType>>>;
using SensorData = boost::container::flat_map<
    std::string, boost::container::flat_map<std::string, BasicVariantType>>;

using GetSubTreeType = std::vector<
    std::pair<std::string,
              std::vector<std::pair<std::string, std::vector<std::string>>>>>;
using SensorBaseConfiguration =
    std::pair<std::string,
              boost::container::flat_map<std::string, BasicVariantType>>;

bool findFiles(const std::filesystem::path dirPath,
               const std::string& matchString,
               std::vector<std::filesystem::path>& foundPaths,
               unsigned int symlinkDepth = 1);
bool isPowerOn(void);
bool hasBiosPost(void);
void setupPowerMatch(const std::shared_ptr<sdbusplus::asio::connection>& conn);
bool getSensorConfiguration(
    const std::string& type,
    const std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    ManagedObjectType& resp, bool useCache = false);

void createAssociation(
    std::shared_ptr<sdbusplus::asio::dbus_interface>& association,
    const std::string& path);

// replaces limits if MinReading and MaxReading are found.
void findLimits(std::pair<double, double>& limits,
                const SensorBaseConfiguration* data);

enum class PowerState
{
    on,
    biosPost,
    always
};

namespace mapper
{
constexpr const char* busName = "xyz.openbmc_project.ObjectMapper";
constexpr const char* path = "/xyz/openbmc_project/object_mapper";
constexpr const char* interface = "xyz.openbmc_project.ObjectMapper";
constexpr const char* subtree = "GetSubTree";
} // namespace mapper

namespace properties
{
constexpr const char* interface = "org.freedesktop.DBus.Properties";
constexpr const char* get = "Get";
} // namespace properties

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
        return std::visit(VariantToDoubleVisitor(), it->second);
    }
    else if constexpr (std::is_unsigned_v<T>)
    {
        return std::visit(VariantToUnsignedIntVisitor(), it->second);
    }
    else if constexpr (std::is_same_v<T, std::string>)
    {
        return std::visit(VariantToStringVisitor(), it->second);
    }
    else
    {
        static_assert("Type Not Implemented");
    }
}

inline void setReadState(const std::string& str, PowerState& val)
{

    if (str == "On")
    {
        val = PowerState::on;
    }
    else if (str == "BiosPost")
    {
        val = PowerState::biosPost;
    }
    else if (str == "Always")
    {
        val = PowerState::always;
    }
}

inline void logDeviceAdded(const std::string& device)
{

    sd_journal_send("MESSAGE=%s", "Inventory Added", "PRIORITY=%i", LOG_ERR,
                    "REDFISH_MESSAGE_ID=%s", "OpenBMC.0.1.InventoryAdded",
                    "REDFISH_MESSAGE_ARGS=%s", device.c_str(), NULL);
}

inline void logDeviceRemoved(const std::string& device)
{

    sd_journal_send("MESSAGE=%s", "Inventory Removed", "PRIORITY=%i", LOG_ERR,
                    "REDFISH_MESSAGE_ID=%s", "OpenBMC.0.1.InventoryRemoved",
                    "REDFISH_MESSAGE_ARGS=%s", device.c_str(), NULL);
}
