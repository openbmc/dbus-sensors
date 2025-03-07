#pragma once

#include "VariantVisitors.hpp"

#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <regex>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

const constexpr char* jsonStore = "/var/configuration/flattened.json";
const constexpr char* inventoryPath = "/xyz/openbmc_project/inventory";
const constexpr char* entityManagerName = "xyz.openbmc_project.EntityManager";

constexpr const char* cpuInventoryPath =
    "/xyz/openbmc_project/inventory/system/chassis/motherboard";
const std::regex illegalDbusRegex("[^A-Za-z0-9_]");

using BasicVariantType =
    std::variant<std::vector<std::string>, std::vector<uint8_t>,
                 std::vector<std::uint64_t>, std::string, int64_t, uint64_t,
                 double, int32_t, uint32_t, int16_t, uint16_t, uint8_t, bool>;
using SensorBaseConfigMap =
    boost::container::flat_map<std::string, BasicVariantType>;
using SensorBaseConfiguration = std::pair<std::string, SensorBaseConfigMap>;
using SensorData = boost::container::flat_map<std::string, SensorBaseConfigMap>;
using ManagedObjectType =
    boost::container::flat_map<sdbusplus::message::object_path, SensorData>;

using GetSubTreeType = std::vector<
    std::pair<std::string,
              std::vector<std::pair<std::string, std::vector<std::string>>>>>;
using Association = std::tuple<std::string, std::string, std::string>;

inline std::string escapeName(const std::string& sensorName)
{
    return boost::replace_all_copy(sensorName, " ", "_");
}

enum class PowerState
{
    on,
    biosPost,
    always,
    chassisOn
};

std::optional<std::string> openAndRead(const std::string& hwmonFile);
std::optional<std::string> getFullHwmonFilePath(
    const std::string& directory, const std::string& hwmonBaseName,
    const std::set<std::string>& permitSet);
std::set<std::string> getPermitSet(const SensorBaseConfigMap& config);
bool findFiles(const std::filesystem::path& dirPath,
               std::string_view matchString,
               std::vector<std::filesystem::path>& foundPaths,
               int symlinkDepth = 1);
bool isPowerOn();
bool hasBiosPost();
bool isChassisOn();
void setupPowerMatchCallback(
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    std::function<void(PowerState type, bool state)>&& callback);
void setupPowerMatch(const std::shared_ptr<sdbusplus::asio::connection>& conn);
bool getSensorConfiguration(
    const std::string& type,
    const std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    ManagedObjectType& resp, bool useCache);

void createAssociation(
    std::shared_ptr<sdbusplus::asio::dbus_interface>& association,
    const std::string& path);

// replaces limits if MinReading and MaxReading are found.
void findLimits(std::pair<double, double>& limits,
                const SensorBaseConfiguration* data);

bool readingStateGood(const PowerState& powerState);

constexpr const char* configInterfacePrefix =
    "xyz.openbmc_project.Configuration.";

inline std::string configInterfaceName(const std::string& type)
{
    return std::string(configInterfacePrefix) + type;
}

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
constexpr const char* set = "Set";
} // namespace properties

namespace power
{
const static constexpr char* busname = "xyz.openbmc_project.State.Host0";
const static constexpr char* interface = "xyz.openbmc_project.State.Host";
const static constexpr char* path = "/xyz/openbmc_project/state/host0";
const static constexpr char* property = "CurrentHostState";
} // namespace power

namespace chassis
{
const static constexpr char* busname = "xyz.openbmc_project.State.Chassis0";
const static constexpr char* interface = "xyz.openbmc_project.State.Chassis";
const static constexpr char* path = "/xyz/openbmc_project/state/chassis0";
const static constexpr char* property = "CurrentPowerState";
const static constexpr char* sOn = ".On";
} // namespace chassis

namespace post
{
const static constexpr char* busname = "xyz.openbmc_project.State.Host0";
const static constexpr char* interface =
    "xyz.openbmc_project.State.OperatingSystem.Status";
const static constexpr char* path = "/xyz/openbmc_project/state/host0";
const static constexpr char* property = "OperatingSystemState";
} // namespace post

namespace association
{
const static constexpr char* interface =
    "xyz.openbmc_project.Association.Definitions";
} // namespace association

template <typename T>
inline T loadVariant(const SensorBaseConfigMap& data, const std::string& key)
{
    auto it = data.find(key);
    if (it == data.end())
    {
        lg2::error("Configuration missing '{KEY}'", "KEY", key);
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
        static_assert(!std::is_same_v<T, T>, "Type Not Implemented");
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
    else if (str == "ChassisOn")
    {
        val = PowerState::chassisOn;
    }
}

inline PowerState getPowerState(const SensorBaseConfigMap& cfg)
{
    PowerState state = PowerState::always;
    auto findPowerState = cfg.find("PowerState");
    if (findPowerState != cfg.end())
    {
        std::string powerState =
            std::visit(VariantToStringVisitor(), findPowerState->second);
        setReadState(powerState, state);
    }
    return state;
}

inline float getPollRate(const SensorBaseConfigMap& cfg, float dflt)
{
    float pollRate = dflt;
    auto findPollRate = cfg.find("PollRate");
    if (findPollRate != cfg.end())
    {
        pollRate = std::visit(VariantToFloatVisitor(), findPollRate->second);
        if (!std::isfinite(pollRate) || pollRate <= 0.0F)
        {
            pollRate = dflt; // poll time invalid, fall back to default
        }
    }
    return pollRate;
}

inline void setLed(const std::shared_ptr<sdbusplus::asio::connection>& conn,
                   const std::string& name, bool on)
{
    conn->async_method_call(
        [name](const boost::system::error_code ec) {
            if (ec)
            {
                lg2::error("Failed to set LED '{NAME}'", "NAME", name);
            }
        },
        "xyz.openbmc_project.LED.GroupManager",
        "/xyz/openbmc_project/led/groups/" + name, properties::interface,
        properties::set, "xyz.openbmc_project.Led.Group", "Asserted",
        std::variant<bool>(on));
}

void createInventoryAssoc(
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& association,
    const std::string& path);

struct GetSensorConfiguration :
    std::enable_shared_from_this<GetSensorConfiguration>
{
    GetSensorConfiguration(
        std::shared_ptr<sdbusplus::asio::connection> connection,
        std::function<void(ManagedObjectType& resp)>&& callbackFunc) :
        dbusConnection(std::move(connection)), callback(std::move(callbackFunc))
    {}

    void getPath(const std::string& path, const std::string& interface,
                 const std::string& owner, size_t retries = 5)
    {
        if (retries > 5)
        {
            retries = 5;
        }
        std::shared_ptr<GetSensorConfiguration> self = shared_from_this();

        self->dbusConnection->async_method_call(
            [self, path, interface, owner, retries](
                const boost::system::error_code ec, SensorBaseConfigMap& data) {
                if (ec)
                {
                    if (retries == 0U)
                    {
                        lg2::error("Error getting '{PATH}': no retries left",
                                   "PATH", path);
                        return;
                    }
                    lg2::error(
                        "Error getting '{PATH}': '{RETRIES}' retries left",
                        "PATH", path, "RETRIES", retries - 1);
                    auto timer = std::make_shared<boost::asio::steady_timer>(
                        self->dbusConnection->get_io_context());
                    timer->expires_after(std::chrono::seconds(10));
                    timer->async_wait([self, timer, path, interface, owner,
                                       retries](boost::system::error_code ec) {
                        if (ec)
                        {
                            lg2::error("Timer error: '{ERROR_MESSAGE}'",
                                       "ERROR_MESSAGE", ec.message());
                            return;
                        }
                        self->getPath(path, interface, owner, retries - 1);
                    });
                    return;
                }

                self->respData[path][interface] = std::move(data);
            },
            owner, path, "org.freedesktop.DBus.Properties", "GetAll",
            interface);
    }

    void getConfiguration(const std::vector<std::string>& types,
                          size_t retries = 0)
    {
        if (retries > 5)
        {
            retries = 5;
        }

        std::vector<std::string> interfaces;
        interfaces.reserve(types.size());
        for (const auto& type : types)
        {
            interfaces.push_back(configInterfaceName(type));
        }

        std::shared_ptr<GetSensorConfiguration> self = shared_from_this();
        dbusConnection->async_method_call(
            [self, interfaces, retries](const boost::system::error_code ec,
                                        const GetSubTreeType& ret) {
                if (ec)
                {
                    lg2::error("Error calling mapper: '{ERROR_MESSAGE}'",
                               "ERROR_MESSAGE", ec.message());
                    if (retries == 0U)
                    {
                        return;
                    }
                    auto timer = std::make_shared<boost::asio::steady_timer>(
                        self->dbusConnection->get_io_context());
                    timer->expires_after(std::chrono::seconds(10));
                    timer->async_wait([self, timer, interfaces,
                                       retries](boost::system::error_code ec) {
                        if (ec)
                        {
                            lg2::error("Timer error: '{ERROR_MESSAGE}'",
                                       "ERROR_MESSAGE", ec.message());
                            return;
                        }
                        self->getConfiguration(interfaces, retries - 1);
                    });

                    return;
                }
                for (const auto& [path, objDict] : ret)
                {
                    if (objDict.empty())
                    {
                        return;
                    }
                    const std::string& owner = objDict.begin()->first;

                    for (const std::string& interface : objDict.begin()->second)
                    {
                        // anything that starts with a requested configuration
                        // is good
                        if (std::find_if(
                                interfaces.begin(), interfaces.end(),
                                [interface](const std::string& possible) {
                                    return interface.starts_with(possible);
                                }) == interfaces.end())
                        {
                            continue;
                        }
                        self->getPath(path, interface, owner);
                    }
                }
            },
            mapper::busName, mapper::path, mapper::interface, mapper::subtree,
            "/", 0, interfaces);
    }

    ~GetSensorConfiguration()
    {
        callback(respData);
    }

    std::shared_ptr<sdbusplus::asio::connection> dbusConnection;
    std::function<void(ManagedObjectType& resp)> callback;
    ManagedObjectType respData;
};

// The common scheme for sysfs files naming is: <type><number>_<item>.
// This function returns optionally these 3 elements as a tuple.
std::optional<std::tuple<std::string, std::string, std::string>> splitFileName(
    const std::filesystem::path& filePath);
std::optional<double> readFile(const std::string& thresholdFile,
                               const double& scaleFactor);
void setupManufacturingModeMatch(sdbusplus::asio::connection& conn);
bool getManufacturingMode();
std::vector<std::unique_ptr<sdbusplus::bus::match_t>>
    setupPropertiesChangedMatches(
        sdbusplus::asio::connection& bus, std::span<const char* const> types,
        const std::function<void(sdbusplus::message_t&)>& handler);

template <typename T>
bool getDeviceBusAddr(const std::string& deviceName, T& bus, T& addr)
{
    auto findHyphen = deviceName.find('-');
    if (findHyphen == std::string::npos)
    {
        lg2::error("found bad device '{NAME}'", "NAME", deviceName);
        return false;
    }
    std::string busStr = deviceName.substr(0, findHyphen);
    std::string addrStr = deviceName.substr(findHyphen + 1);

    std::from_chars_result res{};
    res = std::from_chars(&*busStr.begin(), &*busStr.end(), bus);
    if (res.ec != std::errc{} || res.ptr != &*busStr.end())
    {
        lg2::error("Error finding bus for '{NAME}'", "NAME", deviceName);
        return false;
    }
    res = std::from_chars(&*addrStr.begin(), &*addrStr.end(), addr, 16);
    if (res.ec != std::errc{} || res.ptr != &*addrStr.end())
    {
        lg2::error("Error finding addr for '{NAME}'", "NAME", deviceName);
        return false;
    }

    return true;
}
