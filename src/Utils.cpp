/*
// Copyright (c) 2017 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include "Utils.hpp"

#include "dbus-sensor_config.h"

#include "DeviceMgmt.hpp"
#include "VariantVisitors.hpp"

#include <boost/asio/error.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/exception.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
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

static bool manufacturingMode = false;

// Per-process registry of power-state instances, keyed by host/chassis slotId.
// The cached state lives inside each HostPowerState object (not loose global
// variables), so hosts are tracked independently. See Utils.hpp.
static boost::container::flat_map<size_t, std::shared_ptr<HostPowerState>>
    powerStateRegistry;

/**
 * return the contents of a file
 * @param[in] hwmonFile - the path to the file to read
 * @return the contents of the file as a string or nullopt if the file could not
 * be opened.
 */

std::optional<std::string> openAndRead(const std::string& hwmonFile)
{
    std::string fileVal;
    std::ifstream fileStream(hwmonFile);
    if (!fileStream.is_open())
    {
        return std::nullopt;
    }
    std::getline(fileStream, fileVal);
    return fileVal;
}

/**
 * given a hwmon temperature base name if valid return the full path else
 * nullopt
 * @param[in] directory - the hwmon sysfs directory
 * @param[in] permitSet - a set of labels or hwmon basenames to permit. If this
 * is empty then *everything* is permitted.
 * @return a string to the full path of the file to create a temp sensor with or
 * nullopt to indicate that no sensor should be created for this basename.
 */
std::optional<std::string> getFullHwmonFilePath(
    const std::string& directory, const std::string& hwmonBaseName,
    const std::set<std::string>& permitSet)
{
    std::optional<std::string> result;
    std::string filename;
    if (permitSet.empty())
    {
        result = directory + "/" + hwmonBaseName + "_input";
        return result;
    }
    filename = directory + "/" + hwmonBaseName + "_label";
    auto searchVal = openAndRead(filename);
    if (!searchVal)
    {
        /* if the hwmon temp doesn't have a corresponding label file
         * then use the hwmon temperature base name
         */
        searchVal = hwmonBaseName;
    }
    if (permitSet.contains(*searchVal))
    {
        result = directory + "/" + hwmonBaseName + "_input";
    }
    return result;
}

/**
 * retrieve a set of basenames and labels to allow sensor creation for.
 * @param[in] config - a map representing the configuration for a specific
 * device
 * @return a set of basenames and labels to allow sensor creation for. An empty
 * set indicates that everything is permitted.
 */
std::set<std::string> getPermitSet(const SensorBaseConfigMap& config)
{
    auto permitAttribute = config.find("Labels");
    std::set<std::string> permitSet;
    if (permitAttribute != config.end())
    {
        try
        {
            auto val =
                std::get<std::vector<std::string>>(permitAttribute->second);

            permitSet.insert(std::make_move_iterator(val.begin()),
                             std::make_move_iterator(val.end()));
        }
        catch (const std::bad_variant_access& err)
        {
            lg2::error(
                "'{ERROR_MESSAGE}': PermitList does not contain a list, wrong variant type.",
                "ERROR_MESSAGE", err.what());
        }
    }
    return permitSet;
}

bool getSensorConfiguration(
    std::string_view type,
    const std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    ManagedObjectType& resp, bool useCache)
{
    static ManagedObjectType managedObj;
    std::string typeIntf = configInterfaceName(type);

    if (!useCache)
    {
        managedObj.clear();
        sdbusplus::message_t getManagedObjects =
            dbusConnection->new_method_call(
                entityManagerName, "/xyz/openbmc_project/inventory",
                "org.freedesktop.DBus.ObjectManager", "GetManagedObjects");
        try
        {
            sdbusplus::message_t reply =
                dbusConnection->call(getManagedObjects);
            reply.read(managedObj);
        }
        catch (const sdbusplus::exception_t& e)
        {
            lg2::error(
                "While calling GetManagedObjects on service: '{SERVICE_NAME}'"
                " exception name: '{EXCEPTION_NAME}' and description: "
                "'{EXCEPTION_DESCRIPTION}' was thrown",
                "SERVICE_NAME", entityManagerName, "EXCEPTION_NAME", e.name(),
                "EXCEPTION_DESCRIPTION", e.description());
            return false;
        }
    }
    for (const auto& pathPair : managedObj)
    {
        for (const auto& [intf, cfg] : pathPair.second)
        {
            if (intf.starts_with(typeIntf))
            {
                resp.emplace(pathPair);
                break;
            }
        }
    }
    return true;
}

bool findFiles(const std::filesystem::path& dirPath,
               std::string_view matchString,
               std::vector<std::filesystem::path>& foundPaths, int symlinkDepth)
{
    std::error_code ec;
    if (!std::filesystem::exists(dirPath, ec))
    {
        return false;
    }

    std::vector<std::regex> matchPieces;

    size_t pos = 0;
    std::string token;
    // Generate the regex expressions list from the match we were given
    while ((pos = matchString.find('/')) != std::string::npos)
    {
        token = matchString.substr(0, pos);
        matchPieces.emplace_back(token);
        matchString.remove_prefix(pos + 1);
    }
    matchPieces.emplace_back(std::string{matchString});

    // Check if the match string contains directories, and skip the match of
    // subdirectory if not
    if (matchPieces.size() <= 1)
    {
        std::regex search(std::string{matchString});
        std::smatch match;
        for (auto p = std::filesystem::recursive_directory_iterator(
                 dirPath,
                 std::filesystem::directory_options::follow_directory_symlink);
             p != std::filesystem::recursive_directory_iterator(); ++p)
        {
            std::string path = p->path().string();
            if (!is_directory(*p))
            {
                if (std::regex_search(path, match, search))
                {
                    foundPaths.emplace_back(p->path());
                }
            }
            if (p.depth() >= symlinkDepth)
            {
                p.disable_recursion_pending();
            }
        }
        return true;
    }

    // The match string contains directories, verify each level of sub
    // directories
    for (auto p = std::filesystem::recursive_directory_iterator(
             dirPath,
             std::filesystem::directory_options::follow_directory_symlink);
         p != std::filesystem::recursive_directory_iterator(); ++p)
    {
        std::vector<std::regex>::iterator matchPiece = matchPieces.begin();
        std::filesystem::path::iterator pathIt = p->path().begin();
        for (const std::filesystem::path& dir : dirPath)
        {
            if (dir.empty())
            {
                // When the path ends with '/', it gets am empty path
                // skip such case.
                break;
            }
            pathIt++;
        }

        while (pathIt != p->path().end())
        {
            // Found a path deeper than match.
            if (matchPiece == matchPieces.end())
            {
                p.disable_recursion_pending();
                break;
            }
            std::smatch match;
            std::string component = pathIt->string();
            std::regex regexPiece(*matchPiece);
            if (!std::regex_match(component, match, regexPiece))
            {
                // path prefix doesn't match, no need to iterate further
                p.disable_recursion_pending();
                break;
            }
            matchPiece++;
            pathIt++;
        }

        if (!is_directory(*p))
        {
            if (matchPiece == matchPieces.end())
            {
                foundPaths.emplace_back(p->path());
            }
        }

        if (p.depth() >= symlinkDepth)
        {
            p.disable_recursion_pending();
        }
    }
    return true;
}

bool HostPowerState::isPowerOn() const
{
    return powerStatusOn;
}

bool HostPowerState::hasBiosPost() const
{
    return biosHasPost;
}

bool HostPowerState::isChassisOn() const
{
    return chassisStatusOn;
}

bool HostPowerState::readingStateGood(const PowerState& powerState) const
{
    if (powerState == PowerState::on && !powerStatusOn)
    {
        return false;
    }
    if (powerState == PowerState::biosPost && (!biosHasPost || !powerStatusOn))
    {
        return false;
    }
    if (powerState == PowerState::chassisOn && !chassisStatusOn)
    {
        return false;
    }

    return true;
}

bool isPowerOn(size_t slotId)
{
    auto it = powerStateRegistry.find(slotId);
    if (it == powerStateRegistry.end())
    {
        throw std::runtime_error("Power Match Not Created");
    }
    return it->second->isPowerOn();
}

bool hasBiosPost(size_t slotId)
{
    auto it = powerStateRegistry.find(slotId);
    if (it == powerStateRegistry.end())
    {
        throw std::runtime_error("Post Match Not Created");
    }
    return it->second->hasBiosPost();
}

bool isChassisOn(size_t slotId)
{
    auto it = powerStateRegistry.find(slotId);
    if (it == powerStateRegistry.end())
    {
        throw std::runtime_error("Chassis On Match Not Created");
    }
    return it->second->isChassisOn();
}

bool readingStateGood(const PowerState& powerState, size_t slotId)
{
    // Sensors that always read need no power domain.
    if (powerState == PowerState::always)
    {
        return true;
    }
    auto it = powerStateRegistry.find(slotId);
    if (it == powerStateRegistry.end())
    {
        // No power tracking registered for this host; do not gate the reading.
        return true;
    }
    return it->second->readingStateGood(powerState);
}

void HostPowerState::getInitialPowerStatus(size_t retries)
{
    std::weak_ptr<HostPowerState> weak = weak_from_this();
    conn->async_method_call(
        [weak, retries](boost::system::error_code ec,
                        const std::variant<std::string>& state) {
            auto self = weak.lock();
            if (!self)
            {
                return;
            }
            if (ec)
            {
                if (retries != 0U)
                {
                    auto timer = std::make_shared<boost::asio::steady_timer>(
                        self->conn->get_io_context());
                    timer->expires_after(std::chrono::seconds(15));
                    timer->async_wait(
                        [timer, weak, retries](boost::system::error_code) {
                            if (auto s = weak.lock())
                            {
                                s->getInitialPowerStatus(retries - 1);
                            }
                        });
                    return;
                }

                // we commonly come up before power control, we'll capture the
                // property change later
                lg2::error("error getting power status: '{ERROR_MESSAGE}'",
                           "ERROR_MESSAGE", ec.message());
                return;
            }
            self->powerStatusOn =
                std::get<std::string>(state).ends_with(".Running");
        },
        std::string(power::busname) + std::to_string(slotId),
        std::string(power::path) + std::to_string(slotId),
        properties::interface, properties::get, power::interface,
        power::property);
}

void HostPowerState::getInitialPostStatus(size_t retries)
{
    std::weak_ptr<HostPowerState> weak = weak_from_this();
    conn->async_method_call(
        [weak, retries](boost::system::error_code ec,
                        const std::variant<std::string>& state) {
            auto self = weak.lock();
            if (!self)
            {
                return;
            }
            if (ec)
            {
                if (retries != 0U)
                {
                    auto timer = std::make_shared<boost::asio::steady_timer>(
                        self->conn->get_io_context());
                    timer->expires_after(std::chrono::seconds(15));
                    timer->async_wait(
                        [timer, weak, retries](boost::system::error_code) {
                            if (auto s = weak.lock())
                            {
                                s->getInitialPostStatus(retries - 1);
                            }
                        });
                    return;
                }
                // we commonly come up before power control, we'll capture the
                // property change later
                lg2::error("error getting post status: '{ERROR_MESSAGE}'",
                           "ERROR_MESSAGE", ec.message());
                return;
            }
            const auto& value = std::get<std::string>(state);
            self->biosHasPost =
                (value != "Inactive") &&
                (value != "xyz.openbmc_project.State.OperatingSystem."
                          "Status.OSStatus.Inactive");
        },
        std::string(post::busname) + std::to_string(slotId),
        std::string(post::path) + std::to_string(slotId), properties::interface,
        properties::get, post::interface, post::property);
}

void HostPowerState::getInitialChassisStatus(size_t retries)
{
    std::weak_ptr<HostPowerState> weak = weak_from_this();
    conn->async_method_call(
        [weak, retries](boost::system::error_code ec,
                        const std::variant<std::string>& state) {
            auto self = weak.lock();
            if (!self)
            {
                return;
            }
            if (ec)
            {
                if (retries != 0U)
                {
                    auto timer = std::make_shared<boost::asio::steady_timer>(
                        self->conn->get_io_context());
                    timer->expires_after(std::chrono::seconds(15));
                    timer->async_wait(
                        [timer, weak, retries](boost::system::error_code) {
                            if (auto s = weak.lock())
                            {
                                s->getInitialChassisStatus(retries - 1);
                            }
                        });
                    return;
                }

                // we commonly come up before power control, we'll capture the
                // property change later
                lg2::error(
                    "error getting chassis power status: '{ERROR_MESSAGE}'",
                    "ERROR_MESSAGE", ec.message());
                return;
            }
            self->chassisStatusOn =
                std::get<std::string>(state).ends_with(chassis::sOn);
        },
        std::string(chassis::busname) + std::to_string(slotId),
        std::string(chassis::path) + std::to_string(slotId),
        properties::interface, properties::get, chassis::interface,
        chassis::property);
}

void HostPowerState::handlePowerSignal(sdbusplus::message_t& message)
{
    std::string objectName;
    boost::container::flat_map<std::string, std::variant<std::string>> values;
    message.read(objectName, values);
    auto findState = values.find(power::property);
    if (findState != values.end())
    {
        bool on =
            std::get<std::string>(findState->second).ends_with(".Running");
        if (!on)
        {
            powerTimer.cancel();
            powerStatusOn = false;
            if (hostStatusCallback)
            {
                hostStatusCallback(PowerState::on, powerStatusOn);
            }
            return;
        }
        // on comes too quickly
        powerTimer.expires_after(std::chrono::seconds(10));
        powerTimer.async_wait([weak{weak_from_this()}](
                                  boost::system::error_code ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                return;
            }
            auto self = weak.lock();
            if (!self)
            {
                return;
            }
            if (ec)
            {
                lg2::error("Timer error: '{ERROR_MESSAGE}'", "ERROR_MESSAGE",
                           ec.message());
                return;
            }
            self->powerStatusOn = true;
            if (self->hostStatusCallback)
            {
                self->hostStatusCallback(PowerState::on, self->powerStatusOn);
            }
        });
    }
}

void HostPowerState::handlePostSignal(sdbusplus::message_t& message)
{
    std::string objectName;
    boost::container::flat_map<std::string, std::variant<std::string>> values;
    message.read(objectName, values);
    auto findState = values.find(post::property);
    if (findState != values.end())
    {
        auto& value = std::get<std::string>(findState->second);
        biosHasPost = (value != "Inactive") &&
                      (value != "xyz.openbmc_project.State.OperatingSystem."
                                "Status.OSStatus.Inactive");
        if (hostStatusCallback)
        {
            hostStatusCallback(PowerState::biosPost, biosHasPost);
        }
    }
}

void HostPowerState::handleChassisSignal(sdbusplus::message_t& message)
{
    std::string objectName;
    boost::container::flat_map<std::string, std::variant<std::string>> values;
    message.read(objectName, values);
    auto findState = values.find(chassis::property);
    if (findState != values.end())
    {
        bool on =
            std::get<std::string>(findState->second).ends_with(chassis::sOn);
        if (!on)
        {
            chassisTimer.cancel();
            chassisStatusOn = false;
            if (hostStatusCallback)
            {
                hostStatusCallback(PowerState::chassisOn, chassisStatusOn);
            }
            return;
        }
        // on comes too quickly
        chassisTimer.expires_after(std::chrono::seconds(10));
        chassisTimer.async_wait(
            [weak{weak_from_this()}](boost::system::error_code ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return;
                }
                auto self = weak.lock();
                if (!self)
                {
                    return;
                }
                if (ec)
                {
                    lg2::error("Timer error: '{ERROR_MESSAGE}'",
                               "ERROR_MESSAGE", ec.message());
                    return;
                }
                self->chassisStatusOn = true;
                if (self->hostStatusCallback)
                {
                    self->hostStatusCallback(PowerState::chassisOn,
                                             self->chassisStatusOn);
                }
            });
    }
}

HostPowerState::HostPowerState(
    const std::shared_ptr<sdbusplus::asio::connection>& conn, size_t slotId,
    std::function<void(PowerState, bool)> hostStatusCallback) :
    conn(conn), slotId(slotId),
    hostStatusCallback(std::move(hostStatusCallback)),
    powerTimer(conn->get_io_context()), chassisTimer(conn->get_io_context())
{}

void HostPowerState::setup()
{
    const std::string suffix = std::to_string(slotId);

    powerMatch = std::make_unique<sdbusplus::match>(
        static_cast<sdbusplus::bus_t&>(*conn),
        "type='signal',interface='" + std::string(properties::interface) +
            "',path='" + std::string(power::path) + suffix + "',arg0='" +
            std::string(power::interface) + "'",
        [weak{weak_from_this()}](sdbusplus::message_t& message) {
            if (auto self = weak.lock())
            {
                self->handlePowerSignal(message);
            }
        });

    postMatch = std::make_unique<sdbusplus::match>(
        static_cast<sdbusplus::bus_t&>(*conn),
        "type='signal',interface='" + std::string(properties::interface) +
            "',path='" + std::string(post::path) + suffix + "',arg0='" +
            std::string(post::interface) + "'",
        [weak{weak_from_this()}](sdbusplus::message_t& message) {
            if (auto self = weak.lock())
            {
                self->handlePostSignal(message);
            }
        });

    chassisMatch = std::make_unique<sdbusplus::match>(
        static_cast<sdbusplus::bus_t&>(*conn),
        "type='signal',interface='" + std::string(properties::interface) +
            "',path='" + std::string(chassis::path) + suffix + "',arg0='" +
            std::string(chassis::interface) + "'",
        [weak{weak_from_this()}](sdbusplus::message_t& message) {
            if (auto self = weak.lock())
            {
                self->handleChassisSignal(message);
            }
        });

    // first time do a method call to cache the correct value
    getInitialPowerStatus(2);
    getInitialPostStatus(2);
    getInitialChassisStatus(2);
}

std::shared_ptr<HostPowerState> getOrCreatePowerState(
    const std::shared_ptr<sdbusplus::asio::connection>& conn, size_t slotId,
    const std::function<void(PowerState, bool)>& hostStatusCallback)
{
    auto it = powerStateRegistry.find(slotId);
    if (it != powerStateRegistry.end())
    {
        return it->second;
    }
    auto state =
        std::make_shared<HostPowerState>(conn, slotId, hostStatusCallback);
    state->setup();
    powerStateRegistry[slotId] = state;
    return state;
}

void setupPowerMatchCallback(
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    const std::function<void(PowerState type, bool state, size_t slotId)>&
        callback)
{
    // Ensure the system-wide default host (slotId 0) is tracked. The per-slotId
    // callback is adapted to the single-host HostPowerState callback.
    getOrCreatePowerState(conn, 0, [callback](PowerState type, bool state) {
        if (callback)
        {
            callback(type, state, 0);
        }
    });
}

void setupPowerMatch(const std::shared_ptr<sdbusplus::asio::connection>& conn)
{
    getOrCreatePowerState(conn, 0);
}

// replaces limits if MinReading and MaxReading are found.
void findLimits(std::pair<double, double>& limits,
                const SensorBaseConfiguration* data)
{
    if (data == nullptr)
    {
        return;
    }
    auto maxFind = data->second.find("MaxReading");
    auto minFind = data->second.find("MinReading");

    if (minFind != data->second.end())
    {
        limits.first = std::visit(VariantToDoubleVisitor(), minFind->second);
    }
    if (maxFind != data->second.end())
    {
        limits.second = std::visit(VariantToDoubleVisitor(), maxFind->second);
    }
}

void createAssociation(
    std::shared_ptr<sdbusplus::asio::dbus_interface>& association,
    const std::string& path)
{
    if (association)
    {
        std::filesystem::path p(path);

        std::vector<Association> associations;
        associations.emplace_back("chassis", "all_sensors",
                                  p.parent_path().string());
        association->register_property("Associations", associations);
        association->initialize();
    }
}

void setInventoryAssociation(
    const std::weak_ptr<sdbusplus::asio::dbus_interface>& weakRef,
    const std::string& inventoryPath, const std::string& chassisPath,
    const std::optional<std::string>& monitoredChassisPath = std::nullopt)
{
    auto association = weakRef.lock();
    if (!association)
    {
        return;
    }

    std::vector<Association> associations;
    associations.emplace_back("inventory", "sensors", inventoryPath);
    associations.emplace_back("chassis", "all_sensors", chassisPath);
    if (monitoredChassisPath)
    {
        associations.emplace_back("monitoring", "monitored_by",
                                  *monitoredChassisPath);
    }

    association->register_property("Associations", associations);
    association->initialize();
}

// Finds the chassis inventory object whose Decorator.Slot.SlotNumber matches
// slotId, so a sensor on a shared multi-slot board can publish which chassis it
// monitors. Returns the path via the callback, or std::nullopt if none matches.
static void resolveSlotChassis(
    const std::shared_ptr<sdbusplus::asio::connection>& conn, size_t slotId,
    std::function<void(std::optional<std::string>)>&& callback)
{
    constexpr auto slotChassisInterfaces = std::to_array({
        "xyz.openbmc_project.Inventory.Item.Chassis",
        "xyz.openbmc_project.Inventory.Decorator.Slot",
    });

    auto cb = std::make_shared<std::function<void(std::optional<std::string>)>>(
        std::move(callback));

    conn->async_method_call(
        [conn, slotId, cb](const boost::system::error_code ec,
                           const GetSubTreeType& subtree) {
            if (ec)
            {
                (*cb)(std::nullopt);
                return;
            }

            std::vector<std::pair<std::string, std::string>> candidates;
            for (const auto& [objPath, services] : subtree)
            {
                for (const auto& [service, interfaces] : services)
                {
                    // Only a chassis is a valid monitoring target. Other
                    // objects in the same slot (motherboard, CPU) carry the
                    // same SlotNumber, so require Item.Chassis as well.
                    bool isChassis =
                        std::find(
                            interfaces.begin(), interfaces.end(),
                            "xyz.openbmc_project.Inventory.Item.Chassis") !=
                        interfaces.end();
                    bool hasSlot =
                        std::find(
                            interfaces.begin(), interfaces.end(),
                            "xyz.openbmc_project.Inventory.Decorator.Slot") !=
                        interfaces.end();
                    if (isChassis && hasSlot)
                    {
                        candidates.emplace_back(objPath, service);
                        break;
                    }
                }
            }

            if (candidates.empty())
            {
                (*cb)(std::nullopt);
                return;
            }

            auto remaining = std::make_shared<size_t>(candidates.size());
            auto matched = std::make_shared<bool>(false);
            for (const auto& [objPath, service] : candidates)
            {
                conn->async_method_call(
                    [slotId, cb, remaining, matched,
                     objPath](const boost::system::error_code ec2,
                              const std::variant<uint64_t>& value) {
                        if (!*matched && !ec2)
                        {
                            const auto* number = std::get_if<uint64_t>(&value);
                            if (number != nullptr && *number == slotId)
                            {
                                *matched = true;
                                (*cb)(objPath);
                                return;
                            }
                        }
                        if (--(*remaining) == 0 && !*matched)
                        {
                            (*cb)(std::nullopt);
                        }
                    },
                    service, objPath, "org.freedesktop.DBus.Properties", "Get",
                    "xyz.openbmc_project.Inventory.Decorator.Slot",
                    "SlotNumber");
            }
        },
        mapper::busName, mapper::path, mapper::interface, "GetSubTree",
        inventoryPath, 0, slotChassisInterfaces);
}

std::optional<std::string> findContainingChassis(std::string_view configParent,
                                                 const GetSubTreeType& subtree)
{
    // A parent that is a chassis takes precedence
    for (const auto& [obj, services] : subtree)
    {
        if (obj == configParent)
        {
            return obj;
        }
    }

    // If the parent is not a chassis, the system chassis is used. This does not
    // work if there is more than one System, but we assume there is only one
    // today.
    for (const auto& [obj, services] : subtree)
    {
        for (const auto& [service, interfaces] : services)
        {
            if (std::find(interfaces.begin(), interfaces.end(),
                          "xyz.openbmc_project.Inventory.Item.System") !=
                interfaces.end())
            {
                return obj;
            }
        }
    }
    return std::nullopt;
}

void createInventoryAssoc(
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& association,
    const std::string& path, size_t slotId)
{
    if (!association)
    {
        return;
    }

    constexpr auto allInterfaces = std::to_array({
        "xyz.openbmc_project.Inventory.Item.Board",
        "xyz.openbmc_project.Inventory.Item.Chassis",
    });

    std::weak_ptr<sdbusplus::asio::dbus_interface> weakRef = association;
    conn->async_method_call(
        [conn, weakRef, path, slotId](const boost::system::error_code ec,
                                      const GetSubTreeType& subtree) {
            // The parent of the config is always the inventory object, and may
            // be the associated chassis. If the parent is not itself a chassis
            // or board, the sensor is associated with the system chassis.
            std::string parent =
                std::filesystem::path(path).parent_path().string();
            std::string chassis =
                ec ? parent
                   : findContainingChassis(parent, subtree).value_or(parent);

            // A sensor on a shared multi-slot board additionally publishes a
            // monitoring association to the chassis it serves, resolved from
            // its slot. slotId 0 is the single-host default and needs none.
            if (slotId == 0)
            {
                setInventoryAssociation(weakRef, parent, chassis);
                return;
            }
            resolveSlotChassis(
                conn, slotId,
                [weakRef, parent, chassis](
                    const std::optional<std::string>& monitoredChassisPath) {
                    setInventoryAssociation(weakRef, parent, chassis,
                                            monitoredChassisPath);
                });
        },
        mapper::busName, mapper::path, mapper::interface, "GetSubTree",
        "/xyz/openbmc_project/inventory/system", 2, allInterfaces);
}

std::optional<double> readFile(const std::string& thresholdFile,
                               const double& scaleFactor)
{
    std::string line;
    std::ifstream labelFile(thresholdFile);
    if (labelFile.good())
    {
        std::getline(labelFile, line);
        labelFile.close();

        try
        {
            return std::stod(line) / scaleFactor;
        }
        catch (const std::invalid_argument&)
        {
            return std::nullopt;
        }
    }
    return std::nullopt;
}

std::optional<std::tuple<std::string, std::string, std::string>> splitFileName(
    const std::filesystem::path& filePath)
{
    if (filePath.has_filename())
    {
        const auto fileName = filePath.filename().string();

        size_t numberPos = std::strcspn(fileName.c_str(), "1234567890");
        size_t itemPos = std::strcspn(fileName.c_str(), "_");

        if (numberPos > 0 && itemPos > numberPos && fileName.size() > itemPos)
        {
            return std::make_optional(
                std::make_tuple(fileName.substr(0, numberPos),
                                fileName.substr(numberPos, itemPos - numberPos),
                                fileName.substr(itemPos + 1, fileName.size())));
        }
    }
    return std::nullopt;
}

static void handleSpecialModeChange(const std::string& manufacturingModeStatus)
{
    manufacturingMode = false;
    if (manufacturingModeStatus == "xyz.openbmc_project.Control.Security."
                                   "SpecialMode.Modes.Manufacturing")
    {
        manufacturingMode = true;
    }
    if (validateUnsecureFeature == 1)
    {
        if (manufacturingModeStatus == "xyz.openbmc_project.Control.Security."
                                       "SpecialMode.Modes.ValidationUnsecure")
        {
            manufacturingMode = true;
        }
    }
}

void setupManufacturingModeMatch(sdbusplus::asio::connection& conn)
{
    namespace rules = sdbusplus::match_rules;
    static constexpr const char* specialModeInterface =
        "xyz.openbmc_project.Security.SpecialMode";

    const std::string filterSpecialModeIntfAdd =
        rules::interfacesAdded() +
        rules::argNpath(0, "/xyz/openbmc_project/security/special_mode");
    static std::unique_ptr<sdbusplus::match> specialModeIntfMatch =
        std::make_unique<sdbusplus::match>(
            conn, filterSpecialModeIntfAdd, [](sdbusplus::message_t& m) {
                sdbusplus::object_path path;
                using PropertyMap =
                    boost::container::flat_map<std::string,
                                               std::variant<std::string>>;
                boost::container::flat_map<std::string, PropertyMap>
                    interfaceAdded;
                m.read(path, interfaceAdded);
                auto intfItr = interfaceAdded.find(specialModeInterface);
                if (intfItr == interfaceAdded.end())
                {
                    return;
                }
                PropertyMap& propertyList = intfItr->second;
                auto itr = propertyList.find("SpecialMode");
                if (itr == propertyList.end())
                {
                    lg2::error("error getting SpecialMode property");
                    return;
                }
                auto* manufacturingModeStatus =
                    std::get_if<std::string>(&itr->second);
                handleSpecialModeChange(*manufacturingModeStatus);
            });

    const std::string filterSpecialModeChange =
        rules::type::signal() + rules::member("PropertiesChanged") +
        rules::interface("org.freedesktop.DBus.Properties") +
        rules::argN(0, specialModeInterface);
    static std::unique_ptr<sdbusplus::match> specialModeChangeMatch =
        std::make_unique<sdbusplus::match>(
            conn, filterSpecialModeChange, [](sdbusplus::message_t& m) {
                std::string interfaceName;
                boost::container::flat_map<std::string,
                                           std::variant<std::string>>
                    propertiesChanged;

                m.read(interfaceName, propertiesChanged);
                auto itr = propertiesChanged.find("SpecialMode");
                if (itr == propertiesChanged.end())
                {
                    return;
                }
                auto* manufacturingModeStatus =
                    std::get_if<std::string>(&itr->second);
                handleSpecialModeChange(*manufacturingModeStatus);
            });

    conn.async_method_call(
        [](const boost::system::error_code ec,
           const std::variant<std::string>& getManufactMode) {
            if (ec)
            {
                lg2::error(
                    "error getting SpecialMode status: '{ERROR_MESSAGE}'",
                    "ERROR_MESSAGE", ec.message());
                return;
            }
            const auto* manufacturingModeStatus =
                std::get_if<std::string>(&getManufactMode);
            handleSpecialModeChange(*manufacturingModeStatus);
        },
        "xyz.openbmc_project.SpecialMode",
        "/xyz/openbmc_project/security/special_mode",
        "org.freedesktop.DBus.Properties", "Get", specialModeInterface,
        "SpecialMode");
}

bool getManufacturingMode()
{
    return manufacturingMode;
}

std::vector<std::unique_ptr<sdbusplus::match>> setupPropertiesChangedMatches(
    sdbusplus::asio::connection& bus, std::span<const std::string_view> types,
    const std::function<void(sdbusplus::message_t&)>& handler)
{
    std::vector<std::unique_ptr<sdbusplus::match>> matches;
    for (const std::string_view type : types)
    {
        auto match = std::make_unique<sdbusplus::match>(
            static_cast<sdbusplus::bus_t&>(bus),
            "type='signal',member='PropertiesChanged',path_namespace='" +
                std::string(inventoryPath) + "',arg0namespace='" +
                configInterfaceName(type) + "'",
            handler);
        matches.emplace_back(std::move(match));
    }
    return matches;
}

std::vector<std::unique_ptr<sdbusplus::match>> setupPropertiesChangedMatches(
    sdbusplus::asio::connection& bus,
    std::span<const std::pair<std::string_view, I2CDeviceType>> typeMap,
    const std::function<void(sdbusplus::message_t&)>& handler)
{
    std::vector<std::string_view> types;
    types.reserve(typeMap.size());
    for (const auto& [type, dt] : typeMap)
    {
        types.push_back(type);
    }
    return setupPropertiesChangedMatches(bus, types, handler);
}
