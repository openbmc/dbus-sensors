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

#include <Utils.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

namespace fs = std::filesystem;
const static constexpr char* powerInterfaceName =
    "xyz.openbmc_project.Chassis.Control.Power";
const static constexpr char* powerObjectName =
    "/xyz/openbmc_project/Chassis/Control/Power0";

static bool powerStatusOn = false;
static bool biosHasPost = false;

static std::unique_ptr<sdbusplus::bus::match::match> powerMatch = nullptr;

bool getSensorConfiguration(
    const std::string& type,
    const std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    ManagedObjectType& resp, bool useCache)
{
    static ManagedObjectType managedObj;

    if (!useCache)
    {
        managedObj.clear();
        sdbusplus::message::message getManagedObjects =
            dbusConnection->new_method_call(
                entityManagerName, "/", "org.freedesktop.DBus.ObjectManager",
                "GetManagedObjects");
        bool err = false;
        try
        {
            sdbusplus::message::message reply =
                dbusConnection->call(getManagedObjects);
            reply.read(managedObj);
        }
        catch (const sdbusplus::exception::exception&)
        {
            err = true;
        }

        if (err)
        {
            std::cerr << "Error communicating to entity manager\n";
            return false;
        }
    }
    for (const auto& pathPair : managedObj)
    {
        std::vector<boost::container::flat_map<std::string, BasicVariantType>>
            sensorData;
        bool correctType = false;
        for (const auto& entry : pathPair.second)
        {
            if (boost::starts_with(entry.first, type))
            {
                correctType = true;
                break;
            }
        }
        if (correctType)
        {
            resp.emplace(pathPair);
        }
    }
    return true;
}

bool findFiles(const fs::path dirPath, const std::string& matchString,
               std::vector<fs::path>& foundPaths, unsigned int symlinkDepth)
{
    if (!fs::exists(dirPath))
        return false;

    std::regex search(matchString);
    std::smatch match;
    for (auto& p : fs::recursive_directory_iterator(dirPath))
    {
        std::string path = p.path().string();
        if (!is_directory(p))
        {
            if (std::regex_search(path, match, search))
                foundPaths.emplace_back(p.path());
        }
        else if (is_symlink(p) && symlinkDepth)
        {
            findFiles(p.path(), matchString, foundPaths, symlinkDepth - 1);
        }
    }
    return true;
}

bool isPowerOn(void)
{
    if (!powerMatch)
    {
        throw std::runtime_error("Power Match Not Created");
    }
    return powerStatusOn;
}

bool hasBiosPost(void)
{
    if (!powerMatch)
    {
        throw std::runtime_error("Power Match Not Created");
    }
    return biosHasPost;
}

void setupPowerMatch(const std::shared_ptr<sdbusplus::asio::connection>& conn)
{
    // create a match for powergood changes, first time do a method call to
    // cache the correct value
    std::function<void(sdbusplus::message::message & message)> eventHandler =
        [](sdbusplus::message::message& message) {
            std::string objectName;
            boost::container::flat_map<std::string, std::variant<int32_t, bool>>
                values;
            message.read(objectName, values);
            auto findPgood = values.find("pgood");
            if (findPgood != values.end())
            {
                powerStatusOn = std::get<int32_t>(findPgood->second);
            }
            auto findPostComplete = values.find("post_complete");
            if (findPostComplete != values.end())
            {
                biosHasPost = std::get<bool>(findPostComplete->second);
            }
        };

    powerMatch = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*conn),
        "type='signal',interface='org.freedesktop.DBus.Properties',path_"
        "namespace='/xyz/openbmc_project/Chassis/Control/"
        "Power0',arg0='xyz.openbmc_project.Chassis.Control.Power'",
        eventHandler);

    conn->async_method_call(
        [](boost::system::error_code ec, const std::variant<int32_t>& pgood) {
            if (ec)
            {
                // we commonly come up before power control, we'll capture the
                // property change later
                return;
            }
            powerStatusOn = std::get<int32_t>(pgood);
        },
        powerInterfaceName, powerObjectName, "org.freedesktop.DBus.Properties",
        "Get", powerInterfaceName, "pgood");

    conn->async_method_call(
        [](boost::system::error_code ec,
           const std::variant<int32_t>& postComplete) {
            if (ec)
            {
                // we commonly come up before power control, we'll capture the
                // property change later
                return;
            }
            biosHasPost = std::get<int32_t>(postComplete);
        },
        powerInterfaceName, powerObjectName, "org.freedesktop.DBus.Properties",
        "Get", powerInterfaceName, "post_complete");
}

// replaces limits if MinReading and MaxReading are found.
void findLimits(std::pair<double, double>& limits,
                const SensorBaseConfiguration* data)
{
    if (!data)
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

        using Association = std::tuple<std::string, std::string, std::string>;
        std::vector<Association> associations;
        associations.push_back(
            Association("inventory", "sensors", p.parent_path().string()));
        association->register_property("associations", associations);
        association->initialize();
    }
}