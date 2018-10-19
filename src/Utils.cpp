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
#include <experimental/filesystem>
#include <fstream>
#include <regex>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/bus/match.hpp>

namespace fs = std::experimental::filesystem;
const static constexpr char* POWER_INTERFACE_NAME =
    "xyz.openbmc_project.Chassis.Control.Power";
const static constexpr char* POWER_OBJECT_NAME =
    "/xyz/openbmc_project/Chassis/Control/Power0";

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
                ENTITY_MANAGER_NAME, "/", "org.freedesktop.DBus.ObjectManager",
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

bool find_files(const fs::path dir_path, const std::string& match_string,
                std::vector<fs::path>& found_paths, unsigned int symlink_depth)
{
    if (!fs::exists(dir_path))
        return false;

    fs::directory_iterator end_itr;
    std::regex search(match_string);
    std::smatch match;
    for (auto& p : fs::recursive_directory_iterator(dir_path))
    {
        std::string path = p.path().string();
        if (!is_directory(p))
        {
            if (std::regex_search(path, match, search))
                found_paths.emplace_back(p.path());
        }
        else if (is_symlink(p) && symlink_depth)
        {
            find_files(p.path(), match_string, found_paths, symlink_depth - 1);
        }
    }
    return true;
}

// initially returns false, then sets up matches and returns status
// should be called once first to initialize
bool isPowerOn(const std::shared_ptr<sdbusplus::asio::connection>& conn)
{
    static std::unique_ptr<sdbusplus::bus::match::match> powerMatch = nullptr;
    static bool powerStatusOn = false;

    if (powerMatch != nullptr)
    {
        return powerStatusOn;
    }

    // create a match for powergood changes, first time do a method call to
    // return the correct value
    std::function<void(sdbusplus::message::message & message)> eventHandler =
        [&powerStatusOn](sdbusplus::message::message& message) {
            std::string objectName;
            boost::container::flat_map<std::string,
                                       sdbusplus::message::variant<int32_t>>
                values;
            message.read(objectName, values);
            auto findPgood = values.find("pgood");
            if (findPgood != values.end())
            {
                powerStatusOn = sdbusplus::message::variant_ns::get<int32_t>(
                    findPgood->second);
            }
        };

    powerMatch = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*conn),
        "type='signal',interface='org.freedesktop.DBus.Properties',path_"
        "namespace='/xyz/openbmc_project/Chassis/Control/"
        "power0',arg0='xyz.openbmc_project.Chassis.Control.Power'",
        eventHandler);

    conn->async_method_call(
        [&powerStatusOn](boost::system::error_code ec,
                         const sdbusplus::message::variant<int32_t>& pgood) {
            if (ec)
            {
                std::cerr << "Error getting initial power status\n";
                return;
            }
            powerStatusOn = sdbusplus::message::variant_ns::get<int32_t>(pgood);
        },
        POWER_INTERFACE_NAME, POWER_OBJECT_NAME,
        "org.freedesktop.DBus.Properties", "Get", "pgood");

    return powerStatusOn;
}