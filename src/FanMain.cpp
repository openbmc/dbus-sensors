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

#include <PwmSensor.hpp>
#include <TachSensor.hpp>
#include <Utils.hpp>
#include <VariantVisitors.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/lexical_cast.hpp>
#include <experimental/filesystem>
#include <fstream>
#include <regex>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

static constexpr bool DEBUG = false;

namespace fs = std::experimental::filesystem;
namespace variant_ns = sdbusplus::message::variant_ns;
static constexpr std::array<const char*, 1> sensorTypes = {
    "xyz.openbmc_project.Configuration.AspeedFan"};
static std::regex inputRegex(R"(fan(\d+)_input)");

void createSensors(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::unique_ptr<TachSensor>>&
        tachSensors,
    boost::container::flat_map<std::string, std::unique_ptr<PwmSensor>>&
        pwmSensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::unique_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged)
{
    bool firstScan = sensorsChanged == nullptr;
    // use new data the first time, then refresh
    ManagedObjectType sensorConfigurations;
    bool useCache = false;
    for (const char* type : sensorTypes)
    {
        if (!getSensorConfiguration(type, dbusConnection, sensorConfigurations,
                                    useCache))
        {
            std::cerr << "error communicating to entity manager\n";
            return;
        }
        useCache = true;
    }
    std::vector<fs::path> paths;
    if (!findFiles(fs::path("/sys/class/hwmon"), R"(fan\d+_input)", paths))
    {
        std::cerr << "No temperature sensors in system\n";
        return;
    }

    // iterate through all found fan sensors, and try to match them with
    // configuration
    for (auto& path : paths)
    {
        std::smatch match;
        std::string pathStr = path.string();

        std::regex_search(pathStr, match, inputRegex);
        std::string indexStr = *(match.begin() + 1);

        auto directory = path.parent_path();
        // convert to 0 based
        size_t index = std::stoul(indexStr) - 1;

        const char* baseType;
        const SensorData* sensorData = nullptr;
        const std::string* interfacePath = nullptr;
        const std::pair<std::string, boost::container::flat_map<
                                         std::string, BasicVariantType>>*
            baseConfiguration = nullptr;
        for (const std::pair<sdbusplus::message::object_path, SensorData>&
                 sensor : sensorConfigurations)
        {
            // find the base of the configuration to see if indexes match
            for (const char* type : sensorTypes)
            {
                auto sensorBaseFind = sensor.second.find(type);
                if (sensorBaseFind != sensor.second.end())
                {
                    baseConfiguration = &(*sensorBaseFind);
                    interfacePath = &(sensor.first.str);
                    baseType = type;
                    break;
                }
            }
            if (baseConfiguration == nullptr)
            {
                continue;
            }
            auto connector =
                sensor.second.find(baseType + std::string(".Connector"));
            if (connector == sensor.second.end())
            {
                std::cerr << baseConfiguration->first << " missing connector\n";
                continue;
            }
            auto findPwmIndex = connector->second.find("Pwm");
            if (findPwmIndex == connector->second.end())
            {
                continue;
            }
            uint16_t pwmIndex = variant_ns::visit(VariantToUnsignedIntVisitor(),
                                                  findPwmIndex->second);
            auto oemNamePath = directory.string() + R"(/of_node/oemname)" +
                               std::to_string(pwmIndex);

            if (DEBUG)
            {
                std::cout << "Checking path " << oemNamePath << "\n";
            }
            std::ifstream nameFile(oemNamePath);
            if (!nameFile.good())
            {
                continue;
            }
            std::string oemName;
            std::getline(nameFile, oemName);
            nameFile.close();
            if (!oemName.size())
            {
                // shouldn't have an empty name file
                continue;
            }
            oemName.pop_back(); // remove trailing null
            auto findIndex = baseConfiguration->second.find("Index");
            if (findIndex == baseConfiguration->second.end())
            {
                std::cerr << baseConfiguration->first << " missing index\n";
                continue;
            }
            unsigned int configIndex = variant_ns::visit(
                VariantToUnsignedIntVisitor(), findIndex->second);

            if (configIndex != index)
            {
                continue;
            }
            // now that the indexes match, verify the connector
            auto findConnectorName = connector->second.find("Name");
            if (findConnectorName == connector->second.end())
            {
                continue;
            }
            std::string connectorName = variant_ns::visit(
                VariantToStringVisitor(), findConnectorName->second);
            boost::replace_all(connectorName, " ", "_");
            if (connectorName == oemName)
            {
                sensorData = &(sensor.second);
                break;
            }
        }
        if (sensorData == nullptr)
        {
            std::cerr << "failed to find match for " << path.string() << "\n";
            continue;
        }

        auto findSensorName = baseConfiguration->second.find("Name");
        if (findSensorName == baseConfiguration->second.end())
        {
            std::cerr << "could not determine configuration name for "
                      << path.string() << "\n";
            continue;
        }
        std::string sensorName =
            sdbusplus::message::variant_ns::get<std::string>(
                findSensorName->second);
        // on rescans, only update sensors we were signaled by
        auto findSensor = tachSensors.find(sensorName);
        if (!firstScan && findSensor != tachSensors.end())
        {
            bool found = false;
            for (auto it = sensorsChanged->begin(); it != sensorsChanged->end();
                 it++)
            {
                if (boost::ends_with(*it, findSensor->second->name))
                {
                    sensorsChanged->erase(it);
                    findSensor->second = nullptr;
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                continue;
            }
        }
        std::vector<thresholds::Threshold> sensorThresholds;
        if (!parseThresholdsFromConfig(*sensorData, sensorThresholds))
        {
            std::cerr << "error populating thresholds for " << sensorName
                      << "\n";
        }

        tachSensors[sensorName] = std::make_unique<TachSensor>(
            path.string(), objectServer, dbusConnection, io, sensorName,
            std::move(sensorThresholds), *interfacePath);
    }
    std::vector<fs::path> pwms;
    if (!findFiles(fs::path("/sys/class/hwmon"), R"(pwm\d+)", pwms))
    {
        std::cerr << "No pwm in system\n";
        return;
    }
    for (const fs::path& pwm : pwms)
    {
        // only add new elements
        pwmSensors.insert(std::pair<std::string, std::unique_ptr<PwmSensor>>(
            pwm.string(),
            std::make_unique<PwmSensor>(pwm.string(), objectServer)));
    }
}

int main(int argc, char** argv)
{
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.FanSensor");
    sdbusplus::asio::object_server objectServer(systemBus);
    boost::container::flat_map<std::string, std::unique_ptr<TachSensor>>
        tachSensors;
    boost::container::flat_map<std::string, std::unique_ptr<PwmSensor>>
        pwmSensors;
    std::vector<std::unique_ptr<sdbusplus::bus::match::match>> matches;
    std::unique_ptr<boost::container::flat_set<std::string>> sensorsChanged =
        std::make_unique<boost::container::flat_set<std::string>>();

    io.post([&]() {
        createSensors(io, objectServer, tachSensors, pwmSensors, systemBus,
                      nullptr);
    });

    boost::asio::deadline_timer filterTimer(io);
    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message& message) {
            if (message.is_method_error())
            {
                std::cerr << "callback method error\n";
                return;
            }
            sensorsChanged->insert(message.get_path());
            // this implicitly cancels the timer
            filterTimer.expires_from_now(boost::posix_time::seconds(1));

            filterTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    /* we were canceled*/
                    return;
                }
                else if (ec)
                {
                    std::cerr << "timer error\n";
                    return;
                }
                createSensors(io, objectServer, tachSensors, pwmSensors,
                              systemBus, sensorsChanged);
            });
        };

    for (const char* type : sensorTypes)
    {
        auto match = std::make_unique<sdbusplus::bus::match::match>(
            static_cast<sdbusplus::bus::bus&>(*systemBus),
            "type='signal',member='PropertiesChanged',path_namespace='" +
                std::string(inventoryPath) + "',arg0namespace='" + type + "'",
            eventHandler);
        matches.emplace_back(std::move(match));
    }

    io.run();
}
