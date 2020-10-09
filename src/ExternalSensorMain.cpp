#include "ExternalSensor.hpp"
#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

#include <array>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

// Copied from HwmonTempSensor and inspired by
// https://gerrit.openbmc-project.xyz/c/openbmc/dbus-sensors/+/35476

// The ExternalSensor is a sensor whose value is intended to be writable
// by something external to the BMC, so that the host (or something else)
// can write to it, perhaps by using an IPMI connection.

// Unlike most other sensors, an external sensor does not correspond
// to a hwmon file or other kernel/hardware interface,
// so, after initialization, this module does not have much to do,
// but it handles reinitialization and thresholds, similar to the others.

// As there is no corresponding driver or hardware to support,
// all configuration of this sensor comes from the JSON parameters:
// MinValue, MaxValue, PowerState, Measure, Name

// The purpose of "Measure" is to specify the physical characteristic
// the external sensor is measuring, because with an external sensor
// there is no other way to tell, and it will be used for the object path
// here: /xyz/openbmc_project/sensors/<Measure>/<Name>

static constexpr bool debug = false;

static const char* sensorType =
    "xyz.openbmc_project.Configuration.ExternalSensor";

void createSensors(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<ExternalSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged)
{
    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection,
        [&io, &objectServer, &sensors, &dbusConnection,
         sensorsChanged](const ManagedObjectType& sensorConfigurations) {
            bool firstScan = (sensorsChanged == nullptr);

            for (const std::pair<sdbusplus::message::object_path, SensorData>&
                     sensor : sensorConfigurations)
            {
                const std::string& interfacePath = sensor.first.str;
                const SensorData& sensorData = sensor.second;

                auto sensorBase = sensorData.find(sensorType);
                if (sensorBase == sensorData.end())
                {
                    std::cerr << "Base configuration not found for "
                              << interfacePath << "\n";
                    continue;
                }

                const SensorBaseConfiguration& baseConfiguration = *sensorBase;
                const SensorBaseConfigMap& baseConfigMap =
                    baseConfiguration.second;

                double minValue;
                double maxValue;

                // MinValue and MinValue are mandatory numeric parameters
                auto minFound = baseConfigMap.find("MinValue");
                if (minFound == baseConfigMap.end())
                {
                    std::cerr << "MinValue parameter not found for "
                              << interfacePath << "\n";
                    continue;
                }
                minValue =
                    std::visit(VariantToDoubleVisitor(), minFound->second);
                if (!std::isfinite(minValue))
                {
                    std::cerr << "MinValue parameter not parsed for "
                              << interfacePath << "\n";
                    continue;
                }

                auto maxFound = baseConfigMap.find("MaxValue");
                if (maxFound == baseConfigMap.end())
                {
                    std::cerr << "MaxValue parameter not found for "
                              << interfacePath << "\n";
                    continue;
                }
                maxValue =
                    std::visit(VariantToDoubleVisitor(), maxFound->second);
                if (!std::isfinite(maxValue))
                {
                    std::cerr << "MaxValue parameter not parsed for "
                              << interfacePath << "\n";
                    continue;
                }

                std::string sensorName;
                std::string sensorMeasure;

                // Name and Measure are mandatory string parameters
                auto nameFound = baseConfigMap.find("Name");
                if (nameFound == baseConfigMap.end())
                {
                    std::cerr << "Name parameter not found for "
                              << interfacePath << "\n";
                    continue;
                }
                sensorName =
                    std::visit(VariantToStringVisitor(), nameFound->second);
                if (sensorName.empty())
                {
                    std::cerr << "Name parameter not parsed for "
                              << interfacePath << "\n";
                    continue;
                }

                auto measureFound = baseConfigMap.find("Units");
                if (measureFound == baseConfigMap.end())
                {
                    std::cerr << "Units parameter not found for "
                              << interfacePath << "\n";
                    continue;
                }
                sensorMeasure =
                    std::visit(VariantToStringVisitor(), measureFound->second);
                if (sensorMeasure.empty())
                {
                    std::cerr << "Measure parameter not parsed for "
                              << interfacePath << "\n";
                    continue;
                }

                // on rescans, only update sensors we were signaled by
                auto findSensor = sensors.find(sensorName);
                if (!firstScan && (findSensor != sensors.end()))
                {
                    std::string suffixName = "/";
                    suffixName += findSensor->second->name;
                    bool found = false;
                    for (auto it = sensorsChanged->begin();
                         it != sensorsChanged->end(); it++)
                    {
                        std::string suffixIt = "/";
                        suffixIt += *it;
                        if (boost::ends_with(suffixIt, suffixName))
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
                if (!parseThresholdsFromConfig(sensorData, sensorThresholds))
                {
                    std::cerr << "error populating thresholds for "
                              << sensorName << "\n";
                }

                auto findPowerOn = baseConfiguration.second.find("PowerState");
                PowerState readState = PowerState::always;
                if (findPowerOn != baseConfiguration.second.end())
                {
                    std::string powerState = std::visit(
                        VariantToStringVisitor(), findPowerOn->second);
                    setReadState(powerState, readState);
                }

                auto& sensorEntry = sensors[sensorName];
                sensorEntry = nullptr;

                sensorEntry = std::make_shared<ExternalSensor>(
                    sensorType, objectServer, dbusConnection, sensorName,
                    sensorMeasure, std::move(sensorThresholds), interfacePath,
                    maxValue, minValue, readState);
            }
        });

    getter->getConfiguration(std::vector<std::string>{sensorType});
}

int main()
{
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.ExternalSensor");
    sdbusplus::asio::object_server objectServer(systemBus);
    boost::container::flat_map<std::string, std::shared_ptr<ExternalSensor>>
        sensors;
    std::vector<std::unique_ptr<sdbusplus::bus::match::match>> matches;
    auto sensorsChanged =
        std::make_shared<boost::container::flat_set<std::string>>();

    io.post([&io, &objectServer, &sensors, &systemBus]() {
        createSensors(io, objectServer, sensors, systemBus, nullptr);
    });

    boost::asio::deadline_timer filterTimer(io);
    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&io, &objectServer, &sensors, &systemBus, &sensorsChanged,
         &filterTimer](sdbusplus::message::message& message) {
            if (message.is_method_error())
            {
                std::cerr << "callback method error\n";
                return;
            }
            sensorsChanged->insert(message.get_path());
            // this implicitly cancels the timer
            filterTimer.expires_from_now(boost::posix_time::seconds(1));

            filterTimer.async_wait([&io, &objectServer, &sensors, &systemBus,
                                    &sensorsChanged](
                                       const boost::system::error_code& ec) {
                if (ec)
                {
                    if (ec != boost::asio::error::operation_aborted)
                    {
                        std::cerr << "callback error: " << ec.message() << "\n";
                    }
                    return;
                }
                createSensors(io, objectServer, sensors, systemBus,
                              sensorsChanged);
            });
        };

    auto match = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" + sensorType + "'",
        eventHandler);
    matches.emplace_back(std::move(match));

    io.run();
}
