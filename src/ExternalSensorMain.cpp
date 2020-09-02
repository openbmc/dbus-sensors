#include "ExternalSensor.hpp"
#include "Utils.hpp"

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

static constexpr bool DEBUG = false;

static constexpr std::array<const char*, 1> sensorTypes = {
    "xyz.openbmc_project.Configuration.ExternalSensor",
};

// Copied from HwmonTempSensor and inspired by
// https://gerrit.openbmc-project.xyz/c/openbmc/dbus-sensors/+/35476

// The ExternalSensor is a sensor intended to be settable by something
// external to the BMC, so that the host (or something else external)
// can write to it, perhaps by using an IPMI connection.

// Unlike the other sensors, an external sensor does not correspond
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
                const std::string* interfacePath = &(sensor.first.str);
                const SensorData* sensorData = &(sensor.second);

                const char* sensorType = nullptr;
                const SensorBaseConfiguration* baseConfiguration = nullptr;
                const SensorBaseConfigMap* baseConfigMap = nullptr;

                for (const char* type : sensorTypes)
                {
                    auto sensorBase = sensorData->find(type);
                    if (sensorBase != sensorData->end())
                    {
                        sensorType = type;
                        baseConfiguration = &(*sensorBase);
                        break;
                    }
                }
                if (baseConfiguration == nullptr)
                {
                    std::cerr << "error finding base configuration for "
                              << *interfacePath << "\n";
                    continue;
                }

                baseConfigMap = &(baseConfiguration->second);
                auto configurationMin = baseConfigMap->find("MinValue");
                auto configurationMax = baseConfigMap->find("MaxValue");

                // Range of signed byte seems reasonable default
                double minValue = std::numeric_limits<int8_t>::min();
                double maxValue = std::numeric_limits<int8_t>::max();

                if (configurationMin != baseConfigMap->end())
                {
                    minValue = std::get<double>(configurationMin->second);
                }
                if (configurationMax != baseConfigMap->end())
                {
                    maxValue = std::get<double>(configurationMax->second);
                }

                auto findSensorName = baseConfigMap->find("Name");
                if (findSensorName == baseConfigMap->end())
                {
                    std::cerr << "could not determine configuration name for "
                              << *interfacePath << "\n";
                    continue;
                }

                std::string sensorName =
                    std::get<std::string>(findSensorName->second);

                auto findMeasure = baseConfigMap->find("Measure");
                std::string sensorMeasure;
                if (findMeasure != baseConfigMap->end())
                {
                    sensorMeasure = std::get<std::string>(findMeasure->second);
                }
                else
                {
                    std::cerr << "External sensor " << *interfacePath
                              << " is missing the Measure parameter\n";
                    continue;
                }

                // on rescans, only update sensors we were signaled by
                auto findSensor = sensors.find(sensorName);
                if (!firstScan && (findSensor != sensors.end()))
                {
                    bool found = false;
                    for (auto it = sensorsChanged->begin();
                         it != sensorsChanged->end(); it++)
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
                    std::cerr << "error populating thresholds for "
                              << sensorName << "\n";
                }

                auto findPowerOn = baseConfiguration->second.find("PowerState");
                PowerState readState = PowerState::always;
                if (findPowerOn != baseConfiguration->second.end())
                {
                    std::string powerState = std::visit(
                        VariantToStringVisitor(), findPowerOn->second);
                    setReadState(powerState, readState);
                }

                auto& sensorEntry = sensors[sensorName];
                sensorEntry = nullptr;

                sensorEntry = std::make_shared<ExternalSensor>(
                    sensorType, objectServer, dbusConnection, sensorName,
                    sensorMeasure, std::move(sensorThresholds), *interfacePath,
                    maxValue, minValue, readState);
            }
        });

    getter->getConfiguration(
        std::vector<std::string>(sensorTypes.begin(), sensorTypes.end()));
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

    io.post([&]() {
        createSensors(io, objectServer, sensors, systemBus, nullptr);
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
