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

static bool configToString(const SensorBaseConfigMap* config,
                           const std::string& name, bool missingOk,
                           std::string& value)
{
    auto findName = config->find(name);
    if (findName != config->end())
    {
        auto valuePtr = std::get_if<std::string>(&(findName->second));
        if (valuePtr)
        {
            value = *valuePtr;
            if (value.empty())
            {
                // Reject value of empty string
                return false;
            }
            return true;
        }
    }
    else
    {
        if (missingOk)
        {
            // Leave value unchanged
            return true;
        }
    }
    return false;
}

static bool configToNumber(const SensorBaseConfigMap* config,
                           const std::string& name, bool missingOk,
                           double& value)
{
    auto findName = config->find(name);
    if (findName != config->end())
    {
        // Prefer double type, parse it first
        auto doublePtr = std::get_if<double>(&(findName->second));
        if (doublePtr)
        {
            value = *doublePtr;
            if (!std::isfinite(value))
            {
                // Reject NaN and other weird floating-point
                return false;
            }
            return true;
        }

        // Accept various other numeric types, standardize to double
        auto intPtr = std::get_if<int64_t>(&(findName->second));
        if (intPtr)
        {
            value = static_cast<double>(*intPtr);
            return true;
        }

        auto uintPtr = std::get_if<uint64_t>(&(findName->second));
        if (uintPtr)
        {
            value = static_cast<double>(*uintPtr);
            return true;
        }
    }
    else
    {
        if (missingOk)
        {
            // Leave value unchanged
            return true;
        }
    }
    return false;
}

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

                // Range of signed byte seems reasonable default
                double minValue = std::numeric_limits<int8_t>::min();
                double maxValue = std::numeric_limits<int8_t>::max();

                // MinValue and MinValue are optional numeric parameters
                // It is not an error for them to not exist
                // It is an error for them to exist and fail to parse
                if (!configToNumber(baseConfigMap, "MinValue", true, minValue))
                {
                    std::cerr << "unable to read MinValue parameter for "
                              << *interfacePath << "\n";
                    continue;
                }
                if (!configToNumber(baseConfigMap, "MaxValue", true, maxValue))
                {
                    std::cerr << "unable to read MaxValue parameter for "
                              << *interfacePath << "\n";
                    continue;
                }

                std::string sensorName;
                std::string sensorMeasure;

                // Name and Measure are mandatory string parameters
                if (!configToString(baseConfigMap, "Name", false, sensorName))
                {
                    std::cerr << "unable to read Name parameter for "
                              << *interfacePath << "\n";
                    continue;
                }
                if (!configToString(baseConfigMap, "Measure", false,
                                    sensorMeasure))
                {
                    std::cerr << "unable to read Measure parameter for "
                              << *interfacePath << "\n";
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
