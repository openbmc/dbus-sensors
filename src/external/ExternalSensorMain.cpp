#include "ExternalSensor.hpp"
#include "asio/Thresholds.hpp"
#include "utils/Utils.hpp"
#include "utils/VariantVisitors.hpp"

#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

// Copied from HwmonTempSensor and inspired by
// https://gerrit.openbmc-project.xyz/c/openbmc/dbus-sensors/+/35476

// The ExternalSensor is a sensor whose value is intended to be writable
// by something external to the BMC, so that the host (or something else)
// can write to it, perhaps by using an IPMI or Redfish connection.

// Unlike most other sensors, an external sensor does not correspond
// to a hwmon file or any other kernel/hardware interface,
// so, after initialization, this module does not have much to do,
// but it handles reinitialization and thresholds, similar to the others.
// The main work of this module is to provide backing storage for a
// sensor that exists only virtually, and to provide an optional
// timeout service for detecting loss of timely updates.

// As there is no corresponding driver or hardware to support,
// all configuration of this sensor comes from the JSON parameters:
// MinValue, MaxValue, Timeout, PowerState, Units, Name

// The purpose of "Units" is to specify the physical characteristic
// the external sensor is measuring, because with an external sensor
// there is no other way to tell, and it will be used for the object path
// here: /xyz/openbmc_project/sensors/<Units>/<Name>

// For more information, see external-sensor.md design document:
// https://gerrit.openbmc-project.xyz/c/openbmc/docs/+/41452
// https://github.com/openbmc/docs/tree/master/designs/

static const char* sensorType = "ExternalSensor";

void updateReaper(
    boost::container::flat_map<std::string, std::shared_ptr<ExternalSensor>>&
        sensors,
    boost::asio::steady_timer& timer,
    const std::chrono::steady_clock::time_point& now)
{
    // First pass, reap all stale sensors
    for (const auto& [name, sensor] : sensors)
    {
        if (!sensor)
        {
            continue;
        }

        if (!sensor->isAliveAndPerishable())
        {
            continue;
        }

        if (!sensor->isAliveAndFresh(now))
        {
            // Mark sensor as dead, no longer alive
            sensor->writeInvalidate();
        }
    }

    std::chrono::steady_clock::duration nextCheck;
    bool needCheck = false;

    // Second pass, determine timer interval to next check
    for (const auto& [name, sensor] : sensors)
    {
        if (!sensor)
        {
            continue;
        }

        if (!sensor->isAliveAndPerishable())
        {
            continue;
        }

        auto expiration = sensor->ageRemaining(now);

        if (needCheck)
        {
            nextCheck = std::min(nextCheck, expiration);
        }
        else
        {
            // Initialization
            nextCheck = expiration;
            needCheck = true;
        }
    }

    if (!needCheck)
    {
        lg2::debug("Next ExternalSensor timer idle");

        return;
    }

    timer.expires_at(now + nextCheck);

    timer.async_wait([&sensors, &timer](const boost::system::error_code& err) {
        if (err != boost::system::errc::success)
        {
            // Cancellation is normal, as timer is dynamically rescheduled
            if (err != boost::asio::error::operation_aborted)
            {
                lg2::error(
                    "ExternalSensor timer scheduling problem: {ERROR_MESSAGE}",
                    "ERROR_MESSAGE", err.message());
            }
            return;
        }

        updateReaper(sensors, timer, std::chrono::steady_clock::now());
    });

    lg2::debug("Next ExternalSensor timer '{VALUE}' us", "VALUE",
               std::chrono::duration_cast<std::chrono::microseconds>(nextCheck)
                   .count());
}

void createSensors(
    sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<ExternalSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged,
    boost::asio::steady_timer& reaperTimer)
{
    lg2::debug("ExternalSensor considering creating sensors");

    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection,
        [&objectServer, &sensors, &dbusConnection, sensorsChanged,
         &reaperTimer](const ManagedObjectType& sensorConfigurations) {
            bool firstScan = (sensorsChanged == nullptr);

            for (const std::pair<sdbusplus::message::object_path, SensorData>&
                     sensor : sensorConfigurations)
            {
                const std::string& interfacePath = sensor.first.str;
                const SensorData& sensorData = sensor.second;

                auto sensorBase =
                    sensorData.find(configInterfaceName(sensorType));
                if (sensorBase == sensorData.end())
                {
                    lg2::error("Base configuration not found for '{PATH}'",
                               "PATH", interfacePath);
                    continue;
                }

                const SensorBaseConfiguration& baseConfiguration = *sensorBase;
                const SensorBaseConfigMap& baseConfigMap =
                    baseConfiguration.second;

                // MinValue and MinValue are mandatory numeric parameters
                auto minFound = baseConfigMap.find("MinValue");
                if (minFound == baseConfigMap.end())
                {
                    lg2::error("MinValue parameter not found for '{PATH}'",
                               "PATH", interfacePath);
                    continue;
                }
                double minValue =
                    std::visit(VariantToDoubleVisitor(), minFound->second);
                if (!std::isfinite(minValue))
                {
                    lg2::error("MinValue parameter not parsed for '{PATH}'",
                               "PATH", interfacePath);
                    continue;
                }

                auto maxFound = baseConfigMap.find("MaxValue");
                if (maxFound == baseConfigMap.end())
                {
                    lg2::error("MaxValue parameter not found for '{PATH}'",
                               "PATH", interfacePath);
                    continue;
                }
                double maxValue =
                    std::visit(VariantToDoubleVisitor(), maxFound->second);
                if (!std::isfinite(maxValue))
                {
                    lg2::error("MaxValue parameter not parsed for '{PATH}'",
                               "PATH", interfacePath);
                    continue;
                }

                double timeoutSecs = 0.0;

                // Timeout is an optional numeric parameter
                auto timeoutFound = baseConfigMap.find("Timeout");
                if (timeoutFound != baseConfigMap.end())
                {
                    timeoutSecs = std::visit(VariantToDoubleVisitor(),
                                             timeoutFound->second);
                }
                if (!std::isfinite(timeoutSecs) || (timeoutSecs < 0.0))
                {
                    lg2::error("Timeout parameter not parsed for '{PATH}'",
                               "PATH", interfacePath);
                    continue;
                }

                std::string sensorName;
                std::string sensorUnits;

                // Name and Units are mandatory string parameters
                auto nameFound = baseConfigMap.find("Name");
                if (nameFound == baseConfigMap.end())
                {
                    lg2::error("Name parameter not found for '{PATH}'", "PATH",
                               interfacePath);
                    continue;
                }
                sensorName =
                    std::visit(VariantToStringVisitor(), nameFound->second);
                if (sensorName.empty())
                {
                    lg2::error("Name parameter not parsed for '{PATH}'", "PATH",
                               interfacePath);
                    continue;
                }

                auto unitsFound = baseConfigMap.find("Units");
                if (unitsFound == baseConfigMap.end())
                {
                    lg2::error("Units parameter not found for '{PATH}'", "PATH",
                               interfacePath);
                    continue;
                }
                sensorUnits =
                    std::visit(VariantToStringVisitor(), unitsFound->second);
                if (sensorUnits.empty())
                {
                    lg2::error("Units parameter not parsed for '{PATH}'",
                               "PATH", interfacePath);
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
                        if (suffixIt.ends_with(suffixName))
                        {
                            sensorsChanged->erase(it);
                            findSensor->second = nullptr;
                            found = true;
                            lg2::debug("ExternalSensor '{NAME}' change found",
                                       "NAME", sensorName);
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
                    lg2::error("error populating thresholds for '{NAME}'",
                               "NAME", sensorName);
                }

                PowerState readState = getPowerState(baseConfigMap);

                auto& sensorEntry = sensors[sensorName];
                sensorEntry = nullptr;

                sensorEntry = std::make_shared<ExternalSensor>(
                    sensorType, objectServer, dbusConnection, sensorName,
                    sensorUnits, std::move(sensorThresholds), interfacePath,
                    maxValue, minValue, timeoutSecs, readState);
                sensorEntry->initWriteHook(
                    [&sensors, &reaperTimer](
                        const std::chrono::steady_clock::time_point& now) {
                        updateReaper(sensors, reaperTimer, now);
                    });

                lg2::debug("ExternalSensor '{NAME}' created", "NAME",
                           sensorName);
            }
        });

    getter->getConfiguration(std::vector<std::string>{sensorType});
}

int main()
{
    lg2::debug("ExternalSensor service starting up");

    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus, true);

    objectServer.add_manager("/xyz/openbmc_project/sensors");
    systemBus->request_name("xyz.openbmc_project.ExternalSensor");

    boost::container::flat_map<std::string, std::shared_ptr<ExternalSensor>>
        sensors;
    auto sensorsChanged =
        std::make_shared<boost::container::flat_set<std::string>>();
    boost::asio::steady_timer reaperTimer(io);

    boost::asio::post(io, [&objectServer, &sensors, &systemBus,
                           &reaperTimer]() {
        createSensors(objectServer, sensors, systemBus, nullptr, reaperTimer);
    });

    boost::asio::steady_timer filterTimer(io);
    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&objectServer, &sensors, &systemBus, &sensorsChanged, &filterTimer,
         &reaperTimer](sdbusplus::message_t& message) mutable {
            if (message.is_method_error())
            {
                lg2::error("callback method error");
                return;
            }

            const auto* messagePath = message.get_path();
            sensorsChanged->insert(messagePath);
            lg2::debug("ExternalSensor change event received: '{PATH}'", "PATH",
                       messagePath);

            // this implicitly cancels the timer
            filterTimer.expires_after(std::chrono::seconds(1));

            filterTimer.async_wait(
                [&objectServer, &sensors, &systemBus, &sensorsChanged,
                 &reaperTimer](const boost::system::error_code& ec) mutable {
                    if (ec != boost::system::errc::success)
                    {
                        if (ec != boost::asio::error::operation_aborted)
                        {
                            lg2::error("callback error: '{ERROR_MESSAGE}'",
                                       "ERROR_MESSAGE", ec.message());
                        }
                        return;
                    }

                    createSensors(objectServer, sensors, systemBus,
                                  sensorsChanged, reaperTimer);
                });
        };

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(
            *systemBus, std::to_array<const char*>({sensorType}), eventHandler);

    lg2::debug("ExternalSensor service entering main loop");

    io.run();
}
