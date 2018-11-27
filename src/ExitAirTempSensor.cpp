/*
// Copyright (c) 2018 Intel Corporation
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

#include "ExitAirTempSensor.hpp"

#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <math.h>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <chrono>
#include <iostream>
#include <limits>
#include <numeric>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <vector>

constexpr const float altitudeFactor = 1.14;
constexpr const char* exitAirIface =
    "xyz.openbmc_project.Configuration.ExitAirTempSensor";
constexpr const char* cfmIface = "xyz.openbmc_project.Configuration.CFMSensor";

// todo: this *might* need to be configurable
constexpr const char* inletTemperatureSensor = "temperature/Front_Panel_Temp";
static constexpr double maxReading = 127;
static constexpr double minReading = -128;

static constexpr bool DEBUG = false;

ExitAirTempSensor::ExitAirTempSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    const std::string& sensorConfiguration,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholds) :
    Sensor("Exit_Air_Temperature", "" /* todo: remove arg from base*/,
           std::move(thresholds), sensorConfiguration,
           "xyz.openbmc_project.Configuration.ExitAirTemp", maxReading,
           minReading),
    dbusConnection(conn)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        "xyz.openbmc_project.Sensor.Value");

    if (thresholds::hasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/temperature/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Warning");
    }
    if (thresholds::hasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/temperature/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Critical");
    }
    setInitialProperties(conn);
    setupMatches();
}

ExitAirTempSensor::~ExitAirTempSensor()
{
    // this sensor currently isn't destroyed so we don't care
}

void ExitAirTempSensor::setupMatches(void)
{

    constexpr const std::array<const char*, 3> matchTypes = {
        "power", "fan_pwm", inletTemperatureSensor};

    for (const auto& type : matchTypes)
    {
        std::function<void(sdbusplus::message::message & message)>
            eventHandler = [this, type](sdbusplus::message::message& message) {
                std::string objectName;
                boost::container::flat_map<
                    std::string, sdbusplus::message::variant<double, int64_t>>
                    values;
                message.read(objectName, values);
                auto findValue = values.find("Value");
                if (findValue == values.end())
                {
                    return;
                }
                double value = sdbusplus::message::variant_ns::visit(
                    VariantToDoubleVisitor(), findValue->second);
                if (type == "power")
                {
                    powerReadings[message.get_path()] = value;
                }
                else if (type == inletTemperatureSensor)
                {
                    inletTemp = value;
                }
                else if (type == "fan_pwm")
                {
                    pwmReadings[message.get_path()] = value;
                }
                updateReading();
            };
        matches.emplace_back(static_cast<sdbusplus::bus::bus&>(*dbusConnection),
                             "type='signal',"
                             "member='PropertiesChanged',interface='org."
                             "freedesktop.DBus.Properties',path_"
                             "namespace='/xyz/openbmc_project/sensors/" +
                                 std::string(type) +
                                 "',arg0='xyz.openbmc_project.Sensor.Value'",
                             std::move(eventHandler));
    }
}

void ExitAirTempSensor::updateReading(void)
{

    double val = 0.0;
    if (calculate(val))
    {
        updateValue(val);
    }
    else
    {
        updateValue(std::numeric_limits<double>::quiet_NaN());
    }
}

// todo: break this out into it's own sensor
int32_t ExitAirTempSensor::getTotalCFM(void)
{
    int32_t totalCFM = 0;
    // todo: rpm instead of pwm
    for (const auto& zone : cfmData)
    {
        if (zone.pwm.empty())
        {
            std::cerr << "CFM without PWM";
            return -1;
        }

        const std::string& firstName = zone.pwm[0];

        auto findPwm = std::find_if(
            pwmReadings.begin(), pwmReadings.end(), [&](const auto& item) {
                return boost::ends_with(item.first, firstName);
            });
        if (findPwm == pwmReadings.end())
        {
            std::cerr << "Can't find " << firstName << "in readings\n";
            return -1; // haven't gotten a reading
        }

        double pwm = findPwm->second;
        if constexpr (DEBUG)
        {
            std::cout << "Pwm " << firstName << "at " << pwm << "\n";
        }

        // Do a linear interpolation to get Ci
        // Ci = C1 + (C2 - C1)/(PWM2 - PWM1) * (PWMi - PWM1)

        int32_t ci = 0;
        if (pwm == 0)
        {
            ci = 0;
        }
        else if (pwm < zone.pwmMin)
        {
            ci = zone.c1;
        }
        else if (pwm > zone.pwmMax)
        {
            ci = zone.c2;
        }
        else
        {
            ci = zone.c1 +
                 (((zone.c2 - zone.c1) * (pwm - (int32_t)zone.pwmMin)) /
                  ((int32_t)zone.pwmMax - (int32_t)zone.pwmMin));
        }

        // Now calculate the CFM for this domain
        // CFMi = Ci * QTYi * Qmaxi * PWMi
        totalCFM += ci * zone.pwm.size() * zone.maxCFM * pwm;
    }
    if constexpr (DEBUG)
    {
        std::cout << totalCFM / 100 << " CFM\n";
    }
    return totalCFM /= 100; // divide by 100 since PWM is in percent
}

bool ExitAirTempSensor::calculate(double& val)
{
    static bool firstRead = false;
    double cfm = getTotalCFM();
    if (cfm <= 0)
    {
        std::cerr << "Error getting cfm\n";
        return false;
    }

    // if there is an error getting inlet temp, return error
    if (std::isnan(inletTemp))
    {
        std::cerr << "Cannot get inlet temp\n";
        val = 0;
        return false;
    }

    // if fans are off, just make the exit temp equal to inlet
    if (!isPowerOn(dbusConnection))
    {
        val = inletTemp;
        return true;
    }

    double totalPower = 0;
    for (const auto& reading : powerReadings)
    {
        if (std::isnan(reading.second))
        {
            continue;
        }
        totalPower += reading.second;
    }

    // Calculate power correction factor
    // Ci = CL + (CH - CL)/(QMax - QMin) * (CFM - QMin)
    float powerFactor = 0.0;
    if (cfm <= qMin)
    {
        powerFactor = powerFactorMin;
    }
    else if (cfm >= qMax)
    {
        powerFactor = powerFactorMax;
    }
    else
    {
        powerFactor = powerFactorMin + ((powerFactorMax - powerFactorMin) /
                                        (qMax - qMin) * (cfm - qMin));
    }

    totalPower *= powerFactor;
    totalPower += pOffset;

    if (totalPower == 0)
    {
        std::cerr << "total power 0\n";
        val = 0;
        return false;
    }

    if constexpr (DEBUG)
    {
        std::cout << "Power Factor " << powerFactor << "\n";
        std::cout << "Inlet Temp " << inletTemp << "\n";
        std::cout << "Total Power" << totalPower << "\n";
    }

    // Calculate the exit air temp
    // Texit = Tfp + (1.76 * TotalPower / CFM * Faltitude)
    double reading = 1.76 * totalPower * altitudeFactor;
    reading /= cfm;
    reading += inletTemp;

    if constexpr (DEBUG)
    {
        std::cout << "Reading 1: " << reading << "\n";
    }

    // Now perform the exponential average
    // Calculate alpha based on SDR values and CFM
    // Ai = As + (Af - As)/(QMax - QMin) * (CFM - QMin)

    double alpha = 0.0;
    if (cfm < qMin)
    {
        alpha = alphaS;
    }
    else if (cfm >= qMax)
    {
        alpha = alphaF;
    }
    else
    {
        alpha = alphaS + ((alphaF - alphaS) * (cfm - qMin) / (qMax - qMin));
    }

    auto time = std::chrono::system_clock::now();
    if (!firstRead)
    {
        firstRead = true;
        lastTime = time;
        lastReading = reading;
    }
    double alphaDT =
        std::chrono::duration_cast<std::chrono::seconds>(time - lastTime)
            .count() *
        alpha;

    // cap at 1.0 or the below fails
    if (alphaDT > 1.0)
    {
        alphaDT = 1.0;
    }

    if constexpr (DEBUG)
    {
        std::cout << "AlphaDT: " << alphaDT << "\n";
    }

    reading = ((reading * alphaDT) + (lastReading * (1.0 - alphaDT)));

    if constexpr (DEBUG)
    {
        std::cout << "Reading 2: " << reading << "\n";
    }

    val = reading;
    lastReading = reading;
    lastTime = time;
    return true;
}

void ExitAirTempSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}

static double loadVariantDouble(
    const boost::container::flat_map<std::string, BasicVariantType>& data,
    const std::string& key)
{
    auto it = data.find(key);
    if (it == data.end())
    {
        std::cerr << "Configuration missing " << key << "\n";
        throw std::invalid_argument("Key Missing");
    }
    return sdbusplus::message::variant_ns::visit(VariantToDoubleVisitor(),
                                                 it->second);
}

static void loadVariantPathArray(
    const boost::container::flat_map<std::string, BasicVariantType>& data,
    const std::string& key, std::vector<std::string>& resp)
{
    auto it = data.find(key);
    if (it == data.end())
    {
        std::cerr << "Configuration missing " << key << "\n";
        throw std::invalid_argument("Key Missing");
    }
    BasicVariantType copy = it->second;
    std::vector<std::string> config =
        sdbusplus::message::variant_ns::get<std::vector<std::string>>(copy);
    for (auto& str : config)
    {
        boost::replace_all(str, " ", "_");
    }
    resp = std::move(config);
}

void createSensor(sdbusplus::asio::object_server& objectServer,
                  std::shared_ptr<ExitAirTempSensor>& sensor,
                  std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    if (!dbusConnection)
    {
        std::cerr << "Connection not created\n";
        return;
    }
    dbusConnection->async_method_call(
        [&](boost::system::error_code ec, const ManagedObjectType& resp) {
            if (ec)
            {
                std::cerr << "Error contacting entity manager\n";
                return;
            }

            std::vector<CFMInfo> cfmData;
            bool foundExitAir = false;
            for (const auto& pathPair : resp)
            {
                for (const auto& entry : pathPair.second)
                {
                    if (entry.first == exitAirIface)
                    {
                        if (foundExitAir)
                        {
                            // Something is very wrong
                            std::cerr << "More than one exit air configuration "
                                         "found\n";
                            std::exit(EXIT_FAILURE);
                        }
                        foundExitAir = true;

                        // thresholds should be under the same path
                        std::vector<thresholds::Threshold> sensorThresholds;
                        parseThresholdsFromConfig(pathPair.second,
                                                  sensorThresholds);
                        if (!sensor)
                        {
                            sensor = std::make_shared<ExitAirTempSensor>(
                                dbusConnection, pathPair.first.str,
                                objectServer, std::move(sensorThresholds));
                        }
                        else
                        {
                            sensor->thresholds = sensorThresholds;
                        }

                        sensor->powerFactorMin =
                            loadVariantDouble(entry.second, "PowerFactorMin");
                        sensor->powerFactorMax =
                            loadVariantDouble(entry.second, "PowerFactorMax");
                        sensor->qMin = loadVariantDouble(entry.second, "QMin");
                        sensor->qMax = loadVariantDouble(entry.second, "QMax");
                        sensor->alphaS =
                            loadVariantDouble(entry.second, "AlphaS");
                        sensor->alphaF =
                            loadVariantDouble(entry.second, "AlphaF");
                    }
                    else if (entry.first == cfmIface)

                    {

                        CFMInfo cfm;
                        loadVariantPathArray(entry.second, "Pwms", cfm.pwm);
                        cfm.maxCFM = loadVariantDouble(entry.second, "MaxCFM");

                        // change these into percent upon getting the data
                        cfm.c1 = loadVariantDouble(entry.second, "C1") / 100;
                        cfm.c2 = loadVariantDouble(entry.second, "C2") / 100;
                        cfm.pwmMin =
                            loadVariantDouble(entry.second, "PwmMinPercent") /
                            100;
                        cfm.pwmMax =
                            loadVariantDouble(entry.second, "PwmMaxPercent") /
                            100;

                        cfmData.emplace_back(cfm);
                    }
                }
            }
            if (sensor)
            {
                sensor->cfmData = std::move(cfmData);

                // todo: when power sensors are done delete this fake reading
                sensor->powerReadings["foo"] = 144.0;

                sensor->updateReading();
            }
        },
        entityManagerName, "/", "org.freedesktop.DBus.ObjectManager",
        "GetManagedObjects");
}

int main(int argc, char** argv)
{

    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.ExitAirTempSensor");
    sdbusplus::asio::object_server objectServer(systemBus);
    std::shared_ptr<ExitAirTempSensor> sensor =
        nullptr; // wait until we find the config
    std::vector<std::unique_ptr<sdbusplus::bus::match::match>> matches;

    io.post([&]() { createSensor(objectServer, sensor, systemBus); });

    boost::asio::deadline_timer configTimer(io);

    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message& message) {
            configTimer.expires_from_now(boost::posix_time::seconds(1));
            // create a timer because normally multiple properties change
            configTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return; // we're being canceled
                }
                createSensor(objectServer, sensor, systemBus);
                if (!sensor)
                {
                    std::cout << "Configuration not detected\n";
                }
            });
        };
    constexpr const std::array<const char*, 2> monitorIfaces = {exitAirIface,
                                                                cfmIface};
    for (const char* type : monitorIfaces)
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
