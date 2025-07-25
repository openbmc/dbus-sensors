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

#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "VariantVisitors.hpp"
#include "sensor.hpp"

#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

constexpr const double altitudeFactor = 1.14;
constexpr const char* exitAirType = "ExitAirTempSensor";
constexpr const char* cfmType = "CFMSensor";

// todo: this *might* need to be configurable
constexpr const char* inletTemperatureSensor = "temperature/Front_Panel_Temp";
constexpr const char* pidConfigurationType =
    "xyz.openbmc_project.Configuration.Pid";
constexpr const char* settingsDaemon = "xyz.openbmc_project.Settings";
constexpr const char* cfmSettingPath = "/xyz/openbmc_project/control/cfm_limit";
constexpr const char* cfmSettingIface = "xyz.openbmc_project.Control.CFMLimit";

static constexpr double cfmMaxReading = 255;
static constexpr double cfmMinReading = 0;

static constexpr size_t minSystemCfm = 50;

constexpr const auto monitorTypes{
    std::to_array<const char*>({exitAirType, cfmType})};

static std::vector<std::shared_ptr<CFMSensor>> cfmSensors;

static void setupSensorMatch(
    std::vector<sdbusplus::bus::match_t>& matches, sdbusplus::bus_t& connection,
    const std::string& type,
    std::function<void(const double&, sdbusplus::message_t&)>&& callback)
{
    std::function<void(sdbusplus::message_t & message)> eventHandler =
        [callback{std::move(callback)}](sdbusplus::message_t& message) {
            std::string objectName;
            boost::container::flat_map<std::string,
                                       std::variant<double, int64_t>>
                values;
            message.read(objectName, values);
            auto findValue = values.find("Value");
            if (findValue == values.end())
            {
                return;
            }
            double value =
                std::visit(VariantToDoubleVisitor(), findValue->second);
            if (std::isnan(value))
            {
                return;
            }

            callback(value, message);
        };
    matches.emplace_back(
        connection,
        "type='signal',"
        "member='PropertiesChanged',interface='org."
        "freedesktop.DBus.Properties',path_"
        "namespace='/xyz/openbmc_project/sensors/" +
            std::string(type) + "',arg0='xyz.openbmc_project.Sensor.Value'",
        std::move(eventHandler));
}

static void setMaxPWM(const std::shared_ptr<sdbusplus::asio::connection>& conn,
                      double value)
{
    conn->async_method_call(
        [conn,
         value](const boost::system::error_code ec, const GetSubTreeType& ret) {
            if (ec)
            {
                lg2::error("Error calling mapper");
                return;
            }
            for (const auto& [path, objDict] : ret)
            {
                if (objDict.empty())
                {
                    return;
                }
                const std::string& owner = objDict.begin()->first;

                conn->async_method_call(
                    [conn, value, owner,
                     path{path}](const boost::system::error_code ec,
                                 const std::variant<std::string>& classType) {
                        if (ec)
                        {
                            lg2::error("Error getting pid class");
                            return;
                        }
                        const auto* classStr =
                            std::get_if<std::string>(&classType);
                        if (classStr == nullptr || *classStr != "fan")
                        {
                            return;
                        }
                        conn->async_method_call(
                            [](boost::system::error_code& ec) {
                                if (ec)
                                {
                                    lg2::error("Error setting pid class");
                                    return;
                                }
                            },
                            owner, path, "org.freedesktop.DBus.Properties",
                            "Set", pidConfigurationType, "OutLimitMax",
                            std::variant<double>(value));
                    },
                    owner, path, "org.freedesktop.DBus.Properties", "Get",
                    pidConfigurationType, "Class");
            }
        },
        mapper::busName, mapper::path, mapper::interface, mapper::subtree, "/",
        0, std::array<std::string, 1>{pidConfigurationType});
}

CFMSensor::CFMSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                     const std::string& sensorName,
                     const std::string& sensorConfiguration,
                     sdbusplus::asio::object_server& objectServer,
                     std::vector<thresholds::Threshold>&& thresholdData,
                     std::shared_ptr<ExitAirTempSensor>& parent) :
    Sensor(escapeName(sensorName), std::move(thresholdData),
           sensorConfiguration, "CFMSensor", false, false, cfmMaxReading,
           cfmMinReading, conn, PowerState::on),
    parent(parent), objServer(objectServer)
{
    std::string dbusPath = "/xyz/openbmc_project/sensors/airflow/" + name;
    sensorInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        size_t index = static_cast<size_t>(threshold.level);
        if (thresholdInterfaces[index])
        {
            lg2::error("{INTERFACE} under {PATH} has already been created",
                       "INTERFACE", interface, "PATH", dbusPath);
            continue;
        }

        thresholdInterfaces[index] =
            objectServer.add_interface(dbusPath, interface);
    }

    association = objectServer.add_interface(dbusPath, association::interface);

    setInitialProperties(sensor_paths::unitCFM);

    pwmLimitIface =
        objectServer.add_interface("/xyz/openbmc_project/control/pwm_limit",
                                   "xyz.openbmc_project.Control.PWMLimit");
    cfmLimitIface =
        objectServer.add_interface("/xyz/openbmc_project/control/MaxCFM",
                                   "xyz.openbmc_project.Control.CFMLimit");
}

void CFMSensor::setupMatches()
{
    std::weak_ptr<CFMSensor> weakRef = weak_from_this();
    setupSensorMatch(
        matches, *dbusConnection, "fan_tach",
        [weakRef](const double& value, sdbusplus::message_t& message) {
            auto self = weakRef.lock();
            if (!self)
            {
                return;
            }
            self->tachReadings[message.get_path()] = value;
            if (self->tachRanges.find(message.get_path()) ==
                self->tachRanges.end())
            {
                // calls update reading after updating ranges
                self->addTachRanges(message.get_sender(), message.get_path());
            }
            else
            {
                self->updateReading();
            }
        });

    dbusConnection->async_method_call(
        [weakRef](const boost::system::error_code ec,
                  const std::variant<double> cfmVariant) {
            auto self = weakRef.lock();
            if (!self)
            {
                return;
            }

            uint64_t maxRpm = 100;
            if (!ec)
            {
                const auto* cfm = std::get_if<double>(&cfmVariant);
                if (cfm != nullptr && *cfm >= minSystemCfm)
                {
                    maxRpm = self->getMaxRpm(*cfm);
                }
            }
            self->pwmLimitIface->register_property("Limit", maxRpm);
            self->pwmLimitIface->initialize();
            setMaxPWM(self->dbusConnection, maxRpm);
        },
        settingsDaemon, cfmSettingPath, "org.freedesktop.DBus.Properties",
        "Get", cfmSettingIface, "Limit");

    matches.emplace_back(
        *dbusConnection,
        "type='signal',"
        "member='PropertiesChanged',interface='org."
        "freedesktop.DBus.Properties',path='" +
            std::string(cfmSettingPath) + "',arg0='" +
            std::string(cfmSettingIface) + "'",
        [weakRef](sdbusplus::message_t& message) {
            auto self = weakRef.lock();
            if (!self)
            {
                return;
            }
            boost::container::flat_map<std::string, std::variant<double>>
                values;
            std::string objectName;
            message.read(objectName, values);
            const auto findValue = values.find("Limit");
            if (findValue == values.end())
            {
                return;
            }
            auto* const reading = std::get_if<double>(&(findValue->second));
            if (reading == nullptr)
            {
                lg2::error("Got CFM Limit of wrong type");
                return;
            }
            if (*reading < minSystemCfm && *reading != 0)
            {
                lg2::error("Illegal CFM setting detected");
                return;
            }
            uint64_t maxRpm = self->getMaxRpm(*reading);
            self->pwmLimitIface->set_property("Limit", maxRpm);
            setMaxPWM(self->dbusConnection, maxRpm);
        });
}

CFMSensor::~CFMSensor()
{
    for (const auto& iface : thresholdInterfaces)
    {
        objServer.remove_interface(iface);
    }
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
    objServer.remove_interface(cfmLimitIface);
    objServer.remove_interface(pwmLimitIface);
}

void CFMSensor::createMaxCFMIface()
{
    cfmLimitIface->register_property("Limit", c2 * maxCFM * tachs.size());
    cfmLimitIface->initialize();
}

void CFMSensor::addTachRanges(const std::string& serviceName,
                              const std::string& path)
{
    std::weak_ptr<CFMSensor> weakRef = weak_from_this();
    dbusConnection->async_method_call(
        [weakRef, path](const boost::system::error_code ec,
                        const SensorBaseConfigMap& data) {
            if (ec)
            {
                lg2::error("Error getting properties from '{PATH}'", "PATH",
                           path);
                return;
            }
            auto self = weakRef.lock();
            if (!self)
            {
                return;
            }
            double max = loadVariant<double>(data, "MaxValue");
            double min = loadVariant<double>(data, "MinValue");
            self->tachRanges[path] = std::make_pair(min, max);
            self->updateReading();
        },
        serviceName, path, "org.freedesktop.DBus.Properties", "GetAll",
        "xyz.openbmc_project.Sensor.Value");
}

void CFMSensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}

void CFMSensor::updateReading()
{
    double val = 0.0;
    if (calculate(val))
    {
        if (value != val && parent)
        {
            parent->updateReading();
        }
        updateValue(val);
    }
    else
    {
        updateValue(std::numeric_limits<double>::quiet_NaN());
    }
}

uint64_t CFMSensor::getMaxRpm(uint64_t cfmMaxSetting) const
{
    uint64_t pwmPercent = 100;
    double totalCFM = std::numeric_limits<double>::max();
    if (cfmMaxSetting == 0)
    {
        return pwmPercent;
    }

    bool firstLoop = true;
    while (totalCFM > cfmMaxSetting)
    {
        if (firstLoop)
        {
            firstLoop = false;
        }
        else
        {
            pwmPercent--;
        }

        double ci = 0;
        if (pwmPercent == 0)
        {
            ci = 0;
        }
        else if (pwmPercent < tachMinPercent)
        {
            ci = c1;
        }
        else if (pwmPercent > tachMaxPercent)
        {
            ci = c2;
        }
        else
        {
            ci = c1 + (((c2 - c1) * (pwmPercent - tachMinPercent)) /
                       (tachMaxPercent - tachMinPercent));
        }

        // Now calculate the CFM for this tach
        // CFMi = Ci * Qmaxi * TACHi
        totalCFM = ci * maxCFM * pwmPercent;
        totalCFM *= tachs.size();
        // divide by 100 since pwm is in percent
        totalCFM /= 100;

        if (pwmPercent <= 0)
        {
            break;
        }
    }

    return pwmPercent;
}

bool CFMSensor::calculate(double& value)
{
    double totalCFM = 0;
    for (const std::string& tachName : tachs)
    {
        auto findReading = std::find_if(
            tachReadings.begin(), tachReadings.end(),
            [&](const auto& item) { return item.first.ends_with(tachName); });
        auto findRange = std::find_if(
            tachRanges.begin(), tachRanges.end(),
            [&](const auto& item) { return item.first.ends_with(tachName); });
        if (findReading == tachReadings.end())
        {
            lg2::debug("Can't find '{NAME}' in readings", "NAME", tachName);
            continue; // haven't gotten a reading
        }

        if (findRange == tachRanges.end())
        {
            lg2::error("Can't find '{NAME}' in ranges", "NAME", tachName);
            return false; // haven't gotten a max / min
        }

        // avoid divide by 0
        if (findRange->second.second == 0)
        {
            lg2::error("Tach Max Set to 0, tachName: '{NAME}'", "NAME",
                       tachName);
            return false;
        }

        double rpm = findReading->second;

        // for now assume the min for a fan is always 0, divide by max to get
        // percent and mult by 100
        rpm /= findRange->second.second;
        rpm *= 100;

        lg2::debug("Tach '{NAME}' at '{RPM}'", "NAME", tachName, "RPM", rpm);

        // Do a linear interpolation to get Ci
        // Ci = C1 + (C2 - C1)/(RPM2 - RPM1) * (TACHi - TACH1)

        double ci = 0;
        if (rpm == 0)
        {
            ci = 0;
        }
        else if (rpm < tachMinPercent)
        {
            ci = c1;
        }
        else if (rpm > tachMaxPercent)
        {
            ci = c2;
        }
        else
        {
            ci = c1 + (((c2 - c1) * (rpm - tachMinPercent)) /
                       (tachMaxPercent - tachMinPercent));
        }

        // Now calculate the CFM for this tach
        // CFMi = Ci * Qmaxi * TACHi
        totalCFM += ci * maxCFM * rpm;
        lg2::debug(
            "totalCFM = {CFM}, Ci = {CI}, MaxCFM = {MAXCFM}, rpm = {RPM}, c1 = {C1}"
            ", c2 = {C2}, max = {MAX}, min = {MIN}",
            "CFM", totalCFM, "CI", ci, "MAXCFM", maxCFM, "RPM", rpm, "C1", c1,
            "C2", c2, "MAX", tachMaxPercent, "MIN", tachMinPercent);
    }

    // divide by 100 since rpm is in percent
    value = totalCFM / 100;
    lg2::debug("cfm value = {VALUE}", "VALUE", value);
    return true;
}

static constexpr double exitAirMaxReading = 127;
static constexpr double exitAirMinReading = -128;
ExitAirTempSensor::ExitAirTempSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    const std::string& sensorName, const std::string& sensorConfiguration,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData) :
    Sensor(escapeName(sensorName), std::move(thresholdData),
           sensorConfiguration, "ExitAirTemp", false, false, exitAirMaxReading,
           exitAirMinReading, conn, PowerState::on),
    objServer(objectServer)
{
    std::string dbusPath = "/xyz/openbmc_project/sensors/temperature/" + name;
    sensorInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        size_t index = static_cast<size_t>(threshold.level);
        if (thresholdInterfaces[index])
        {
            lg2::error("{INTERFACE} under {PATH} has already been created",
                       "INTERFACE", interface, "PATH", dbusPath);
            continue;
        }

        thresholdInterfaces[index] =
            objectServer.add_interface(dbusPath, interface);
    }
    association = objectServer.add_interface(dbusPath, association::interface);
    setInitialProperties(sensor_paths::unitDegreesC);
}

ExitAirTempSensor::~ExitAirTempSensor()
{
    for (const auto& iface : thresholdInterfaces)
    {
        objServer.remove_interface(iface);
    }
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

void ExitAirTempSensor::setupMatches()
{
    constexpr const auto matchTypes{
        std::to_array<const char*>({"power", inletTemperatureSensor})};

    std::weak_ptr<ExitAirTempSensor> weakRef = weak_from_this();
    for (const std::string type : matchTypes)
    {
        setupSensorMatch(
            matches, *dbusConnection, type,
            [weakRef,
             type](const double& value, sdbusplus::message_t& message) {
                auto self = weakRef.lock();
                if (!self)
                {
                    return;
                }
                if (type == "power")
                {
                    std::string path = message.get_path();
                    if (path.find("PS") != std::string::npos &&
                        path.ends_with("Input_Power"))
                    {
                        self->powerReadings[message.get_path()] = value;
                    }
                }
                else if (type == inletTemperatureSensor)
                {
                    self->inletTemp = value;
                }
                self->updateReading();
            });
    }
    dbusConnection->async_method_call(
        [weakRef](boost::system::error_code ec,
                  const std::variant<double>& value) {
            if (ec)
            {
                // sensor not ready yet
                return;
            }
            auto self = weakRef.lock();
            if (!self)
            {
                return;
            }
            self->inletTemp = std::visit(VariantToDoubleVisitor(), value);
        },
        "xyz.openbmc_project.HwmonTempSensor",
        std::string("/xyz/openbmc_project/sensors/") + inletTemperatureSensor,
        properties::interface, properties::get, sensorValueInterface, "Value");
    dbusConnection->async_method_call(
        [weakRef](boost::system::error_code ec, const GetSubTreeType& subtree) {
            if (ec)
            {
                lg2::error("Error contacting mapper");
                return;
            }
            auto self = weakRef.lock();
            if (!self)
            {
                return;
            }
            for (const auto& [path, matches] : subtree)
            {
                size_t lastSlash = path.rfind('/');
                if (lastSlash == std::string::npos ||
                    lastSlash == path.size() || matches.empty())
                {
                    continue;
                }
                std::string sensorName = path.substr(lastSlash + 1);
                if (sensorName.starts_with("PS") &&
                    sensorName.ends_with("Input_Power"))
                {
                    // lambda capture requires a proper variable (not a
                    // structured binding)
                    const std::string& cbPath = path;
                    self->dbusConnection->async_method_call(
                        [weakRef, cbPath](boost::system::error_code ec,
                                          const std::variant<double>& value) {
                            if (ec)
                            {
                                lg2::error("Error getting value from '{PATH}'",
                                           "PATH", cbPath);
                            }
                            auto self = weakRef.lock();
                            if (!self)
                            {
                                return;
                            }
                            double reading =
                                std::visit(VariantToDoubleVisitor(), value);
                            lg2::debug("'{PATH}' reading '{VALUE}'", "PATH",
                                       cbPath, "VALUE", reading);
                            self->powerReadings[cbPath] = reading;
                        },
                        matches[0].first, cbPath, properties::interface,
                        properties::get, sensorValueInterface, "Value");
                }
            }
        },
        mapper::busName, mapper::path, mapper::interface, mapper::subtree,
        "/xyz/openbmc_project/sensors/power", 0,
        std::array<const char*, 1>{sensorValueInterface});
}

void ExitAirTempSensor::updateReading()
{
    double val = 0.0;
    if (calculate(val))
    {
        val = std::floor(val + 0.5);
        updateValue(val);
    }
    else
    {
        updateValue(std::numeric_limits<double>::quiet_NaN());
    }
}

double ExitAirTempSensor::getTotalCFM()
{
    double sum = 0;
    for (auto& sensor : cfmSensors)
    {
        double reading = 0;
        if (!sensor->calculate(reading))
        {
            return -1;
        }
        sum += reading;
    }

    return sum;
}

bool ExitAirTempSensor::calculate(double& val)
{
    constexpr size_t maxErrorPrint = 5;
    static bool firstRead = false;
    static size_t errorPrint = maxErrorPrint;

    double cfm = getTotalCFM();
    if (cfm <= 0)
    {
        lg2::error("Error getting cfm");
        return false;
    }

    // Though cfm is not expected to be less than qMin normally,
    // it is not a hard limit for exit air temp calculation.
    // 50% qMin is chosen as a generic limit between providing
    // a valid derived exit air temp and reporting exit air temp not available.
    constexpr const double cfmLimitFactor = 0.5;
    if (cfm < (qMin * cfmLimitFactor))
    {
        if (errorPrint > 0)
        {
            errorPrint--;
            lg2::error("cfm '{CFM}' is too low, expected qMin '{QMIN}'", "CFM",
                       cfm, "QMIN", qMin);
        }
        val = 0;
        return false;
    }

    // if there is an error getting inlet temp, return error
    if (std::isnan(inletTemp))
    {
        if (errorPrint > 0)
        {
            errorPrint--;
            lg2::error("Cannot get inlet temp");
        }
        val = 0;
        return false;
    }

    // if fans are off, just make the exit temp equal to inlet
    if (!isPowerOn())
    {
        val = inletTemp;
        return true;
    }

    double totalPower = 0;
    for (const auto& [path, reading] : powerReadings)
    {
        if (std::isnan(reading))
        {
            continue;
        }
        totalPower += reading;
    }

    // Calculate power correction factor
    // Ci = CL + (CH - CL)/(QMax - QMin) * (CFM - QMin)
    double powerFactor = 0.0;
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
        if (errorPrint > 0)
        {
            errorPrint--;
            lg2::error("total power 0");
        }
        val = 0;
        return false;
    }

    lg2::debug(
        "Power Factor: {POWER_FACTOR}, Inlet Temp: {INLET_TEMP}, Total Power: {TOTAL_POWER}",
        "POWER_FACTOR", powerFactor, "INLET_TEMP", inletTemp, "TOTAL_POWER",
        totalPower);

    // Calculate the exit air temp
    // Texit = Tfp + (1.76 * TotalPower / CFM * Faltitude)
    double reading = 1.76 * totalPower * altitudeFactor;
    reading /= cfm;
    reading += inletTemp;

    lg2::debug("Reading 1: '{VALUE}'", "VALUE", reading);

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

    auto time = std::chrono::steady_clock::now();
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

    lg2::debug("AlphaDT: '{ADT}'", "ADT", alphaDT);

    reading = ((reading * alphaDT) + (lastReading * (1.0 - alphaDT)));

    lg2::debug("Reading 2: '{VALUE}'", "VALUE", reading);

    val = reading;
    lastReading = reading;
    lastTime = time;
    errorPrint = maxErrorPrint;
    return true;
}

void ExitAirTempSensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}

static void loadVariantPathArray(const SensorBaseConfigMap& data,
                                 const std::string& key,
                                 std::vector<std::string>& resp)
{
    auto it = data.find(key);
    if (it == data.end())
    {
        lg2::error("Configuration missing '{KEY}'", "KEY", key);
        throw std::invalid_argument("Key Missing");
    }
    BasicVariantType copy = it->second;
    std::vector<std::string> config = std::get<std::vector<std::string>>(copy);
    for (auto& str : config)
    {
        boost::replace_all(str, " ", "_");
    }
    resp = std::move(config);
}

void createSensor(sdbusplus::asio::object_server& objectServer,
                  std::shared_ptr<ExitAirTempSensor>& exitAirSensor,
                  std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    if (!dbusConnection)
    {
        lg2::error("Connection not created");
        return;
    }
    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection, [&objectServer, &dbusConnection,
                         &exitAirSensor](const ManagedObjectType& resp) {
            cfmSensors.clear();
            for (const auto& [path, interfaces] : resp)
            {
                for (const auto& [intf, cfg] : interfaces)
                {
                    if (intf == configInterfaceName(exitAirType))
                    {
                        // thresholds should be under the same path
                        std::vector<thresholds::Threshold> sensorThresholds;
                        parseThresholdsFromConfig(interfaces, sensorThresholds);

                        std::string name =
                            loadVariant<std::string>(cfg, "Name");
                        exitAirSensor = nullptr;
                        exitAirSensor = std::make_shared<ExitAirTempSensor>(
                            dbusConnection, name, path.str, objectServer,
                            std::move(sensorThresholds));
                        exitAirSensor->powerFactorMin =
                            loadVariant<double>(cfg, "PowerFactorMin");
                        exitAirSensor->powerFactorMax =
                            loadVariant<double>(cfg, "PowerFactorMax");
                        exitAirSensor->qMin = loadVariant<double>(cfg, "QMin");
                        exitAirSensor->qMax = loadVariant<double>(cfg, "QMax");
                        exitAirSensor->alphaS =
                            loadVariant<double>(cfg, "AlphaS");
                        exitAirSensor->alphaF =
                            loadVariant<double>(cfg, "AlphaF");
                    }
                    else if (intf == configInterfaceName(cfmType))
                    {
                        // thresholds should be under the same path
                        std::vector<thresholds::Threshold> sensorThresholds;
                        parseThresholdsFromConfig(interfaces, sensorThresholds);
                        std::string name =
                            loadVariant<std::string>(cfg, "Name");
                        auto sensor = std::make_shared<CFMSensor>(
                            dbusConnection, name, path.str, objectServer,
                            std::move(sensorThresholds), exitAirSensor);
                        loadVariantPathArray(cfg, "Tachs", sensor->tachs);
                        sensor->maxCFM = loadVariant<double>(cfg, "MaxCFM");

                        // change these into percent upon getting the data
                        sensor->c1 = loadVariant<double>(cfg, "C1") / 100;
                        sensor->c2 = loadVariant<double>(cfg, "C2") / 100;
                        sensor->tachMinPercent =
                            loadVariant<double>(cfg, "TachMinPercent");
                        sensor->tachMaxPercent =
                            loadVariant<double>(cfg, "TachMaxPercent");
                        sensor->createMaxCFMIface();
                        sensor->setupMatches();

                        cfmSensors.emplace_back(std::move(sensor));
                    }
                }
            }
            if (exitAirSensor)
            {
                exitAirSensor->setupMatches();
                exitAirSensor->updateReading();
            }
        });
    getter->getConfiguration(
        std::vector<std::string>(monitorTypes.begin(), monitorTypes.end()));
}

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");
    systemBus->request_name("xyz.openbmc_project.ExitAirTempSensor");
    std::shared_ptr<ExitAirTempSensor> sensor =
        nullptr; // wait until we find the config

    boost::asio::post(io,
                      [&]() { createSensor(objectServer, sensor, systemBus); });

    boost::asio::steady_timer configTimer(io);

    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&](sdbusplus::message_t&) {
            configTimer.expires_after(std::chrono::seconds(1));
            // create a timer because normally multiple properties change
            configTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return; // we're being canceled
                }
                createSensor(objectServer, sensor, systemBus);
                if (!sensor)
                {
                    lg2::info("Configuration not detected");
                }
            });
        };
    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(*systemBus, monitorTypes, eventHandler);

    setupManufacturingModeMatch(*systemBus);
    io.run();
    return 0;
}
