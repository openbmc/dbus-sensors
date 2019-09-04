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

#include "TachSensor.hpp"

#include "Utils.hpp"

#include <unistd.h>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <fstream>
#include <iostream>
#include <limits>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <string>

static constexpr unsigned int pwmPollMs = 500;
static constexpr size_t warnAfterErrorCount = 10;

TachSensor::TachSensor(const std::string& path, const std::string& objectType,
                       sdbusplus::asio::object_server& objectServer,
                       std::shared_ptr<sdbusplus::asio::connection>& conn,
                       std::unique_ptr<PresenceSensor>&& presenceSensor,
                       std::optional<RedundancySensor>* redundancy,
                       boost::asio::io_service& io, const std::string& fanName,
                       std::vector<thresholds::Threshold>&& _thresholds,
                       const std::string& sensorConfiguration,
                       const std::pair<size_t, size_t>& limits) :
    Sensor(boost::replace_all_copy(fanName, " ", "_"), std::move(_thresholds),
           sensorConfiguration, objectType, limits.second, limits.first),
    path(path), objServer(objectServer), presence(std::move(presenceSensor)),
    redundancy(redundancy), inputDev(io, open(path.c_str(), O_RDONLY)),
    waitTimer(io), errCount(0)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/fan_tach/" + name,
        "xyz.openbmc_project.Sensor.Value");

    if (thresholds::hasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/fan_tach/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Warning");
    }
    if (thresholds::hasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/fan_tach/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Critical");
    }
    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/fan_tach/" + name,
        "org.openbmc.Associations");

    if (presence)
    {
        itemIface =
            objectServer.add_interface("/xyz/openbmc_project/inventory/" + name,
                                       "xyz.openbmc_project.Inventory.Item");
        itemIface->register_property("PrettyName",
                                     std::string()); // unused property
        itemIface->register_property("Present", true);
        itemIface->initialize();
        itemAssoc =
            objectServer.add_interface("/xyz/openbmc_project/inventory/" + name,
                                       "org.openbmc.Associations");
        itemAssoc->register_property(
            "associations",
            std::vector<Association>{
                {"sensors", "inventory",
                 "/xyz/openbmc_project/sensors/fan_tach/" + name}});
        itemAssoc->initialize();
    }
    setInitialProperties(conn);
    setupPowerMatch(conn);
    setupRead();
}

TachSensor::~TachSensor()
{
    // close the input dev to cancel async operations
    inputDev.close();
    waitTimer.cancel();
    objServer.remove_interface(thresholdInterfaceWarning);
    objServer.remove_interface(thresholdInterfaceCritical);
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
    objServer.remove_interface(itemIface);
    objServer.remove_interface(itemAssoc);
}

void TachSensor::setupRead(void)
{
    boost::asio::async_read_until(
        inputDev, readBuf, '\n',
        [&](const boost::system::error_code& ec,
            std::size_t /*bytes_transfered*/) { handleResponse(ec); });
}

void TachSensor::handleResponse(const boost::system::error_code& err)
{
    if (err == boost::system::errc::bad_file_descriptor)
    {
        return; // we're being destroyed
    }
    bool missing = false;
    size_t pollTime = pwmPollMs;
    if (presence)
    {
        if (!presence->getValue())
        {
            updateValue(std::numeric_limits<double>::quiet_NaN());
            missing = true;
            pollTime = sensorFailedPollTimeMs;
        }
        itemIface->set_property("Present", !missing);
    }
    std::istream responseStream(&readBuf);
    if (!missing)
    {
        if (!err)
        {
            std::string response;
            try
            {
                std::getline(responseStream, response);
                float nvalue = std::stof(response);
                responseStream.clear();
                if (static_cast<double>(nvalue) != value)
                {
                    updateValue(nvalue);
                }
                errCount = 0;
            }
            catch (const std::invalid_argument&)
            {
                errCount++;
            }
        }
        else
        {
            if (!isPowerOn())
            {
                errCount = 0;
                updateValue(std::numeric_limits<double>::quiet_NaN());
            }
            else
            {
                pollTime = sensorFailedPollTimeMs;
                errCount++;
            }
        }
        if (errCount >= warnAfterErrorCount)
        {
            // only print once
            if (errCount == warnAfterErrorCount)
            {
                std::cerr << "Failure to read sensor " << name << " at " << path
                          << " ec:" << err << "\n";
            }
            updateValue(0);
        }
    }
    responseStream.clear();
    inputDev.close();
    int fd = open(path.c_str(), O_RDONLY);
    if (fd <= 0)
    {
        return; // we're no longer valid
    }
    inputDev.assign(fd);
    waitTimer.expires_from_now(boost::posix_time::milliseconds(pollTime));
    waitTimer.async_wait([&](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        setupRead();
    });
}

void TachSensor::checkThresholds(void)
{
    if (!isPowerOn())
    {
        return;
    }

    bool status = thresholds::checkThresholds(this);

    if (redundancy && *redundancy)
    {
        (*redundancy)
            ->update("/xyz/openbmc_project/sensors/fan_tach/" + name, !status);
    }
}

PresenceSensor::PresenceSensor(const size_t index, bool inverted,
                               boost::asio::io_service& io,
                               const std::string& name) :
    inverted(inverted),
    inputDev(io), name(name)
{
    // todo: use gpiodaemon
    std::string device = gpioPath + std::string("gpio") + std::to_string(index);
    fd = open((device + "/value").c_str(), O_RDONLY);
    if (fd < 0)
    {
        std::cerr << "Error opening gpio " << index << "\n";
        return;
    }

    std::ofstream deviceFile(device + "/edge");
    if (!deviceFile.good())
    {
        std::cerr << "Error setting edge " << device << "\n";
        return;
    }
    deviceFile << "both";
    deviceFile.close();

    inputDev.assign(boost::asio::ip::tcp::v4(), fd);
    monitorPresence();
    read();
}

PresenceSensor::~PresenceSensor()
{
    inputDev.close();
    close(fd);
}

void PresenceSensor::monitorPresence(void)
{
    inputDev.async_wait(boost::asio::ip::tcp::socket::wait_error,
                        [this](const boost::system::error_code& ec) {
                            if (ec == boost::system::errc::bad_file_descriptor)
                            {
                                return; // we're being destroyed
                            }
                            else if (ec)
                            {
                                std::cerr
                                    << "Error on presence sensor socket\n";
                            }
                            else
                            {
                                read();
                            }
                            monitorPresence();
                        });
}

void PresenceSensor::read(void)
{
    constexpr size_t readSize = sizeof("0");
    std::string readBuf;
    readBuf.resize(readSize);
    lseek(fd, 0, SEEK_SET);
    size_t r = ::read(fd, readBuf.data(), readSize);
    if (r != readSize)
    {
        std::cerr << "Error reading gpio\n";
    }
    else
    {
        bool value = std::stoi(readBuf);
        if (inverted)
        {
            value = !value;
        }
        if (value != status)
        {
            status = value;
            if (status)
            {
                logFanInserted(name);
            }
            else
            {
                logFanRemoved(name);
            }
        }
    }
}

bool PresenceSensor::getValue(void)
{
    return status;
}

RedundancySensor::RedundancySensor(size_t count,
                                   const std::vector<std::string>& children,
                                   sdbusplus::asio::object_server& objectServer,
                                   const std::string& sensorConfiguration) :
    count(count),
    iface(objectServer.add_interface(
        "/xyz/openbmc_project/control/FanRedundancy/Tach",
        "xyz.openbmc_project.Control.FanRedundancy")),
    association(objectServer.add_interface(
        "/xyz/openbmc_project/control/FanRedundancy/Tach",
        "org.openbmc.Associations")),
    objectServer(objectServer)
{
    createAssociation(association, sensorConfiguration);
    iface->register_property("Collection", children);
    iface->register_property("Status", std::string("Full"));
    iface->register_property("AllowedFailures", static_cast<uint8_t>(count));
    iface->initialize();
}
RedundancySensor::~RedundancySensor()
{
    objectServer.remove_interface(association);
    objectServer.remove_interface(iface);
}
void RedundancySensor::update(const std::string& name, bool failed)
{
    statuses[name] = failed;
    size_t failedCount = 0;

    std::string newState = redundancy::full;
    for (const auto& status : statuses)
    {
        if (status.second)
        {
            failedCount++;
        }
        if (failedCount > count)
        {
            newState = redundancy::failed;
            break;
        }
        else if (failedCount)
        {
            newState = redundancy::degraded;
        }
    }
    if (state != newState)
    {
        if (state == redundancy::full)
        {
            logFanRedundancyLost();
        }
        else if (newState == redundancy::full)
        {
            logFanRedundancyRestored();
        }
        state = newState;
        iface->set_property("Status", state);
    }
}
