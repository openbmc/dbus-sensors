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

#include <unistd.h>

#include <CPUSensor.hpp>
#include <Utils.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <iostream>
#include <istream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

CPUSensor::CPUSensor(const std::string& path, const std::string& objectType,
                     sdbusplus::asio::object_server& objectServer,
                     std::shared_ptr<sdbusplus::asio::connection>& conn,
                     boost::asio::io_service& io, const std::string& sensorName,
                     std::vector<thresholds::Threshold>&& thresholds,
                     const std::string& sensorConfiguration, int cpuId,
                     bool show, double dtsOffset) :
    sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(thresholds), sensorConfiguration, objectType, 0, 0, conn,
           PowerState::on),
    objServer(objectServer), inputDev(io), waitTimer(io), path(path),
    privTcontrol(std::numeric_limits<double>::quiet_NaN()),
    dtsOffset(dtsOffset), show(show), pollTime(CPUSensor::sensorPollMs),
    minMaxReadCounter(0)
{
    sensor.checkThresholdsFunc = [this]() { checkThresholds(); };
    nameTcontrol = labelTcontrol;
    nameTcontrol += " CPU" + std::to_string(cpuId);
    if (show)
    {
        if (auto fileParts = splitFileName(path))
        {
            auto& [type, nr, item] = *fileParts;
            std::string interfacePath;
            if (type.compare("power") == 0)
            {
                interfacePath = "/xyz/openbmc_project/sensors/power/" + sensor.name;
                sensor.minValue = 0;
                sensor.maxValue = 511;
            }
            else
            {
                interfacePath =
                    "/xyz/openbmc_project/sensors/temperature/" + sensor.name;
                sensor.minValue = -128;
                sensor.maxValue = 127;
            }

            sensor.sensorInterface = objectServer.add_interface(
                interfacePath, "xyz.openbmc_project.Sensor.Value");
            if (thresholds::hasWarningInterface(sensor.thresholds))
            {
                sensor.thresholdInterfaceWarning = objectServer.add_interface(
                    interfacePath,
                    "xyz.openbmc_project.Sensor.Threshold.Warning");
            }
            if (thresholds::hasCriticalInterface(sensor.thresholds))
            {
                sensor.thresholdInterfaceCritical = objectServer.add_interface(
                    interfacePath,
                    "xyz.openbmc_project.Sensor.Threshold.Critical");
            }
            sensor.association = objectServer.add_interface(
                interfacePath, association::interface);

            sensor.setInitialProperties(conn);
        }
    }

    // call setup always as not all sensors call setInitialProperties
    setupPowerMatch(conn);
    setupRead();
}

CPUSensor::~CPUSensor()
{
    // close the input dev to cancel async operations
    inputDev.close();
    waitTimer.cancel();
    if (show)
    {
        objServer.remove_interface(sensor.thresholdInterfaceWarning);
        objServer.remove_interface(sensor.thresholdInterfaceCritical);
        objServer.remove_interface(sensor.sensorInterface);
        objServer.remove_interface(sensor.association);
    }
}

void CPUSensor::setupRead(void)
{
    if (sensor.readingStateGood())
    {
        inputDev.close();
        int fd = open(path.c_str(), O_RDONLY);
        if (fd >= 0)
        {
            inputDev.assign(fd);

            boost::asio::async_read_until(
                inputDev, readBuf, '\n',
                [this](const boost::system::error_code& ec,
                       std::size_t /*bytes_transfered*/) {
                    this->handleResponse(ec);
                });
        }
        else
        {
            std::cerr << sensor.name << " unable to open fd!\n";
            pollTime = sensorFailedPollTimeMs;
        }
    }
    else
    {
        pollTime = sensorFailedPollTimeMs;
        sensor.markAvailable(false);
    }
    waitTimer.expires_from_now(boost::posix_time::milliseconds(pollTime));
    waitTimer.async_wait([this](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        this->setupRead();
    });
}

void CPUSensor::updateMinMaxValues(void)
{
    const boost::container::flat_map<
        std::string,
        std::vector<std::tuple<const char*, std::reference_wrapper<double>,
                               const char*>>>
        map = {
            {
                "cap",
                {
                    std::make_tuple("cap_max", std::ref(sensor.maxValue),
                                    "MaxValue"),
                    std::make_tuple("cap_min", std::ref(sensor.minValue),
                                    "MinValue"),
                },
            },
        };

    if (auto fileParts = splitFileName(path))
    {
        auto& [fileType, fileNr, fileItem] = *fileParts;
        const auto mapIt = map.find(fileItem);
        if (mapIt != map.cend())
        {
            for (const auto& vectorItem : mapIt->second)
            {
                auto& [suffix, oldValue, dbusName] = vectorItem;
                auto attrPath = boost::replace_all_copy(path, fileItem, suffix);
                if (auto newVal =
                        readFile(attrPath, CPUSensor::sensorScaleFactor))
                {
                    sensor.updateProperty(sensor.sensorInterface, oldValue,
                                          *newVal, dbusName);
                }
                else
                {
                    if (isPowerOn())
                    {
                        sensor.updateProperty(sensor.sensorInterface, oldValue,
                                              0, dbusName);
                    }
                    else
                    {
                        sensor.updateProperty(
                            sensor.sensorInterface, oldValue,
                            std::numeric_limits<double>::quiet_NaN(), dbusName);
                    }
                }
            }
        }
    }
}

void CPUSensor::handleResponse(const boost::system::error_code& err)
{

    if (err == boost::system::errc::bad_file_descriptor)
    {
        return; // we're being destroyed
    }
    if (err == boost::system::errc::operation_canceled)
    {
        if (sensor.readingStateGood())
        {
            if (!loggedInterfaceDown)
            {
                std::cerr << sensor.name << " interface down!\n";
                loggedInterfaceDown = true;
            }
            pollTime = CPUSensor::sensorPollMs * 10u;
            sensor.markFunctional(false);
        }
        return;
    }
    loggedInterfaceDown = false;
    pollTime = CPUSensor::sensorPollMs;
    std::istream responseStream(&readBuf);
    if (!err)
    {
        std::string response;
        try
        {
            std::getline(responseStream, response);
            sensor.rawValue = std::stod(response);
            responseStream.clear();
            double nvalue = sensor.rawValue / CPUSensor::sensorScaleFactor;

            if (show)
            {
                sensor.updateValue(nvalue);
            }
            else
            {
                sensor.value = nvalue;
            }
            if (minMaxReadCounter++ % 8 == 0)
            {
                updateMinMaxValues();
            }
            double gTcontrol = std::numeric_limits<double>::quiet_NaN();
            auto it = gCpuSensors.find(nameTcontrol);
            if (it == gCpuSensors.end())
            {
                gTcontrol = it->second.sensor.value;
            }

            if (gTcontrol != privTcontrol)
            {
                privTcontrol = gTcontrol;

                if (!sensor.thresholds.empty())
                {
                    std::vector<thresholds::Threshold> newThresholds;
                    if (parseThresholdsFromAttr(newThresholds, path,
                                                CPUSensor::sensorScaleFactor,
                                                dtsOffset))
                    {
                        if (!std::equal(sensor.thresholds.begin(),
                                        sensor.thresholds.end(),
                                        newThresholds.begin(),
                                        newThresholds.end()))
                        {
                            sensor.thresholds = newThresholds;
                            if (show)
                            {
                                thresholds::updateThresholds(sensor);
                            }
                        }
                    }
                    else
                    {
                        std::cerr << "Failure to update thresholds for "
                                  << sensor.name << "\n";
                    }
                }
            }
        }
        catch (const std::invalid_argument&)
        {
            sensor.incrementError();
        }
    }
    else
    {
        pollTime = sensorFailedPollTimeMs;
        sensor.incrementError();
    }

    responseStream.clear();
}

void CPUSensor::checkThresholds(void)
{
    if (show)
    {
        thresholds::checkThresholds(sensor);
    }
}
