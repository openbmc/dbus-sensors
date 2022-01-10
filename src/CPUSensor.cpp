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
                     std::vector<thresholds::Threshold>&& thresholdsIn,
                     const std::string& sensorConfiguration, int cpuId,
                     bool show, double dtsOffset) :
    Sensor(escapeName(sensorName), std::move(thresholdsIn), sensorConfiguration,
           objectType, false, false, 0, 0, conn, PowerState::on),
    std::enable_shared_from_this<CPUSensor>(), objServer(objectServer),
    inputDev(io), waitTimer(io), path(path),
    privTcontrol(std::numeric_limits<double>::quiet_NaN()),
    dtsOffset(dtsOffset), show(show), pollTime(CPUSensor::sensorPollMs),
    minMaxReadCounter(0)
{
    nameTcontrol = labelTcontrol;
    nameTcontrol += " CPU" + std::to_string(cpuId);
    if (show)
    {
        if (auto fileParts = splitFileName(path))
        {
            auto& [type, nr, item] = *fileParts;
            std::string interfacePath;
            const char* units;
            if (type.compare("power") == 0)
            {
                interfacePath = "/xyz/openbmc_project/sensors/power/" + name;
                units = sensor_paths::unitWatts;
                minValue = 0;
                maxValue = 511;
            }
            else
            {
                interfacePath =
                    "/xyz/openbmc_project/sensors/temperature/" + name;
                units = sensor_paths::unitDegreesC;
                minValue = -128;
                maxValue = 127;
            }

            sensorInterface = objectServer.add_interface(
                interfacePath, "xyz.openbmc_project.Sensor.Value");
            for (const auto& threshold : thresholds)
            {
                std::string interface =
                    thresholds::getInterface(threshold.level);
                thresholdInterfaces[static_cast<size_t>(threshold.level)] =
                    objectServer.add_interface(interfacePath, interface);
            }
            association = objectServer.add_interface(interfacePath,
                                                     association::interface);

            setInitialProperties(conn, units);
        }
    }

    // call setup always as not all sensors call setInitialProperties
    setupPowerMatch(conn);
}

CPUSensor::~CPUSensor()
{
    // close the input dev to cancel async operations
    inputDev.close();
    waitTimer.cancel();
    if (show)
    {
        for (const auto& iface : thresholdInterfaces)
        {
            objServer.remove_interface(iface);
        }
        objServer.remove_interface(sensorInterface);
        objServer.remove_interface(association);
        objServer.remove_interface(availableInterface);
        objServer.remove_interface(operationalInterface);
    }
}

void CPUSensor::setupRead(void)
{
    std::weak_ptr<CPUSensor> weakRef = weak_from_this();

    if (readingStateGood())
    {
        inputDev.close();
        int fd = open(path.c_str(), O_RDONLY);
        if (fd >= 0)
        {
            inputDev.assign(fd);

            boost::asio::async_read_until(
                inputDev, readBuf, '\n',
                [weakRef](const boost::system::error_code& ec,
                          std::size_t /*bytes_transfered*/) {
                    std::shared_ptr<CPUSensor> self = weakRef.lock();
                    if (!self)
                    {
                        return;
                    }
                    self->handleResponse(ec);
                });
        }
        else
        {
            std::cerr << name << " unable to open fd!\n";
            pollTime = sensorFailedPollTimeMs;
        }
    }
    else
    {
        pollTime = sensorFailedPollTimeMs;
        markAvailable(false);
    }
    waitTimer.expires_from_now(boost::posix_time::milliseconds(pollTime));
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        std::shared_ptr<CPUSensor> self = weakRef.lock();
        if (!self)
        {
            return;
        }
        self->setupRead();
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
                    std::make_tuple("cap_max", std::ref(maxValue), "MaxValue"),
                    std::make_tuple("cap_min", std::ref(minValue), "MinValue"),
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
                    updateProperty(sensorInterface, oldValue, *newVal,
                                   dbusName);
                }
                else
                {
                    if (isPowerOn())
                    {
                        updateProperty(sensorInterface, oldValue, 0, dbusName);
                    }
                    else
                    {
                        updateProperty(sensorInterface, oldValue,
                                       std::numeric_limits<double>::quiet_NaN(),
                                       dbusName);
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
        if (readingStateGood())
        {
            if (!loggedInterfaceDown)
            {
                std::cerr << name << " interface down!\n";
                loggedInterfaceDown = true;
            }
            pollTime = CPUSensor::sensorPollMs * 10u;
            markFunctional(false);
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
            rawValue = std::stod(response);
            responseStream.clear();
            double nvalue = rawValue / CPUSensor::sensorScaleFactor;

            if (show)
            {
                updateValue(nvalue);
            }
            else
            {
                value = nvalue;
            }
            if (minMaxReadCounter++ % 8 == 0)
            {
                updateMinMaxValues();
            }

            double gTcontrol = gCpuSensors[nameTcontrol]
                                   ? gCpuSensors[nameTcontrol]->value
                                   : std::numeric_limits<double>::quiet_NaN();
            if (gTcontrol != privTcontrol)
            {
                privTcontrol = gTcontrol;

                if (!thresholds.empty())
                {
                    std::vector<thresholds::Threshold> newThresholds;
                    if (parseThresholdsFromAttr(newThresholds, path,
                                                CPUSensor::sensorScaleFactor,
                                                dtsOffset))
                    {
                        if (!std::equal(thresholds.begin(), thresholds.end(),
                                        newThresholds.begin(),
                                        newThresholds.end()))
                        {
                            thresholds = newThresholds;
                            if (show)
                            {
                                thresholds::updateThresholds(this);
                            }
                        }
                    }
                    else
                    {
                        std::cerr << "Failure to update thresholds for " << name
                                  << "\n";
                    }
                }
            }
        }
        catch (const std::invalid_argument&)
        {
            incrementError();
        }
    }
    else
    {
        pollTime = sensorFailedPollTimeMs;
        incrementError();
    }

    responseStream.clear();
}

void CPUSensor::checkThresholds(void)
{
    if (show)
    {
        thresholds::checkThresholds(this);
    }
}
