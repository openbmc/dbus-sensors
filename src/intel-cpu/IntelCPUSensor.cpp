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

#include "IntelCPUSensor.hpp"

#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/descriptor_base.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

IntelCPUSensor::IntelCPUSensor(
    const std::string& path, const std::string& objectType,
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const std::string& sensorName,
    std::vector<thresholds::Threshold>&& thresholdsIn,
    const std::string& sensorConfiguration, int cpuId, bool show,
    double dtsOffset) :
    Sensor(escapeName(sensorName), std::move(thresholdsIn), sensorConfiguration,
           objectType, false, false, 0, 0, conn, PowerState::on),
    objServer(objectServer), inputDev(io), waitTimer(io),
    nameTcontrol("Tcontrol CPU" + std::to_string(cpuId)), path(path),
    privTcontrol(std::numeric_limits<double>::quiet_NaN()),
    dtsOffset(dtsOffset), show(show)
{
    if (show)
    {
        if (auto fileParts = splitFileName(path))
        {
            auto& [type, nr, item] = *fileParts;
            std::string interfacePath;
            const char* units = nullptr;
            if (type == "power")
            {
                interfacePath = "/xyz/openbmc_project/sensors/power/" + name;
                units = sensor_paths::unitWatts;
                minValue = 0;
                maxValue = 511;
            }
            else
            {
                interfacePath = "/xyz/openbmc_project/sensors/temperature/" +
                                name;
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

            setInitialProperties(units);
        }
    }

    // call setup always as not all sensors call setInitialProperties
    setupPowerMatch(conn);
}

IntelCPUSensor::~IntelCPUSensor()
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

void IntelCPUSensor::restartRead()
{
    std::weak_ptr<IntelCPUSensor> weakRef = weak_from_this();
    waitTimer.expires_after(std::chrono::milliseconds(pollTime));
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            lg2::error("Failed to reschedule");
            return;
        }
        std::shared_ptr<IntelCPUSensor> self = weakRef.lock();

        if (self)
        {
            self->setupRead();
        }
    });
}

void IntelCPUSensor::setupRead()
{
    if (readingStateGood())
    {
        inputDev.close();

        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
        fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0)
        {
            lg2::error("'{NAME}' unable to open fd!", "NAME", name);
            return;
        }

        inputDev.assign(fd);
    }
    else
    {
        markAvailable(false);
        if (show)
        {
            updateValue(std::numeric_limits<double>::quiet_NaN());
        }
        else
        {
            value = std::numeric_limits<double>::quiet_NaN();
        }
        restartRead();
        return;
    }

    std::weak_ptr<IntelCPUSensor> weakRef = weak_from_this();
    inputDev.async_wait(boost::asio::posix::descriptor_base::wait_read,
                        [weakRef](const boost::system::error_code& ec) {
                            std::shared_ptr<IntelCPUSensor> self =
                                weakRef.lock();

                            if (self)
                            {
                                self->handleResponse(ec);
                            }
                        });
}

void IntelCPUSensor::updateMinMaxValues()
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
                const auto& [suffix, oldValue, dbusName] = vectorItem;
                auto attrPath = boost::replace_all_copy(path, fileItem, suffix);
                if (auto newVal =
                        readFile(attrPath, IntelCPUSensor::sensorScaleFactor))
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

void IntelCPUSensor::handleResponse(const boost::system::error_code& err)
{
    if ((err == boost::system::errc::bad_file_descriptor) ||
        (err == boost::asio::error::misc_errors::not_found))
    {
        return; // we're being destroyed
    }
    if (err == boost::system::errc::operation_canceled)
    {
        if (readingStateGood())
        {
            if (!loggedInterfaceDown)
            {
                lg2::error("'{NAME}' interface down!", "NAME", name);
                loggedInterfaceDown = true;
            }
            pollTime = static_cast<size_t>(IntelCPUSensor::sensorPollMs) * 10U;
            markFunctional(false);
        }
        return;
    }
    loggedInterfaceDown = false;

    if (err)
    {
        pollTime = sensorFailedPollTimeMs;
        incrementError();
        return;
    }

    static constexpr uint32_t bufLen = 128;
    std::string response;
    response.resize(bufLen);
    int rdLen = 0;

    if (fd >= 0)
    {
        rdLen = pread(fd, response.data(), bufLen, 0);
    }

    if (rdLen > 0)
    {
        try
        {
            rawValue = std::stod(response);
            double nvalue = rawValue / IntelCPUSensor::sensorScaleFactor;

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
                    if (parseThresholdsFromAttr(
                            newThresholds, path,
                            IntelCPUSensor::sensorScaleFactor, dtsOffset, 0))
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
                        privTcontrol = std::numeric_limits<double>::quiet_NaN();
                        lg2::error("Failure to update thresholds for '{NAME}'",
                                   "NAME", name);
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
    restartRead();
}

void IntelCPUSensor::checkThresholds()
{
    if (show)
    {
        thresholds::checkThresholds(this);
    }
}
