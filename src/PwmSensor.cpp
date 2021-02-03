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
#include <PwmSensor.hpp>
#include <Utils.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

static constexpr double sysPwmMax = 255.0;
static constexpr double psuPwmMax = 100.0;
static constexpr double defaultPwm = 30.0;
static constexpr double targetIfaceMax = sysPwmMax;

PwmSensor::PwmSensor(const std::string& name, const std::string& sysPath,
                     std::shared_ptr<sdbusplus::asio::connection>& conn,
                     sdbusplus::asio::object_server& objectServer,
                     const std::string& sensorConfiguration,
                     const std::string& sensorType) :
    sysPath(sysPath),
    objectServer(objectServer), name(name)
{
    // add interface under sensor and Control.FanPwm as Control is used
    // in obmc project, also add sensor so it can be viewed as a sensor
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/fan_pwm/" + name,
        "xyz.openbmc_project.Sensor.Value");
    uint32_t pwmValue = getValue(false);
    if (sensorType == "PSU")
    {
        pwmMax = psuPwmMax;
    }
    else
    {
        pwmMax = sysPwmMax;
    }

    if (!pwmValue)
    {
        // default pwm to non 0
        pwmValue = static_cast<uint32_t>(pwmMax * (defaultPwm / 100.0));
        setValue(pwmValue);
    }
    double fValue = 100.0 * (static_cast<double>(pwmValue) / pwmMax);
    sensorInterface->register_property(
        "Value", fValue,
        [this](const double& req, double& resp) {
            if (!std::isfinite(req))
            {
                // Reject attempted change, if to NaN or other non-numeric
                return -1;
            }
            if (req > 100.0 || req < 0.0)
            {
                // TODO(): It does not seem desirable to halt daemon here,
                // probably should just reject the change, continue running?
                throw std::runtime_error("Value out of range");
                return -1;
            }

            double reqValue = (req / 100.0) * pwmMax;
            double respValue = (resp / 100.0) * pwmMax;
            auto reqInt = static_cast<uint32_t>(std::round(reqValue));
            auto respInt = static_cast<uint32_t>(std::round(respValue));
            // Avoid floating-point equality, compare as integers
            if (reqInt == respInt)
            {
                return 1;
            }
            setValue(reqInt);
            resp = req;

            controlInterface->signal_property("Target");

            return 1;
        },
        [this](double& curVal) {
            double currScaled = (curVal / 100.0) * pwmMax;
            auto currInt = static_cast<uint32_t>(std::round(currScaled));
            auto getInt = getValue();
            // Avoid floating-point equality, compare as integers
            if (currInt != getInt)
            {
                double getScaled =
                    100.0 * (static_cast<double>(getInt) / pwmMax);
                curVal = getScaled;
                controlInterface->signal_property("Target");
                sensorInterface->signal_property("Value");
            }
            return curVal;
        });
    // pwm sensor interface is in percent
    sensorInterface->register_property("MaxValue", static_cast<int64_t>(100));
    sensorInterface->register_property("MinValue", static_cast<int64_t>(0));

    controlInterface = objectServer.add_interface(
        "/xyz/openbmc_project/control/fanpwm/" + name,
        "xyz.openbmc_project.Control.FanPwm");
    controlInterface->register_property(
        "Target", static_cast<uint64_t>(pwmValue),
        [this](const uint64_t& req, uint64_t& resp) {
            if (req > static_cast<uint64_t>(targetIfaceMax))
            {
                throw std::runtime_error("Value out of range");
                return -1;
            }
            if (req == resp)
            {
                return 1;
            }
            auto scaledValue = static_cast<double>(req) / targetIfaceMax;
            auto roundValue = std::round(scaledValue * pwmMax);
            setValue(static_cast<uint32_t>(roundValue));
            resp = req;

            sensorInterface->signal_property("Value");

            return 1;
        },
        [this](uint64_t& curVal) {
            auto getInt = getValue();
            auto scaledValue = static_cast<double>(getInt) / pwmMax;
            auto roundValue = std::round(scaledValue * targetIfaceMax);
            auto value = static_cast<uint64_t>(roundValue);
            if (curVal != value)
            {
                curVal = value;
                controlInterface->signal_property("Target");
                sensorInterface->signal_property("Value");
            }
            return curVal;
        });
    sensorInterface->initialize();
    controlInterface->initialize();

    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/fan_pwm/" + name, association::interface);

    // PowerSupply sensors should be associated with chassis board path
    // and inventory along with psu object.
    if (sensorType == "PSU")
    {
        createInventoryAssoc(conn, association, sensorConfiguration);
    }
    else
    {
        createAssociation(association, sensorConfiguration);
    }
}
PwmSensor::~PwmSensor()
{
    objectServer.remove_interface(sensorInterface);
    objectServer.remove_interface(controlInterface);
    objectServer.remove_interface(association);
}

void PwmSensor::setValue(uint32_t value)
{
    std::ofstream ref(sysPath);
    if (!ref.good())
    {
        throw std::runtime_error("Bad Write File");
    }
    ref << value;
}

// on success returns pwm, on failure throws except on initialization, where it
// prints an error and returns 0
uint32_t PwmSensor::getValue(bool errThrow)
{
    std::ifstream ref(sysPath);
    if (!ref.good())
    {
        return -1;
    }
    std::string line;
    if (!std::getline(ref, line))
    {
        return -1;
    }
    try
    {
        uint32_t value = std::stoi(line);
        return value;
    }
    catch (std::invalid_argument&)
    {
        std::cerr << "Error reading pwm at " << sysPath << "\n";
        // throw if not initial read to be caught by dbus bindings
        if (errThrow)
        {
            throw std::runtime_error("Bad Read");
        }
    }
    return 0;
}
