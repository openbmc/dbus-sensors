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

#include "ChassisIntrusionSensor.hpp"

#include "utils/Utils.hpp"

#include <fcntl.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <sys/syslog.h>
#include <systemd/sd-journal.h>
#include <unistd.h>

#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <gpiod.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

extern "C"
{
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

static constexpr unsigned int defaultPollSec = 1;
static constexpr unsigned int sensorFailedPollSec = 5;
static unsigned int intrusionSensorPollSec = defaultPollSec;
static constexpr const char* hwIntrusionValStr =
    "xyz.openbmc_project.Chassis.Intrusion.Status.HardwareIntrusion";
static constexpr const char* normalValStr =
    "xyz.openbmc_project.Chassis.Intrusion.Status.Normal";
static constexpr const char* manualRearmStr =
    "xyz.openbmc_project.Chassis.Intrusion.RearmMode.Manual";
static constexpr const char* autoRearmStr =
    "xyz.openbmc_project.Chassis.Intrusion.RearmMode.Automatic";

// SMLink Status Register
const static constexpr size_t pchStatusRegIntrusion = 0x04;

// Status bit field masks
const static constexpr size_t pchRegMaskIntrusion = 0x01;

// Value to clear intrusion status hwmon file
const static constexpr size_t intrusionStatusHwmonClearValue = 0;

void ChassisIntrusionSensor::updateValue(const size_t& value)
{
    std::string newValue = value != 0 ? hwIntrusionValStr : normalValStr;

    // Take no action if the hardware status does not change
    // Same semantics as Sensor::updateValue(const double&)
    if (newValue == mValue)
    {
        return;
    }

    lg2::debug("Update value from '{VALUE}' to '{NEWVALUE}'", "VALUE", mValue,
               "NEWVALUE", newValue);

    // Automatic Rearm mode allows direct update
    // Manual Rearm mode requires a rearm action to clear the intrusion
    // status
    if (!mAutoRearm)
    {
        if (newValue == normalValStr)
        {
            // Chassis is first closed from being open. If it has been
            // rearmed externally, reset the flag, update mValue and
            // return, without having to write "Normal" to DBus property
            // (because the rearm action already did).
            // Otherwise, return with no more action.
            if (mRearmFlag)
            {
                mRearmFlag = false;
                mValue = newValue;
            }
            return;
        }
    }

    // Flush the rearm flag everytime it allows an update to Dbus
    mRearmFlag = false;

    // indicate that it is internal set call
    mOverridenState = false;
    mInternalSet = true;
    mIface->set_property("Status", newValue);
    mInternalSet = false;

    mValue = newValue;
}

int ChassisIntrusionPchSensor::readSensor()
{
    int32_t statusMask = pchRegMaskIntrusion;
    int32_t statusReg = pchStatusRegIntrusion;

    int32_t value = i2c_smbus_read_byte_data(mBusFd, statusReg);
    lg2::debug("Pch type: raw value is '{VALUE}'", "VALUE", value);

    if (value < 0)
    {
        lg2::error("i2c_smbus_read_byte_data failed");
        return -1;
    }

    // Get status value with mask
    value &= statusMask;

    lg2::debug("Pch type: masked raw value is '{VALUE}'", "VALUE", value);
    return value;
}

void ChassisIntrusionPchSensor::pollSensorStatus()
{
    std::weak_ptr<ChassisIntrusionPchSensor> weakRef = weak_from_this();

    // setting a new experation implicitly cancels any pending async wait
    mPollTimer.expires_after(std::chrono::seconds(intrusionSensorPollSec));

    mPollTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        // case of being canceled
        if (ec == boost::asio::error::operation_aborted)
        {
            lg2::error("Timer of intrusion sensor is cancelled");
            return;
        }

        std::shared_ptr<ChassisIntrusionPchSensor> self = weakRef.lock();
        if (!self)
        {
            lg2::error("ChassisIntrusionSensor no self");
            return;
        }

        int value = self->readSensor();
        if (value < 0)
        {
            intrusionSensorPollSec = sensorFailedPollSec;
        }
        else
        {
            intrusionSensorPollSec = defaultPollSec;
            self->updateValue(value);
        }

        // trigger next polling
        self->pollSensorStatus();
    });
}

int ChassisIntrusionGpioSensor::readSensor()
{
    mGpioLine.event_read();
    auto value = mGpioLine.get_value();
    lg2::debug("Gpio type: raw value is '{VALUE}'", "VALUE", value);
    return value;
}

void ChassisIntrusionGpioSensor::pollSensorStatus()
{
    mGpioFd.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [this](const boost::system::error_code& ec) {
            if (ec == boost::system::errc::bad_file_descriptor)
            {
                return; // we're being destroyed
            }

            if (ec)
            {
                lg2::error("Error on GPIO based intrusion sensor wait event");
            }
            else
            {
                int value = readSensor();
                if (value >= 0)
                {
                    updateValue(value);
                }
                // trigger next polling
                pollSensorStatus();
            }
        });
}

int ChassisIntrusionHwmonSensor::readSensor()
{
    int value = 0;

    std::fstream stream(mHwmonPath, std::ios::in | std::ios::out);
    if (!stream.good())
    {
        lg2::error("Error reading status at '{PATH}'", "PATH", mHwmonPath);
        return -1;
    }

    std::string line;
    if (!std::getline(stream, line))
    {
        lg2::error("Error reading status at '{PATH}'", "PATH", mHwmonPath);
        return -1;
    }

    try
    {
        value = std::stoi(line);
        lg2::debug("Hwmon type: raw value is '{VALUE}'", "VALUE", value);
    }
    catch (const std::invalid_argument& e)
    {
        lg2::error("Error reading status at '{PATH}': '{ERR}'", "PATH",
                   mHwmonPath, "ERR", e);
        return -1;
    }

    // Reset chassis intrusion status after every reading
    stream << intrusionStatusHwmonClearValue;

    return value;
}

void ChassisIntrusionHwmonSensor::pollSensorStatus()
{
    std::weak_ptr<ChassisIntrusionHwmonSensor> weakRef = weak_from_this();

    // setting a new experation implicitly cancels any pending async wait
    mPollTimer.expires_after(std::chrono::seconds(intrusionSensorPollSec));

    mPollTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        // case of being canceled
        if (ec == boost::asio::error::operation_aborted)
        {
            lg2::error("Timer of intrusion sensor is cancelled");
            return;
        }

        std::shared_ptr<ChassisIntrusionHwmonSensor> self = weakRef.lock();
        if (!self)
        {
            lg2::error("ChassisIntrusionSensor no self");
            return;
        }

        int value = self->readSensor();
        if (value < 0)
        {
            intrusionSensorPollSec = sensorFailedPollSec;
        }
        else
        {
            intrusionSensorPollSec = defaultPollSec;
            self->updateValue(value);
        }

        // trigger next polling
        self->pollSensorStatus();
    });
}

int ChassisIntrusionSensor::setSensorValue(const std::string& req,
                                           std::string& propertyValue)
{
    if (!mInternalSet)
    {
        /*
           1. Assuming that setting property in Automatic mode causes
           no effect but only event logs and propertiesChanged signal
           (because the property will be updated continuously to the
           current hardware status anyway), only update Status property
           and raise rearm flag in Manual rearm mode.
           2. Only accept Normal value from an external call.
        */
        if (!mAutoRearm && req == normalValStr)
        {
            mRearmFlag = true;
            propertyValue = req;
            mOverridenState = true;
        }
    }
    else if (!mOverridenState)
    {
        propertyValue = req;
    }
    else
    {
        return 1;
    }
    // Send intrusion event to Redfish
    if (mValue == normalValStr && propertyValue != normalValStr)
    {
        sd_journal_send("MESSAGE=%s", "Chassis intrusion assert event",
                        "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                        "OpenBMC.0.1.ChassisIntrusionDetected", NULL);
    }
    else if (mValue == hwIntrusionValStr && propertyValue == normalValStr)
    {
        sd_journal_send("MESSAGE=%s", "Chassis intrusion de-assert event",
                        "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                        "OpenBMC.0.1.ChassisIntrusionReset", NULL);
    }
    return 1;
}

void ChassisIntrusionSensor::start()
{
    mIface->register_property(
        "Status", mValue,
        [&](const std::string& req, std::string& propertyValue) {
            return setSensorValue(req, propertyValue);
        });
    std::string rearmStr = mAutoRearm ? autoRearmStr : manualRearmStr;
    mIface->register_property("Rearm", rearmStr);
    mIface->initialize();
    pollSensorStatus();
}

ChassisIntrusionSensor::ChassisIntrusionSensor(
    bool autoRearm, sdbusplus::asio::object_server& objServer) :
    mValue(normalValStr), mAutoRearm(autoRearm), mObjServer(objServer)
{
    mIface = mObjServer.add_interface("/xyz/openbmc_project/Chassis/Intrusion",
                                      "xyz.openbmc_project.Chassis.Intrusion");
}

ChassisIntrusionPchSensor::ChassisIntrusionPchSensor(
    bool autoRearm, boost::asio::io_context& io,
    sdbusplus::asio::object_server& objServer, int busId, int slaveAddr) :
    ChassisIntrusionSensor(autoRearm, objServer), mPollTimer(io)
{
    if (busId < 0 || slaveAddr <= 0)
    {
        throw std::invalid_argument(
            "Invalid i2c bus " + std::to_string(busId) + " address " +
            std::to_string(slaveAddr) + "\n");
    }

    mSlaveAddr = slaveAddr;

    std::string devPath = "/dev/i2c-" + std::to_string(busId);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    mBusFd = open(devPath.c_str(), O_RDWR | O_CLOEXEC);
    if (mBusFd < 0)
    {
        throw std::invalid_argument("Unable to open " + devPath + "\n");
    }

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    if (ioctl(mBusFd, I2C_SLAVE_FORCE, mSlaveAddr) < 0)
    {
        throw std::runtime_error("Unable to set device address\n");
    }

    unsigned long funcs = 0;

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    if (ioctl(mBusFd, I2C_FUNCS, &funcs) < 0)
    {
        throw std::runtime_error("Don't support I2C_FUNCS\n");
    }

    if ((funcs & I2C_FUNC_SMBUS_READ_BYTE_DATA) == 0U)
    {
        throw std::runtime_error(
            "Do not have I2C_FUNC_SMBUS_READ_BYTE_DATA \n");
    }
}

ChassisIntrusionGpioSensor::ChassisIntrusionGpioSensor(
    bool autoRearm, boost::asio::io_context& io,
    sdbusplus::asio::object_server& objServer, bool gpioInverted) :
    ChassisIntrusionSensor(autoRearm, objServer), mGpioInverted(gpioInverted),
    mGpioFd(io)
{
    mGpioLine = gpiod::find_line(mPinName);
    if (!mGpioLine)
    {
        throw std::invalid_argument(
            "Error finding gpio pin name: " + mPinName + "\n");
    }
    mGpioLine.request(
        {"ChassisIntrusionSensor", gpiod::line_request::EVENT_BOTH_EDGES,
         mGpioInverted ? gpiod::line_request::FLAG_ACTIVE_LOW : 0});

    auto gpioLineFd = mGpioLine.event_get_fd();
    if (gpioLineFd < 0)
    {
        throw std::invalid_argument("Failed to get " + mPinName + " fd\n");
    }

    mGpioFd.assign(gpioLineFd);
}

ChassisIntrusionHwmonSensor::ChassisIntrusionHwmonSensor(
    bool autoRearm, boost::asio::io_context& io,
    sdbusplus::asio::object_server& objServer, std::string hwmonName) :
    ChassisIntrusionSensor(autoRearm, objServer),
    mHwmonName(std::move(hwmonName)), mPollTimer(io)
{
    std::vector<std::filesystem::path> paths;

    if (!findFiles(std::filesystem::path("/sys/class/hwmon"), mHwmonName,
                   paths))
    {
        throw std::invalid_argument("Failed to find hwmon path in sysfs\n");
    }

    if (paths.empty())
    {
        throw std::invalid_argument(
            "Hwmon file " + mHwmonName + " can't be found in sysfs\n");
    }

    if (paths.size() > 1)
    {
        lg2::error("Found more than 1 hwmon file to read chassis intrusion"
                   " status. Taking the first one.");
    }

    // Expecting only one hwmon file for one given chassis
    mHwmonPath = paths[0].string();

    lg2::debug(
        "Found '{NUM_PATHS}' paths for intrusion status. The first path is: '{PATH}'",
        "NUM_PATHS", paths.size(), "PATH", mHwmonPath);
}

ChassisIntrusionSensor::~ChassisIntrusionSensor()
{
    mObjServer.remove_interface(mIface);
}

ChassisIntrusionPchSensor::~ChassisIntrusionPchSensor()
{
    mPollTimer.cancel();
    if (close(mBusFd) < 0)
    {
        lg2::error("Failed to close fd '{FD}'", "FD", mBusFd);
    }
}

ChassisIntrusionGpioSensor::~ChassisIntrusionGpioSensor()
{
    mGpioFd.close();
    if (mGpioLine)
    {
        mGpioLine.release();
    }
}

ChassisIntrusionHwmonSensor::~ChassisIntrusionHwmonSensor()
{
    mPollTimer.cancel();
}
