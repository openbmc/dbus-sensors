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

#include <fcntl.h>
#include <sys/ioctl.h>
#include <systemd/sd-journal.h>
#include <unistd.h>

#include <Utils.hpp>
#include <boost/asio/io_context.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cerrno>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

extern "C"
{
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

static constexpr bool debug = false;

static constexpr unsigned int intrusionSensorPollSec = 1;

// SMLink Status Register
const static constexpr size_t pchStatusRegIntrusion = 0x04;

// Status bit field masks
const static constexpr size_t pchRegMaskIntrusion = 0x01;

void ChassisIntrusionSensor::updateValue(const std::string& newValue)
{
    // Take no action if value already equal
    // Same semantics as Sensor::updateValue(const double&)
    if (newValue == mValue)
    {
        return;
    }

    // indicate that it is internal set call
    mInternalSet = true;
    mIface->set_property("Status", newValue);
    mInternalSet = false;

    mValue = newValue;

    if (mOldValue == "Normal" && mValue != "Normal")
    {
        sd_journal_send("MESSAGE=%s", "Chassis intrusion assert event",
                        "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                        "OpenBMC.0.1.ChassisIntrusionDetected", NULL);
        mOldValue = mValue;
    }
    else if (mOldValue != "Normal" && mValue == "Normal")
    {
        sd_journal_send("MESSAGE=%s", "Chassis intrusion de-assert event",
                        "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                        "OpenBMC.0.1.ChassisIntrusionReset", NULL);
        mOldValue = mValue;
    }
}

void ChassisIntrusionPchSensor::readSensor()
{
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    if (ioctl(mBusFd, I2C_SLAVE_FORCE, mSlaveAddr) < 0)
    {
        std::cerr << "Unable to set device address\n";
        return;
    }

    unsigned long funcs = 0;

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    if (ioctl(mBusFd, I2C_FUNCS, &funcs) < 0)
    {
        std::cerr << "Don't support I2C_FUNCS \n";
        return;
    }

    if ((funcs & I2C_FUNC_SMBUS_READ_BYTE_DATA) == 0U)
    {
        std::cerr << "Do not have I2C_FUNC_SMBUS_READ_BYTE_DATA \n";
        return;
    }

    int32_t statusMask = pchRegMaskIntrusion;
    int32_t statusReg = pchStatusRegIntrusion;

    int32_t statusValue = i2c_smbus_read_byte_data(mBusFd, statusReg);
    if constexpr (debug)
    {
        std::cout << "statusValue is " << statusValue << "\n";
    }

    if (statusValue < 0)
    {
        std::cerr << "i2c_smbus_read_byte_data failed \n";
        return;
    }

    // Get status value with mask
    statusValue &= statusMask;

    if constexpr (debug)
    {
        std::cout << "statusValue is " << statusValue << "\n";
    }

    std::string newValue = statusValue != 0 ? "HardwareIntrusion" : "Normal";

    if (mValue != newValue)
    {
        std::cout << "update value from " << mValue << " to " << newValue
                  << "\n";
        updateValue(newValue);
    }
    // trigger next polling
    pollSensorStatus();
}

void ChassisIntrusionPchSensor::pollSensorStatus()
{
    std::weak_ptr<ChassisIntrusionPchSensor> weakRef = weak_from_this();
    // setting a new experation implicitly cancels any pending async wait
    mPollTimer.expires_after(std::chrono::seconds(intrusionSensorPollSec));

    mPollTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        std::shared_ptr<ChassisIntrusionPchSensor> self = weakRef.lock();
        // case of being canceled
        if (ec == boost::asio::error::operation_aborted)
        {
            std::cerr << "Timer of intrusion sensor is cancelled\n";
            return;
        }
        if (self)
        {
            self->readSensor();
        }
        else
        {
            std::cerr << "ChassisIntrusionSensor no self\n";
        }
    });
}

void ChassisIntrusionGpioSensor::readSensor()
{
    mGpioLine.event_read();
    auto value = mGpioLine.get_value();

    // set string defined in chassis redfish schema
    std::string newValue = value != 0 ? "HardwareIntrusion" : "Normal";

    if constexpr (debug)
    {
        std::cout << "\nGPIO value is " << value << "\n";
        std::cout << "Intrusion sensor value is " << newValue << "\n";
    }

    if (mValue != newValue)
    {
        std::cout << "update value from " << mValue << " to " << newValue
                  << "\n";
        updateValue(newValue);
    }
    // trigger next polling
    pollSensorStatus();
}

void ChassisIntrusionGpioSensor::pollSensorStatus()
{
    mGpioFd.async_wait(boost::asio::posix::stream_descriptor::wait_read,
                       [this](const boost::system::error_code& ec) {
        if (ec == boost::system::errc::bad_file_descriptor)
        {
            return; // we're being destroyed
        }
        if (ec)
        {
            std::cerr << "Error on GPIO based intrusion sensor wait event\n";
        }
        else
        {
            readSensor();
        }
    });
}

int ChassisIntrusionSensor::setSensorValue(const std::string& req,
                                           std::string& propertyValue)
{
    if (!mInternalSet)
    {
        propertyValue = req;
        mOverridenState = true;
    }
    else if (!mOverridenState)
    {
        propertyValue = req;
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
    mIface->initialize();
    pollSensorStatus();
}

ChassisIntrusionSensor::ChassisIntrusionSensor(
    IntrusionSensorType type, sdbusplus::asio::object_server& objServer) :
    mValue("unknown"),
    mOldValue("unknown"), mObjServer(objServer), mType(type)
{
    mIface = mObjServer.add_interface("/xyz/openbmc_project/Chassis/Intrusion",
                                      "xyz.openbmc_project.Chassis.Intrusion");
}

ChassisIntrusionPchSensor::ChassisIntrusionPchSensor(
    IntrusionSensorType type, boost::asio::io_context& io,
    sdbusplus::asio::object_server& objServer, int busId, int slaveAddr) :
    ChassisIntrusionSensor(type, objServer),
    mPollTimer(io)
{
    if (busId <= 0 || slaveAddr <= 0)
    {
        throw std::invalid_argument("Invalid i2c bus " + std::to_string(busId) +
                                    " address " + std::to_string(slaveAddr) +
                                    "\n");
    }
    mBusId = busId;
    mSlaveAddr = slaveAddr;
    std::string devPath = "/dev/i2c-" + std::to_string(busId);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    mBusFd = open(devPath.c_str(), O_RDWR | O_CLOEXEC);
    if (mBusFd < 0)
    {
        throw std::invalid_argument("Unable to open " + devPath + "\n");
    }
}

ChassisIntrusionGpioSensor::ChassisIntrusionGpioSensor(
    IntrusionSensorType type, boost::asio::io_context& io,
    sdbusplus::asio::object_server& objServer, bool gpioInverted) :
    ChassisIntrusionSensor(type, objServer),
    mGpioInverted(gpioInverted), mGpioFd(io)
{

    mGpioLine = gpiod::find_line(mPinName);
    if (!mGpioLine)
    {
        throw std::invalid_argument("Error finding gpio pin name: " + mPinName +
                                    "\n");
    }
    try
    {
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
    catch (const std::system_error&)
    {
        std::cerr << "ChassisIntrusionSensor error requesting gpio pin name: "
                  << mPinName << "\n";
    }
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
        std::cerr << "Failed to close fd " << std::to_string(mBusFd);
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
