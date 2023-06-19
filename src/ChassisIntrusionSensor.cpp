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

static constexpr unsigned int defaultPollSec = 1;
static constexpr unsigned int sensorFailedPollSec = 5;
static unsigned int intrusionSensorPollSec = defaultPollSec;
static constexpr const char* hwIntrusionValStr = "HardwareIntrusion";
static constexpr const char* normalValStr = "Normal";

// SMLink Status Register
const static constexpr size_t pchStatusRegIntrusion = 0x04;

// Status bit field masks
const static constexpr size_t pchRegMaskIntrusion = 0x01;

// Value to clear intrusion status hwmon file
const static constexpr size_t intrusionStatusHwmonClearValue = 0;

void ChassisIntrusionSensor::updateValue(const size_t& value)
{
    std::string newValue = value != 0 ? hwIntrusionValStr : normalValStr;

    // Take no action if value already equal
    // Same semantics as Sensor::updateValue(const double&)
    if (newValue == mValue)
    {
        return;
    }

    if constexpr (debug)
    {
        std::cout << "Update value from " << mValue << " to " << newValue
                  << "\n";
    }

    // indicate that it is internal set call
    mInternalSet = true;
    mIface->set_property("Status", newValue);
    mInternalSet = false;

    mValue = newValue;

    if (mOldValue == normalValStr && mValue != normalValStr)
    {
        sd_journal_send("MESSAGE=%s", "Chassis intrusion assert event",
                        "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                        "OpenBMC.0.1.ChassisIntrusionDetected", NULL);
        mOldValue = mValue;
    }
    else if (mOldValue == hwIntrusionValStr && mValue == normalValStr)
    {
        sd_journal_send("MESSAGE=%s", "Chassis intrusion de-assert event",
                        "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                        "OpenBMC.0.1.ChassisIntrusionReset", NULL);
        mOldValue = mValue;
    }
}

int ChassisIntrusionPchSensor::readSensor()
{
    int32_t statusMask = pchRegMaskIntrusion;
    int32_t statusReg = pchStatusRegIntrusion;

    int32_t value = i2c_smbus_read_byte_data(mBusFd, statusReg);
    if constexpr (debug)
    {
        std::cout << "Pch type: raw value is " << value << "\n";
    }

    if (value < 0)
    {
        std::cerr << "i2c_smbus_read_byte_data failed \n";
        return -1;
    }

    // Get status value with mask
    value &= statusMask;

    if constexpr (debug)
    {
        std::cout << "Pch type: masked raw value is " << value << "\n";
    }
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
            std::cerr << "Timer of intrusion sensor is cancelled\n";
            return;
        }

        std::shared_ptr<ChassisIntrusionPchSensor> self = weakRef.lock();
        if (!self)
        {
            std::cerr << "ChassisIntrusionSensor no self\n";
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
    if constexpr (debug)
    {
        std::cout << "Gpio type: raw value is " << value << "\n";
    }
    return value;
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
        std::cerr << "Error reading status at " << mHwmonPath << "\n";
        return -1;
    }

    std::string line;
    if (!std::getline(stream, line))
    {
        std::cerr << "Error reading status at " << mHwmonPath << "\n";
        return -1;
    }

    try
    {
        value = std::stoi(line);
        if constexpr (debug)
        {
            std::cout << "Hwmon type: raw value is " << value << "\n";
        }
    }
    catch (const std::invalid_argument& e)
    {
        std::cerr << "Error reading status at " << mHwmonPath << " : "
                  << e.what() << "\n";
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
            std::cerr << "Timer of intrusion sensor is cancelled\n";
            return;
        }

        std::shared_ptr<ChassisIntrusionHwmonSensor> self = weakRef.lock();
        if (!self)
        {
            std::cerr << "ChassisIntrusionSensor no self\n";
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
    sdbusplus::asio::object_server& objServer) :
    mObjServer(objServer)
{
    mIface = mObjServer.add_interface("/xyz/openbmc_project/Chassis/Intrusion",
                                      "xyz.openbmc_project.Chassis.Intrusion");
}

ChassisIntrusionPchSensor::ChassisIntrusionPchSensor(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objServer,
    int busId, int slaveAddr) :
    ChassisIntrusionSensor(objServer),
    mPollTimer(io)
{
    if (busId < 0 || slaveAddr <= 0)
    {
        throw std::invalid_argument("Invalid i2c bus " + std::to_string(busId) +
                                    " address " + std::to_string(slaveAddr) +
                                    "\n");
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
    boost::asio::io_context& io, sdbusplus::asio::object_server& objServer,
    bool gpioInverted) :
    ChassisIntrusionSensor(objServer),
    mGpioInverted(gpioInverted), mGpioFd(io)
{
    mGpioLine = gpiod::find_line(mPinName);
    if (!mGpioLine)
    {
        throw std::invalid_argument("Error finding gpio pin name: " + mPinName +
                                    "\n");
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
    boost::asio::io_context& io, sdbusplus::asio::object_server& objServer,
    std::string hwmonName) :
    ChassisIntrusionSensor(objServer),
    mHwmonName(std::move(hwmonName)), mPollTimer(io)
{
    std::vector<fs::path> paths;

    if (!findFiles(fs::path("/sys/class/hwmon"), mHwmonName, paths))
    {
        throw std::invalid_argument("Failed to find hwmon path in sysfs\n");
    }

    if (paths.empty())
    {
        throw std::invalid_argument("Hwmon file " + mHwmonName +
                                    " can't be found in sysfs\n");
    }

    if (paths.size() > 1)
    {
        std::cerr << "Found more than 1 hwmon file to read chassis intrusion"
                  << " status. Taking the first one. \n";
    }

    // Expecting only one hwmon file for one given chassis
    mHwmonPath = paths[0].string();

    if constexpr (debug)
    {
        std::cout << "Found " << paths.size()
                  << " paths for intrusion status \n"
                  << " The first path is: " << mHwmonPath << "\n";
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

ChassisIntrusionHwmonSensor::~ChassisIntrusionHwmonSensor()
{
    mPollTimer.cancel();
}
