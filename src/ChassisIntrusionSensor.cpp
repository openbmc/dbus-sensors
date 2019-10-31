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

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <chrono>
#include <iostream>
#include <sdbusplus/asio/object_server.hpp>
#include <string>
#include <thread>

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

static constexpr bool DEBUG = false;

static constexpr unsigned int intrusionSensorPollSec = 1;

// SMLink Status Register
const static constexpr size_t pchStatusRegIntrusion = 0x04;

// Status bit field masks
const static constexpr size_t pchRegMaskIntrusion = 0x01;

// gpio sysfs path
constexpr const char* gpioPath = "/sys/class/gpio/";

void ChassisIntrusionSensor::updateValue(const std::string newValue)
{
    // indicate that it is internal set call
    mInternalSet = true;
    mIface->set_property("Status", newValue);
    mInternalSet = false;

    mValue = newValue;

    if (mOldValue == "Normal" && mValue != "Normal")
    {
        std::cerr << "save to SEL for intrusion assert event \n";
        // TODO: call add SEL log API, depends on patch #13956
        mOldValue = mValue;
    }
    else if (mOldValue != "Normal" && mValue == "Normal")
    {
        std::cerr << "save to SEL for intrusion de-assert event \n";
        // TODO: call add SEL log API, depends on patch #13956
        mOldValue = mValue;
    }
}

int ChassisIntrusionSensor::i2cReadFromPch(int busId, int slaveAddr)
{
    std::string i2cBus = "/dev/i2c-" + std::to_string(busId);

    int fd = open(i2cBus.c_str(), O_RDWR | O_CLOEXEC);
    if (fd < 0)
    {
        std::cerr << "unable to open i2c device \n";
        return -1;
    }
    if (ioctl(fd, I2C_SLAVE_FORCE, slaveAddr) < 0)
    {
        std::cerr << "unable to set device address\n";
        close(fd);
        return -1;
    }

    unsigned long funcs = 0;
    if (ioctl(fd, I2C_FUNCS, &funcs) < 0)
    {
        std::cerr << "not support I2C_FUNCS \n";
        close(fd);
        return -1;
    }

    if (!(funcs & I2C_FUNC_SMBUS_READ_BYTE_DATA))
    {
        std::cerr << "not support I2C_FUNC_SMBUS_READ_BYTE_DATA \n";
        close(fd);
        return -1;
    }

    int statusValue;
    unsigned int statusMask = pchRegMaskIntrusion;
    unsigned int statusReg = pchStatusRegIntrusion;

    statusValue = i2c_smbus_read_byte_data(fd, statusReg);
    if (DEBUG)
    {
        std::cout << "\nRead bus " << busId << " addr " << slaveAddr
                  << ", value = " << statusValue << "\n";
    }

    close(fd);

    if (statusValue < 0)
    {
        std::cerr << "i2c_smbus_read_byte_data failed \n";
        return -1;
    }

    // Get status value with mask
    int newValue = statusValue & statusMask;

    if (DEBUG)
    {
        std::cout << "statusValue is " << statusValue << "\n";
        std::cout << "Intrusion sensor value is " << newValue << "\n";
    }

    return newValue;
}

void ChassisIntrusionSensor::pollSensorStatusByPch()
{
    // setting a new experation implicitly cancels any pending async wait
    mPollTimer.expires_from_now(
        boost::posix_time::seconds(intrusionSensorPollSec));

    mPollTimer.async_wait([&](const boost::system::error_code& ec) {
        // case of timer expired
        if (!ec)
        {
            int statusValue = i2cReadFromPch(mBusId, mSlaveAddr);
            std::string newValue = statusValue ? "HardwareIntrusion" : "Normal";

            if (newValue != "unknown" && mValue != newValue)
            {
                std::cout << "update value from " << mValue << " to "
                          << newValue << "\n";
                updateValue(newValue);
            }

            // trigger next polling
            pollSensorStatusByPch();
        }
        // case of being canceled
        else if (ec == boost::asio::error::operation_aborted)
        {
            std::cerr << "Timer of intrusion sensor is cancelled. Return \n";
            return;
        }
    });
}

void ChassisIntrusionSensor::readGpio()
{
    constexpr size_t readSize = sizeof("0");
    std::string readBuf;
    readBuf.resize(readSize);
    lseek(mFd, 0, SEEK_SET);
    size_t r = ::read(mFd, readBuf.data(), readSize);
    if (r != readSize)
    {
        std::cerr << "Error reading gpio\n";
    }
    else
    {
        bool value = std::stoi(readBuf);
        if (mGpioInverted)
        {
            value = !value;
        }

        // set string defined in chassis redfish schema
        std::string newValue = value ? "HardwareIntrusion" : "Normal";

        if (DEBUG)
        {
            std::cout << "\nGPIO value is " << value << "\n";
            std::cout << "Intrusion sensor value is " << newValue << "\n";
        }

        if (newValue != "unknown" && mValue != newValue)
        {
            std::cout << "update value from " << mValue << " to " << newValue
                      << "\n";
            updateValue(newValue);
        }
    }
}

void ChassisIntrusionSensor::pollSensorStatusByGpio(void)
{
    mInputDev.async_wait(
        boost::asio::ip::tcp::socket::wait_error,
        [this](const boost::system::error_code& ec) {
            if (ec == boost::system::errc::bad_file_descriptor)
            {
                return; // we're being destroyed
            }
            else if (ec)
            {
                std::cerr << "Error on GPIO based intrusion sensor socket\n";
            }
            else
            {
                readGpio();
            }
            pollSensorStatusByGpio();
        });
}

void ChassisIntrusionSensor::initGpioDeviceFile(const int index)
{
    std::string device = gpioPath + std::string("gpio") + std::to_string(index);
    mFd = open((device + "/value").c_str(), O_RDONLY);
    if (mFd < 0)
    {
        std::cerr << "Error opening gpio " << index << "\n";
        return;
    }
    mInputDev.assign(boost::asio::ip::tcp::v4(), mFd);
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

void ChassisIntrusionSensor::start(IntrusionSensorType type, int busId,
                                   int slaveAddr, int gpioIndex,
                                   bool gpioInverted)
{
    if (DEBUG)
    {
        std::cerr << "enter ChassisIntrusionSensor::start, type = " << type
                  << "\n";
        if (type == IntrusionSensorType::pch)
        {
            std::cerr << "busId = " << busId << ", slaveAddr = " << slaveAddr
                      << "\n";
        }
        else if (type == IntrusionSensorType::gpio)
        {
            std::cerr << "gpioIndex = " << gpioIndex
                      << ", gpioInverted = " << gpioInverted << "\n";
        }
    }

    if ((type == IntrusionSensorType::pch && busId == mBusId &&
         slaveAddr == mSlaveAddr) ||
        (type == IntrusionSensorType::gpio && gpioIndex == mGpioIndex &&
         gpioInverted == mGpioInverted))
    {
        return;
    }

    mType = type;
    mBusId = busId;
    mSlaveAddr = slaveAddr;
    mGpioIndex = gpioIndex;
    mGpioInverted = gpioInverted;

    if ((mType == IntrusionSensorType::pch && mBusId > 0 && mSlaveAddr > 0) ||
        (mType == IntrusionSensorType::gpio && mGpioIndex > 0))
    {
        // initialize first if not initialized before
        if (!mInitialized)
        {
            mIface->register_property(
                "Status", mValue,
                [&](const std::string& req, std::string& propertyValue) {
                    return setSensorValue(req, propertyValue);
                });
            mIface->initialize();

            if (mType == IntrusionSensorType::gpio)
            {
                initGpioDeviceFile(mGpioIndex);
            }

            mInitialized = true;
        }

        // start polling value
        if (mType == IntrusionSensorType::pch)
        {
            pollSensorStatusByPch();
        }
        else if (mType == IntrusionSensorType::gpio && mFd > 0)
        {
            pollSensorStatusByGpio();
        }
    }

    // invalid para, release resource
    else
    {
        if (mInitialized)
        {
            if (mType == IntrusionSensorType::pch)
            {
                mPollTimer.cancel();
            }
            else if (mType == IntrusionSensorType::gpio)
            {
                mInputDev.close();
                close(mFd);
            }
            mInitialized = false;
        }
    }
}

ChassisIntrusionSensor::ChassisIntrusionSensor(
    boost::asio::io_service& io,
    std::shared_ptr<sdbusplus::asio::dbus_interface> iface) :
    mPollTimer(io),
    mIface(iface), mInputDev(io), mType(IntrusionSensorType::gpio), mBusId(-1),
    mSlaveAddr(-1), mGpioIndex(-1), mGpioInverted(false), mValue("unknown"),
    mOldValue("unknown")
{
}

ChassisIntrusionSensor::~ChassisIntrusionSensor()
{
    if (mType == IntrusionSensorType::pch)
    {
        mPollTimer.cancel();
    }
    else if (mType == IntrusionSensorType::gpio)
    {
        mInputDev.close();
        close(mFd);
    }
}
