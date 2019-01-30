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

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <ChassisIntrusionSensor.hpp>
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

// Hold current PCH register values
static unsigned int intrudeValue;

void ChassisIntrusionSensor::updateValue(const int newValue)
{
    // indicate that it is internal set call
    mInternalSet = true;
    mIface->set_property("ChassisIntrusionStatus", newValue);

    mValue = newValue;

    if (mOldValue == 0 && mValue == 1)
    {
        std::cerr << "std: save to SEL for intrusion assert event \n";
        // TODO: call add SEL log API, depends on patch #13956
        mOldValue = mValue;
    }
    else if (mOldValue == 1 && mValue == 0)
    {
        std::cerr << "std: save to SEL for intrusion de-assert event \n";
        // TODO: call add SEL log API, depends on patch #13956
        mOldValue = mValue;
    }
}

bool ChassisIntrusionSensor::i2cReadFromPch()
{
    std::string i2cBus = "/dev/i2c-" + std::to_string(mBusId);

    int fd = open(i2cBus.c_str(), O_RDWR | O_CLOEXEC);
    if (fd < 0)
    {
        std::cerr << "unable to open i2c device \n";
        return false;
    }
    if (ioctl(fd, I2C_SLAVE_FORCE, mSlaveAddr) < 0)
    {
        std::cerr << "unable to set device address\n";
        close(fd);
        return false;
    }

    unsigned long funcs = 0;
    if (ioctl(fd, I2C_FUNCS, &funcs) < 0)
    {
        std::cerr << "not support I2C_FUNCS \n";
        close(fd);
        return false;
    }

    if (!(funcs & I2C_FUNC_SMBUS_READ_BYTE_DATA))
    {
        std::cerr << "not support I2C_FUNC_SMBUS_READ_BYTE_DATA \n";
        close(fd);
        return false;
    }

    unsigned int statusValue;
    unsigned int statusMask = pchRegMaskIntrusion;
    unsigned int statusReg = pchStatusRegIntrusion;

    statusValue = i2c_smbus_read_byte_data(fd, statusReg);
    if (DEBUG)
    {
        std::cout << "\nRead bus " << mBusId << " addr " << mSlaveAddr
                  << ", value = " << statusValue << "\n";
    }

    close(fd);

    if (statusValue < 0)
    {
        std::cerr << "i2c_smbus_read_byte_data failed \n";
        return false;
    }

    // Get status value with mask
    int newValue = statusValue & statusMask;

    if (DEBUG)
    {
        std::cout << "statusValue is " << statusValue << "\n";
        std::cout << "Intrusion sensor value is " << mValue << "\n";
    }

    if (mOverridenState)
    {
        newValue = mOverriddenValue;
    }
    if (mValue != newValue)
    {
        updateValue(newValue);
        std::cout << "update value from " << mValue << " to " << newValue
                  << "\n";
    }

    return true;
}

void ChassisIntrusionSensor::pollPchInfo()
{
    // setting a new experation implicitly cancels any pending async wait
    mPollTimer.expires_from_now(
        boost::posix_time::seconds(intrusionSensorPollSec));

    mPollTimer.async_wait([&](const boost::system::error_code& ec) {
        // case of timer expired
        if (!ec)
        {
            if (i2cReadFromPch())
            {
                pollPchInfo();
            }
        }
        // case of being canceled
        else if (ec == boost::asio::error::operation_aborted)
        {
            std::cerr << "Poll timer cancelled. Return \n";
            return;
        }
    });
}

int ChassisIntrusionSensor::setSensorValue(const int& req, int& propertyValue)
{
    if (mInternalSet)
    {
        mInternalSet = false;
        propertyValue = req;
    }
    else
    {
        mOverriddenValue = req;
        mOverridenState = true;
    }
    return 1;
}

void ChassisIntrusionSensor::start(int busId, int slaveAddr)
{
    if (mBusId == busId && mSlaveAddr == slaveAddr)
    {
        return;
    }

    mBusId = busId;
    mSlaveAddr = slaveAddr;

    if (mBusId > 0 && mSlaveAddr > 0)
    {
        if (!mInitialized)
        {
            mIface->register_property("ChassisIntrusionStatus", mValue,
                                      [&](const int& req, int& propertyValue) {
                                          return setSensorValue(req,
                                                                propertyValue);
                                      });
            mIface->initialize();
            mInitialized = true;
        }

        pollPchInfo();
    }
    else
    {
        if (mInitialized)
        {
            mPollTimer.cancel();
            mInitialized = false;
        }
    }
}

ChassisIntrusionSensor::ChassisIntrusionSensor(
    boost::asio::io_service& io,
    std::shared_ptr<sdbusplus::asio::dbus_interface> iface) :
    mPollTimer(io),
    mIface(iface), mBusId(-1), mSlaveAddr(-1), mValue(-1), mOldValue(-1)
{
}

ChassisIntrusionSensor::~ChassisIntrusionSensor()
{
    mPollTimer.cancel();
}
