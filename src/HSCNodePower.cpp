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

#include <HSCNodePower.hpp>
#include <Utils.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>
#include <string>
#include <thread>

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

HSCNodePower::HSCNodePower(boost::asio::io_service& io,
                           sdbusplus::asio::object_server& objectServer,
                           std::shared_ptr<sdbusplus::asio::connection>& conn) :
    mPollTimer(io),
    mObjServer(objectServer), mConn(conn), mType(HSCType::MP5920), mBusId(-1),
    mSlaveAddr(-1), mValue(0)
{
}

HSCNodePower::~HSCNodePower()
{
    mPollTimer.cancel();
    mObjServer.remove_interface(mItemIface);
}

int HSCNodePower::getHSCRegsInfoByte(uint8_t regs, uint8_t* pu8data)
{
    std::string i2cBus = "/dev/i2c-" + std::to_string(mBusId);
    int fd = open(i2cBus.c_str(), O_RDWR);

    size_t i = 0;
    if (fd < 0)
    {
        return -1;
    }
    if (ioctl(fd, I2C_SLAVE_FORCE, mSlaveAddr) < 0)
    {
        close(fd);
        return -1;
    }

    unsigned long funcs = 0;
    if (ioctl(fd, I2C_FUNCS, &funcs) < 0)
    {
        close(fd);
        return -1;
    }

    if (!(funcs & I2C_FUNC_SMBUS_READ_BYTE_DATA))
    {
        close(fd);
        return -1;
    }

    *pu8data = i2c_smbus_read_byte_data(fd, regs);
    if (*pu8data < 0)
    {
        return -1;
    }

    close(fd);
    return 0;
}

int HSCNodePower::getHSCRegsInfoBytes(uint8_t regs, uint8_t length,
                                      uint8_t* pu8data)
{
    std::string i2cBus = "/dev/i2c-" + std::to_string(mBusId);
    int fd = open(i2cBus.c_str(), O_RDWR);

    size_t i = 0;
    if (fd < 0)
    {
        return -1;
    }
    if (ioctl(fd, I2C_SLAVE_FORCE, mSlaveAddr) < 0)
    {
        close(fd);
        return -1;
    }

    unsigned long funcs = 0;
    if (ioctl(fd, I2C_FUNCS, &funcs) < 0)
    {
        close(fd);
        return -1;
    }

    if (!(funcs & I2C_FUNC_SMBUS_READ_I2C_BLOCK))
    {
        close(fd);
        return -1;
    }

    int len = i2c_smbus_read_i2c_block_data(fd, regs, length, pu8data);
    if (len < 0)
    {
        return -1;
    }

    close(fd);
    return 0;
}

int HSCNodePower::setHSCRegsInfoByte(uint8_t regs, uint8_t length,
                                     uint8_t pu8data)
{
    std::string i2cBus = "/dev/i2c-" + std::to_string(mBusId);
    int fd = open(i2cBus.c_str(), O_RDWR);
    size_t i = 0;

    if (fd < 0)
    {
        return -1;
    }
    if (ioctl(fd, I2C_SLAVE_FORCE, mSlaveAddr) < 0)
    {
        close(fd);
        return -1;
    }

    unsigned long funcs = 0;
    if (ioctl(fd, I2C_FUNCS, &funcs) < 0)
    {
        close(fd);
        return -1;
    }

    if (!(funcs & I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
    {
        close(fd);
        return -1;
    }

    int len = i2c_smbus_write_byte_data(fd, regs, pu8data);
    if (len < 0)
    {
        return -1;
    }

    close(fd);
    return 0;
}

int HSCNodePower::setHSCRegsInfoBytes(uint8_t regs, uint8_t length,
                                      uint8_t* pu8data)
{
    std::string i2cBus = "/dev/i2c-" + std::to_string(mBusId);
    int fd = open(i2cBus.c_str(), O_RDWR);
    size_t i = 0;

    if (fd < 0)
    {
        return -1;
    }
    if (ioctl(fd, I2C_SLAVE_FORCE, mSlaveAddr) < 0)
    {
        close(fd);
        return -1;
    }

    unsigned long funcs = 0;
    if (ioctl(fd, I2C_FUNCS, &funcs) < 0)
    {
        close(fd);
        return -1;
    }

    if (!(funcs & I2C_FUNC_SMBUS_WRITE_I2C_BLOCK))
    {
        close(fd);
        return -1;
    }

    int len = i2c_smbus_write_i2c_block_data(fd, regs, length, pu8data);
    if (len < 0)
    {
        return -1;
    }

    close(fd);
    return 0;
}

void HSCNodePower::initCoefficient()
{
    if (mType == HSCType::MP5920)
    {
        mHSCCoefficient.current_b = MP5920_CURRENT_b;
        mHSCCoefficient.current_m = MP5920_CURRENT_m;
        mHSCCoefficient.current_r = MP5920_CURRENT_r;
        mHSCCoefficient.power_b = MP5920_PWR_b;
        mHSCCoefficient.power_m = MP5920_PWR_m;
        mHSCCoefficient.power_r = MP5920_PWR_r;
        mHSCCoefficient.voltage_b = MP5920_VOLTAGE_b;
        mHSCCoefficient.voltage_m = MP5920_VOLTAGE_m;
        mHSCCoefficient.voltage_r = MP5920_VOLTAGE_r;
    }
}

void HSCNodePower::initMP5920()
{
    uint8_t writeBuff[2];
    int iStatus;

    writeBuff[0] = PMBUS_PWD_LOW;
    writeBuff[1] = PMBUS_PWD_HIGH;
    iStatus =
        setHSCRegsInfoBytes(MP5920_PMBUS_PWD, sizeof(writeBuff), &writeBuff[0]);
    if (iStatus != 0)
    {
        return;
    }

    writeBuff[0] = BLACKBOX_PWD_LOW;
    writeBuff[1] = BLACKBOX_PWD_HIGH;
    iStatus = setHSCRegsInfoBytes(MP5920_BLACKBOX_PWD, sizeof(writeBuff),
                                  &writeBuff[0]);
    if (iStatus != 0)
    {
        return;
    }

    writeBuff[0] = NVM_PWD_LOW;
    writeBuff[1] = BLACKBOX_PWD_HIGH;
    iStatus =
        setHSCRegsInfoBytes(MP5920_NVM_PWD, sizeof(writeBuff), &writeBuff[0]);
    if (iStatus != 0)
    {
        return;
    }

    writeBuff[0] = OPERATION_ON;
    iStatus = setHSCRegsInfoBytes(MP5920_OPERATION, 1, &writeBuff[0]);
    if (iStatus != 0)
    {
        return;
    }
}

void HSCNodePower::configHSCDevice()
{
    mItemIface =
        mObjServer.add_interface("/xyz/openbmc_project/sensors/power/Node_PWR",
                                 "xyz.openbmc_project.Sensor.Value");

    mItemIface->register_property("Value", mValue);
    mItemIface->initialize();

    // setup match
    setupPowerMatch(mConn);

    initCoefficient();

    if (mType == HSCType::MP5920)
    {
        initMP5920();
    }
}

void HSCNodePower::updateValue(uint64_t newValue)
{
    mItemIface->set_property("Value", newValue);
    mValue = newValue;
}

int HSCNodePower::get_average_power(SHSCDevInfo* psHSCInfo)
{
    uint16_t u16Raw;
    double dReading;
    uint8_t u8WriteBuf[4] = {0};
    uint8_t u8ReadBuf[7] = {0};
    int16_t Accumulator;
    uint8_t Rollover_count;
    // STATUS Status;
    bool berror = false;
    uint16_t powerVal;

    getHSCRegsInfoBytes(PMB_CMD_READ_EIN, 7, u8ReadBuf);

    Accumulator = (u8ReadBuf[2] << 8) + u8ReadBuf[1];
    Rollover_count = u8ReadBuf[3];
    psHSCInfo->u64CurrentSampleCount =
        (u8ReadBuf[6] << 16) + (u8ReadBuf[5] << 8) + u8ReadBuf[4];
    psHSCInfo->u64CurrentEnergyCount = (Rollover_count << 15) + Accumulator;

    if (psHSCInfo->u64CurrentSampleCount > psHSCInfo->u64PrevSampleCount &&
        psHSCInfo->u64CurrentEnergyCount > psHSCInfo->u64PrevEnergyCount &&
        psHSCInfo->u64PrevSampleCount != 0 &&
        psHSCInfo->u64PrevEnergyCount != 0)
    {

        u16Raw =
            (psHSCInfo->u64CurrentEnergyCount - psHSCInfo->u64PrevEnergyCount) /
            (psHSCInfo->u64CurrentSampleCount - psHSCInfo->u64PrevSampleCount);
        dReading =
            (static_cast<double>(1) /
             static_cast<double>(mHSCCoefficient.power_m)) *
            ((static_cast<double>(u16Raw) * pow(10, mHSCCoefficient.power_r)) -
             static_cast<double>(mHSCCoefficient.power_b));

        powerVal = dReading;
        // PRINT(PRINT_CMC, PRINT_INFO, "hsc %d reading is %lf. \n",
        // psHSCInfo->u8HSCNum, dReading);
        if (static_cast<uint64_t>(powerVal) <= 0x7FFF)
        {
            // Truncate the power value. The ME currently does not have a way to
            // accept a non-whole-integer power value.
            psHSCInfo->u64AveragePower =
                (static_cast<uint16_t>(powerVal) & 0x7FFF);
        }
        else
        {
            // Anything greater than 0x7FFF should be treated as an error.
            // Return a reading of -1 to indicate to the ME that there was a
            // problem.
            psHSCInfo->u64AveragePower = -1;
        }
    }
    else
    // sample frequency is too fast, last_sample_count == cur_sample_count
    // sample count roll over, last_sample_count > cur_sample_count
    // invalid power reading, last_power_accumulator > cur_power_accumulator
    // first sample, cur_sample_count is 0 or cur_energy_count is 0
    {
        berror = true;
    }

    psHSCInfo->u64PrevEnergyCount = psHSCInfo->u64CurrentEnergyCount;
    psHSCInfo->u64PrevSampleCount = psHSCInfo->u64CurrentSampleCount;

    if (berror)
    {
        return -1;
    }

    return 0;
}

void HSCNodePower::pollHSCChipRegs()
{
    // setting a new experation implicitly cancels any pending async wait
    mPollTimer.expires_from_now(boost::posix_time::seconds(1));

    mPollTimer.async_wait([&](const boost::system::error_code& ec) {
        // case of timer expired
        if (!ec)
        {
            if (!isPowerOn())
            {
                int statusValue = get_average_power(&g_sHSCDevInfo);

                if ((mValue != g_sHSCDevInfo.u64AveragePower))
                {
                    updateValue(g_sHSCDevInfo.u64AveragePower);
                }
            }
            else
            {
                updateValue(0);
            }

            // trigger next polling
            pollHSCChipRegs();
        }
        // case of being canceled
        else if (ec == boost::asio::error::operation_aborted)
        {
            return;
        }
    });
}

int HSCNodePower::pingHSCDevice()
{
    int status;
    uint8_t buf = 0;

    if (mBusId < 0 || mSlaveAddr < 0)
    {
        return -1;
    }

    status = getHSCRegsInfoByte(PMBUS_REVISION, &buf);

    if (0 != status)
    {
        std::cerr << "PingHSCDevice Failed..."
                  << "\n";
        return 1;
    }

    return 0;
}

void HSCNodePower::start(HSCType type, int busId, int slaveAddr)
{
    if ((type == HSCType::MP5920 && busId == mBusId && slaveAddr == mSlaveAddr))
    {
        return;
    }

    mType = type;
    mBusId = busId;
    mSlaveAddr = slaveAddr;

    if (pingHSCDevice() != 0)
    {
        return;
    }
    // config HSC device regs
    configHSCDevice();

    // read data from HSC device
    pollHSCChipRegs();
}