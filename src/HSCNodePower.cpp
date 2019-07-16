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

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

static constexpr double hscMaxReading = 0xFF;
static constexpr double hscMinReading = 0;
static constexpr const char* nodePwrInf =
    "/xyz/openbmc_project/sensors/power/Node_PWR";

HSCNodePowerSensor::HSCNodePowerSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_service& io, const std::string& sensorName,
    const std::string& sensorConfiguration,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholds, uint8_t busId,
    uint8_t hscAddress, HSCType type) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"), std::move(thresholds),
           sensorConfiguration, "xyz.openbmc_project.Configuration.MP5920",
           hscMaxReading, hscMinReading),
    mBusId(busId), mSlaveAddr(hscAddress), mType(type), mPollTimer(io),
    mObjServer(objectServer), mConn(conn), mValue(0)
{
    sensorInterface = objectServer.add_interface(
        nodePwrInf, "xyz.openbmc_project.Sensor.Value");

    sensorInterface->register_property("Value", mValue);

    if (thresholds::hasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            nodePwrInf, "xyz.openbmc_project.Sensor.Threshold.Warning");
    }
    if (thresholds::hasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            nodePwrInf, "xyz.openbmc_project.Sensor.Threshold.Critical");
    }
    association =
        objectServer.add_interface(nodePwrInf, "org.openbmc.Associations");

    setInitialProperties(conn);
    // setup match
    setupPowerMatch(conn);
}

HSCNodePowerSensor::~HSCNodePowerSensor()
{
    mPollTimer.cancel();
    mObjServer.remove_interface(mItemIface);
    mObjServer.remove_interface(thresholdInterfaceWarning);
    mObjServer.remove_interface(thresholdInterfaceCritical);
    mObjServer.remove_interface(sensorInterface);
    mObjServer.remove_interface(association);
}

int HSCNodePowerSensor::getHSCRegsInfoByte(uint8_t regs, int8_t* pu8data)
{
    std::string i2cBus = "/dev/i2c-" + std::to_string(mBusId);
    int fd = open(i2cBus.c_str(), O_RDWR);
    int ret = 0;

    if (fd < 0)
    {
        return -1;
    }

    do
    {
        if (ioctl(fd, I2C_SLAVE_FORCE, mSlaveAddr) < 0)
        {
            ret = -1;
            break;
        }

        unsigned long funcs = 0;
        if (ioctl(fd, I2C_FUNCS, &funcs) < 0)
        {
            ret = -1;
            break;
        }

        if (!(funcs & I2C_FUNC_SMBUS_READ_BYTE_DATA))
        {
            ret = -1;
            break;
        }

        *pu8data = i2c_smbus_read_byte_data(fd, regs);
        if (*pu8data < 0)
        {
            ret = -1;
            break;
        }
    } while (0);
    close(fd);

    return ret;
}

int HSCNodePowerSensor::getHSCRegsInfoBytes(uint8_t regs, uint8_t length,
                                            uint8_t* pu8data)
{
    std::string i2cBus = "/dev/i2c-" + std::to_string(mBusId);
    int fd = open(i2cBus.c_str(), O_RDWR);
    int ret = 0;

    if (fd < 0)
    {
        return -1;
    }

    do
    {
        if (ioctl(fd, I2C_SLAVE_FORCE, mSlaveAddr) < 0)
        {
            ret = -1;
            break;
        }

        unsigned long funcs = 0;
        if (ioctl(fd, I2C_FUNCS, &funcs) < 0)
        {
            ret = -1;
            break;
        }

        if (!(funcs & I2C_FUNC_SMBUS_READ_I2C_BLOCK))
        {
            ret = -1;
            break;
        }

        int len = i2c_smbus_read_i2c_block_data(fd, regs, length, pu8data);
        if (len < 0)
        {
            ret = -1;
            break;
        }
    } while (0);
    close(fd);

    return ret;
}

int HSCNodePowerSensor::setHSCRegsInfoByte(uint8_t regs, uint8_t pu8data)
{
    std::string i2cBus = "/dev/i2c-" + std::to_string(mBusId);
    int fd = open(i2cBus.c_str(), O_RDWR);
    int ret = 0;

    if (fd < 0)
    {
        return -1;
    }

    do
    {
        if (ioctl(fd, I2C_SLAVE_FORCE, mSlaveAddr) < 0)
        {
            ret = -1;
            break;
        }

        unsigned long funcs = 0;
        if (ioctl(fd, I2C_FUNCS, &funcs) < 0)
        {
            ret = -1;
            break;
        }

        if (!(funcs & I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
        {
            ret = -1;
            break;
        }

        int len = i2c_smbus_write_byte_data(fd, regs, pu8data);
        if (len < 0)
        {
            ret = -1;
            break;
        }
    } while (0);
    close(fd);

    return ret;
}

int HSCNodePowerSensor::setHSCRegsInfoBytes(uint8_t regs, uint8_t length,
                                            uint8_t* pu8data)
{
    std::string i2cBus = "/dev/i2c-" + std::to_string(mBusId);
    int fd = open(i2cBus.c_str(), O_RDWR);
    int ret = 0;

    if (fd < 0)
    {
        return -1;
    }

    do
    {
        if (ioctl(fd, I2C_SLAVE_FORCE, mSlaveAddr) < 0)
        {
            ret = -1;
            break;
        }

        unsigned long funcs = 0;
        if (ioctl(fd, I2C_FUNCS, &funcs) < 0)
        {
            ret = -1;
            break;
        }

        if (!(funcs & I2C_FUNC_SMBUS_WRITE_I2C_BLOCK))
        {
            ret = -1;
            break;
        }

        int len = i2c_smbus_write_i2c_block_data(fd, regs, length, pu8data);
        if (len < 0)
        {
            ret = -1;
            break;
        }
    } while (0);
    close(fd);

    return ret;
}

void HSCNodePowerSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}

void HSCNodePowerSensor::initCoefficient()
{
    if (mType == HSCType::MP5920)
    {
        mHSCCoefficient.currentB = mp5920CurrentB;
        mHSCCoefficient.currentM = mp5920CurrentM;
        mHSCCoefficient.currentR = mp5920CurrentR;
        mHSCCoefficient.powerB = mp5920PwrB;
        mHSCCoefficient.powerM = mp5920PwrM;
        mHSCCoefficient.powerR = mp5920PwrR;
        mHSCCoefficient.voltageB = mp5920VoltageB;
        mHSCCoefficient.voltageM = mp5920VoltageM;
        mHSCCoefficient.voltageR = mp5920VoltageR;
    }
}

void HSCNodePowerSensor::initMP5920()
{
    uint8_t writeBuff[2];
    int iStatus;

    writeBuff[0] = pmbusPwdLow;
    writeBuff[1] = pmbusPwdHigh;
    iStatus =
        setHSCRegsInfoBytes(mp5920PmbusPwd, sizeof(writeBuff), &writeBuff[0]);
    if (iStatus != 0)
    {
        return;
    }

    writeBuff[0] = blackboxPwdLow;
    writeBuff[1] = blackboxPwdHigh;
    iStatus = setHSCRegsInfoBytes(mp5920BlackboxPwd, sizeof(writeBuff),
                                  &writeBuff[0]);
    if (iStatus != 0)
    {
        return;
    }

    writeBuff[0] = nvmPwdLow;
    writeBuff[1] = blackboxPwdHigh;
    iStatus =
        setHSCRegsInfoBytes(mp5920NvmPwd, sizeof(writeBuff), &writeBuff[0]);
    if (iStatus != 0)
    {
        return;
    }

    writeBuff[0] = operationOn;
    iStatus = setHSCRegsInfoBytes(mp5920Operation, 1, &writeBuff[0]);
    if (iStatus != 0)
    {
        return;
    }
}

void HSCNodePowerSensor::configHSCDevice()
{
    if (mType == HSCType::MP5920)
    {
        initMP5920();
    }
}

void HSCNodePowerSensor::updateValue(uint64_t newValue)
{
    mItemIface->set_property("Value", newValue);
    mValue = newValue;
}

int HSCNodePowerSensor::getAveragePower(SHSCDevInfo* psHSCInfo)
{
    uint16_t u16Raw;
    double dReading;
    uint8_t u8ReadBuf[7];
    int16_t Accumulator;
    uint8_t Rollover_count;
    // STATUS Status;
    bool berror = false;
    uint16_t powerVal;
    int ret;

    ret = getHSCRegsInfoBytes(pmbCmdReadEin, 7, u8ReadBuf);
    if (ret != 0)
    {
        return -1;
    }

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
             static_cast<double>(mHSCCoefficient.powerM)) *
            ((static_cast<double>(u16Raw) * pow(10, mHSCCoefficient.powerR)) -
             static_cast<double>(mHSCCoefficient.powerB));

        powerVal = dReading;
        // PRINT(PRINT_CMC, PRINT_INFO, "hsc %d reading is %lf. \n",
        // psHSCInfo->u8HSCNum, dReading);
        if (static_cast<uint64_t>(powerVal) <= 0x7FFF)
        {
            // Truncate the power value. The ME currently does not have a way to
            // accept a non-whole-integer power value.
            psHSCInfo->u64AveragePower =
                (static_cast<uint64_t>(powerVal) & 0x7FFF);
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

void HSCNodePowerSensor::pollHSCChipRegs()
{
    // setting a new experation implicitly cancels any pending async wait
    mPollTimer.expires_from_now(boost::posix_time::seconds(1));

    mPollTimer.async_wait([&](const boost::system::error_code& ec) {
        // case of timer expired
        if (!ec)
        {
            if (!isPowerOn())
            {
                if (!getAveragePower(&mHSCDevInfo))
                {
                    if ((mValue != mHSCDevInfo.u64AveragePower))
                    {
                        updateValue(mHSCDevInfo.u64AveragePower);
                    }
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

int HSCNodePowerSensor::pingHSCDevice()
{
    int status;
    int8_t buf = 0;

    if (mBusId < 0 || mSlaveAddr < 0)
    {
        return -1;
    }

    status = getHSCRegsInfoByte(pmbusRevision, &buf);

    if (0 != status)
    {
        std::cerr << "PingHSCDevice Failed..."
                  << "\n";
        return 1;
    }

    return 0;
}

void HSCNodePowerSensor::start(HSCType type, int busId, int slaveAddr)
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
