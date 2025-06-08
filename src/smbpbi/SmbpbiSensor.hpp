/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include "Thresholds.hpp"
#include "Utils.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/random_access_file.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

constexpr std::array<size_t, 3> i2CReadLenValues = {4, 8, 8};

enum class I2C_READ_LEN_INDEX
{
    FLOAT32,
    FLOAT64,
    UINT64
};

struct SmbpbiSensor : public Sensor
{
    SmbpbiSensor(
        std::shared_ptr<sdbusplus::asio::connection>& conn,
        boost::asio::io_context& io, const std::string& name,
        const std::string& sensorConfiguration, const std::string& objType,
        sdbusplus::asio::object_server& objectServer,
        std::vector<thresholds::Threshold>&& thresholdData, uint8_t busId,
        uint8_t addr, uint16_t offset, std::string& sensorUnits,
        std::string& valueType, size_t pollTime, double minVal, double maxVal,
        std::string& path, const PowerState& powerState);
    ~SmbpbiSensor() override;

    void checkThresholds() override;

    size_t getPollRate() const
    {
        return pollRateSecond;
    }
    void read();
    void init();

    uint8_t busId;
    uint8_t addr;
    uint16_t offset;
    std::string sensorUnits;
    std::string sensorType;
    std::string valueType;

  private:
    int i2cReadDataBytes(uint8_t* reading, int length);
    int i2cReadDataBytesDouble(double& reading);
    int i2cReadDataBytesUI64(uint64_t& reading);
    int readRawEEPROMData(double& data);
    int readFloat64EEPROMData(double& data);
    static double convert2Temp(const uint8_t* rawData);
    static double convert2Power(const uint8_t* rawData);
    void waitReadCallback(const boost::system::error_code& ec);
    sdbusplus::asio::object_server& objectServer;
    boost::asio::random_access_file inputDev;
    boost::asio::steady_timer waitTimer;
    size_t pollRateSecond;
};

bool checkInvalidReading(uint8_t* reading, int length)
{
    // there is no value updated from HMC if reading data is all 0xff
    uint8_t* ptr = reading;
    for (int i = 0; i < length; i++, ptr++)
    {
        if (*ptr != 0xFF)
        {
            return false;
        }
    }
    return true;
}
