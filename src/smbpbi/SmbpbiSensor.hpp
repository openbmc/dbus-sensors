/*
 * SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <boost/asio/io_context.hpp>
#include <boost/asio/random_access_file.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

constexpr std::array<int, 4> I2C_READ_LEN_VALUES = {4, 8, 8, 8};

enum class I2C_READ_LEN_INDEX
{
    FLOAT32,
    FLOAT64,
    DOUBLE64,
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
        std::string& path);
    ~SmbpbiSensor() override;

    void checkThresholds() override;

    size_t getPollRate() const
    {
        return pollRate;
    }
    void read();
    void init();
    int i2cCmd(uint8_t* reading, int length);
    int i2cCmdDouble(double& reading);
    int i2cCmdUI64(uint64_t& reading);

    std::string name;
    uint8_t busId;
    uint8_t addr;
    uint16_t offset;
    std::string sensorUnits;
    std::string sensorType;
    std::string valueType;

  private:
    int readRawEEPROMData(double& data);
    int readDoubleEEPROMData(double& data);
    double reading2tempEp(const uint8_t* rawData) const;
    double reading2power(const uint8_t* rawData) const;
    void waitReadCallback(const boost::system::error_code& ec);
    static uint8_t getLength(uint16_t offset)
    {
        // return offset to avoid the unused variable error.
        return offset;
    }
    sdbusplus::asio::object_server& objectServer;
    boost::asio::random_access_file inputDev;
    boost::asio::steady_timer waitTimer;
    size_t pollRate;
};
