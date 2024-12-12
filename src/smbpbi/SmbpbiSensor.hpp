/*
// SPDX-FileCopyrightText: Copyright (c) 2022-2024 NVIDIA CORPORATION &
// AFFILIATES. All rights reserved.
//SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#pragma once
#include <boost/asio/io_context.hpp>
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

int i2cCmd(uint8_t bus, uint8_t addr, size_t offset, uint8_t* reading,
           int length);
int i2cCmdDouble(uint8_t bus, uint8_t addr, size_t offset, double& reading);
int i2cCmdUI64(uint8_t bus, uint8_t addr, size_t offset, uint64_t& reading);

constexpr std::array<int, 4> I2C_READ_LEN_VALUES = {4, 8, 8, 8};

enum class I2C_READ_LEN_INDEX
{
    FLOAT32,
    FLOAT64,
    DOUBLE,
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
        uint8_t addr, uint16_t offset, std::string& sensorType,
        std::string& valueType, size_t pollTime, double minVal, double maxVal);
    ~SmbpbiSensor() override;

    void checkThresholds() override;

    size_t getPollRate() const
    {
        return pollRate;
    }
    void read();
    void init();

    std::string name;
    uint8_t busId;
    uint8_t addr;
    uint16_t offset;
    std::string sensorType;
    std::string valueType;

  private:
    int readRawEEPROMData(size_t off, double& data) const;
    int readDoubleEEPROMData(size_t off, double& data) const;
    double reading2tempEp(const uint8_t* rawData) const;
    double reading2power(const uint8_t* rawData) const;
    void waitReadCallback(const boost::system::error_code& ec);
    static uint8_t getLength(uint16_t offset)
    {
        // return offset to avoid the unused variable error.
        return offset;
    }
    sdbusplus::asio::object_server& objectServer;
    boost::asio::steady_timer waitTimer;
    size_t pollRate;
};
