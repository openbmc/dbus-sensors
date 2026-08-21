/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "Thresholds.hpp"
#include "sensor.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

struct NvidiaSmaLeakSensor :
    public Sensor,
    public std::enable_shared_from_this<NvidiaSmaLeakSensor>
{
  public:
    NvidiaSmaLeakSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                        const std::string& name,
                        const std::string& sensorConfiguration,
                        sdbusplus::asio::object_server& objectServer,
                        std::vector<thresholds::Threshold>&& thresholdData,
                        gpu::DeviceIdentification deviceType);

    ~NvidiaSmaLeakSensor() override;

    void checkThresholds() override;

    void updateState(uint8_t value);

  private:
    std::shared_ptr<sdbusplus::asio::connection> conn;

    sdbusplus::asio::object_server& objectServer;

    std::shared_ptr<sdbusplus::asio::dbus_interface>
        commonPhysicalContextInterface;

    std::shared_ptr<sdbusplus::asio::dbus_interface> leakDetectorInterface;

    std::shared_ptr<sdbusplus::asio::dbus_interface> leakFaultInterface;

    enum class LeakState
    {
        Normal,
        Abnormal
    };

    LeakState lastLeakState = LeakState::Normal;

    LeakState lastLeakFault = LeakState::Normal;
};

struct NvidiaSmaLeakSensorCarrier :
    public std::enable_shared_from_this<NvidiaSmaLeakSensorCarrier>
{
  public:
    NvidiaSmaLeakSensorCarrier(
        std::shared_ptr<sdbusplus::asio::connection>& conn,
        mctp::MctpRequester& mctpRequester, const std::string& name,
        const std::string& sensorConfiguration, uint8_t eid,
        sdbusplus::asio::object_server& objectServer,
        std::vector<thresholds::Threshold>&& thresholdData,
        gpu::DeviceIdentification deviceType);

    ~NvidiaSmaLeakSensorCarrier() = default;

    void init();

    void update();

  private:
    void processResponse(const std::error_code& ec,
                         std::span<const uint8_t> buffer);

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    std::string name;

    std::string sensorConfiguration;

    uint8_t eid;

    sdbusplus::asio::object_server& objectServer;

    std::vector<thresholds::Threshold> thresholdData;

    gpu::DeviceIdentification deviceType;

    std::array<uint8_t, gpu::getLeakDetectionInfoRequestSize> request{};

    std::map<uint8_t, std::shared_ptr<NvidiaSmaLeakSensor>> leakSensors;

    std::vector<gpu::LeakSensorData> parsedSensors;
};
