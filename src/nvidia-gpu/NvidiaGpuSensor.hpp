/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";
constexpr const char* sensorType = "NvidiaMctpVdm";

struct GpuTempSensor :
    public Sensor,
    public std::enable_shared_from_this<GpuTempSensor>
{
  public:
    GpuTempSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                  boost::asio::io_context& io,
                  mctp::MctpRequester& mctpRequester, const std::string& name,
                  const std::string& sensorConfiguration,
                  sdbusplus::asio::object_server& objectServer,
                  std::vector<thresholds::Threshold>&& thresholdData,
                  std::chrono::milliseconds pollRate);

    ~GpuTempSensor() override;

    void checkThresholds() override;

  private:
    void read();

    void update();

    void discoverGpus();

    void processResponse(int sendRecvMsgResult);

    void processQueryDeviceIdResponse(uint8_t eid, int sendRecvMsgResult);

    void queryEndpoints(const boost::system::error_code& ec,
                        const GetSubTreeType& ret);

    void processEndpoint(const boost::system::error_code& ec,
                         const SensorBaseConfigMap& endpoint);
    void processGpuEndpoint(uint8_t eid);

    uint8_t eid{};

    uint8_t sensorId;

    std::chrono::milliseconds sensorPollMs;

    boost::asio::steady_timer waitTimer;

    mctp::MctpRequester& mctpRequester;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    sdbusplus::asio::object_server& objectServer;

    std::array<uint8_t, sizeof(ocp::accelerator_management::Message) +
                            sizeof(gpu::GetTemperatureReadingRequest)>
        getTemperatureReadingRequest{};

    std::array<uint8_t, sizeof(ocp::accelerator_management::Message) +
                            sizeof(gpu::GetTemperatureReadingResponse)>
        getTemperatureReadingResponse{};

    std::array<uint8_t, sizeof(ocp::accelerator_management::Message) +
                            sizeof(gpu::QueryDeviceIdentificationRequest)>
        queryDeviceIdentificationRequest{};

    std::array<uint8_t, sizeof(ocp::accelerator_management::Message) +
                            sizeof(gpu::QueryDeviceIdentificationResponse)>
        queryDeviceIdentificationResponse{};
};

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuTempSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    mctp::MctpRequester& mctpRequester);

void interfaceRemoved(
    sdbusplus::message_t& message,
    boost::container::flat_map<std::string, std::shared_ptr<GpuTempSensor>>&
        sensors);
