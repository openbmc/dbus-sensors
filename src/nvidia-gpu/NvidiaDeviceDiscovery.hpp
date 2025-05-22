/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "Utils.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";
constexpr const char* deviceType = "NvidiaMctpVdm";

struct SensorConfigs
{
    std::string name;
    uint64_t pollRate{};
};

class GpuDevice;

struct DeviceDiscoveryManager
{
  public:
    DeviceDiscoveryManager(const SensorConfigs& config, const std::string& path,
                           std::shared_ptr<sdbusplus::asio::connection>& conn,
                           boost::asio::io_context& io,
                           mctp::MctpRequester& mctpRequester,
                           sdbusplus::asio::object_server& objectServer);

    const std::string& getPath()
    {
        return path;
    }

  private:
    void discoverDevices();

    void processQueryDeviceIdResponse(uint8_t eid, int sendRecvMsgResult);

    void queryEndpoints(const boost::system::error_code& ec,
                        const GetSubTreeType& ret);

    void processEndpoint(const boost::system::error_code& ec,
                         const SensorBaseConfigMap& endpoint);

    void queryDeviceIdentification(uint8_t eid);

    std::array<uint8_t, sizeof(gpu::QueryDeviceIdentificationRequest)>
        queryDeviceIdentificationRequest{};

    std::array<uint8_t, sizeof(gpu::QueryDeviceIdentificationResponse)>
        queryDeviceIdentificationResponse{};

    boost::asio::io_context& io;

    mctp::MctpRequester& mctpRequester;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    sdbusplus::asio::object_server& objectServer;

    SensorConfigs configs;

    std::string path;

    std::vector<std::shared_ptr<GpuDevice>> gpuDevice;
};

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<
        std::string, std::shared_ptr<DeviceDiscoveryManager>>& gpuDevice,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    mctp::MctpRequester& mctpRequester);

void interfaceRemoved(
    sdbusplus::message_t& message,
    boost::container::flat_map<
        std::string, std::shared_ptr<DeviceDiscoveryManager>>& gpuDevice);
