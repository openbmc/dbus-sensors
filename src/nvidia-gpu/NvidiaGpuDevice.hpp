/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "Inventory.hpp"
#include "MctpRequester.hpp"
#include "NvidiaDeviceDiscovery.hpp"
#include "NvidiaGpuSensor.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "Memory.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/asio/property.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

using InventoryRequestBuffer =
    std::array<uint8_t, sizeof(gpu::GetInventoryInformationRequest)>;
using InventoryResponseBuffer =
    std::array<uint8_t, sizeof(gpu::GetInventoryInformationResponse)>;

class GpuDevice : public std::enable_shared_from_this<GpuDevice>
{
  public:
    GpuDevice(const SensorConfigs& configs, const std::string& name,
              const std::string& path,
              const std::shared_ptr<sdbusplus::asio::connection>& conn,
              uint8_t eid, boost::asio::io_context& io,
              mctp::MctpRequester& mctpRequester,
              sdbusplus::asio::object_server& objectServer);

    const std::string& getPath() const
    {
        return path;
    }

  private:
    void makeSensors();
    void read();
    void fetchUUID();
    void handleUUIDResponse(int sendRecvMsgResult,
                           std::shared_ptr<InventoryResponseBuffer> responseBuffer);

    uint8_t eid{};
    std::chrono::milliseconds sensorPollMs;
    boost::asio::steady_timer waitTimer;
    mctp::MctpRequester& mctpRequester;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    sdbusplus::asio::object_server& objectServer;
    std::shared_ptr<NvidiaGpuTempSensor> tempSensor;
    SensorConfigs configs;
    std::string name;
    std::string path;
    std::string uuid;
    std::shared_ptr<sdbusplus::asio::dbus_interface> acceleratorInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> uuidInterface;
    std::unique_ptr<Inventory> inventory;
    std::shared_ptr<Memory> memoryModule;
};
