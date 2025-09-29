/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "NvidiaDeviceDiscovery.hpp"
#include "NvidiaPcieInterface.hpp"

#include <NvidiaPciePort.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

constexpr const char* pcieDevicePathPrefix =
    "/xyz/openbmc_project/inventory/pcie_devices/";

constexpr const char* fabricPath =
    "/xyz/openbmc_project/inventory/fabrics/PCIeTopology";

struct PcieDeviceInfo
{
    uint16_t numUpstreamPorts{};
    std::vector<uint8_t> numDownstreamPorts;
};

class PcieDevice : public std::enable_shared_from_this<PcieDevice>
{
  public:
    PcieDevice(const SensorConfigs& configs, const std::string& name,
               const std::string& path,
               const std::shared_ptr<sdbusplus::asio::connection>& conn,
               uint8_t eid, boost::asio::io_context& io,
               mctp::MctpRequester& mctpRequester,
               sdbusplus::asio::object_server& objectServer);

    const std::string& getPath() const
    {
        return path;
    }

    void init();

  private:
    void makeSensors();

    void read();

    void getPciePortCounts();

    void processPciePortCountsResponse(const std::error_code& ec,
                                       std::span<const uint8_t> response);

    PcieDeviceInfo pcieDeviceInfo;

    uint8_t eid{};

    std::chrono::milliseconds sensorPollMs;

    boost::asio::steady_timer waitTimer;

    mctp::MctpRequester& mctpRequester;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    sdbusplus::asio::object_server& objectServer;

    SensorConfigs configs;

    std::string name;

    std::string path;

    std::array<uint8_t, sizeof(ocp::accelerator_management::CommonRequest)>
        request{};

    std::shared_ptr<NvidiaPcieInterface> pcieInterface;
    static std::shared_ptr<sdbusplus::asio::dbus_interface> fabricInterface;

    std::vector<std::shared_ptr<NvidiaPciePortInfo>> pciePorts;
};
