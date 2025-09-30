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
#include <NvidiaPciePortMetrics.hpp>
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
    "/xyz/openbmc_project/inventory/PCIeDevices/";

constexpr const char* fabricPathPrefix =
    "/xyz/openbmc_project/inventory/fabrics/";

class PcieDevice
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

  private:
    void makeSensors();

    void read();

    uint8_t eid{};

    std::chrono::milliseconds sensorPollMs;

    boost::asio::steady_timer waitTimer;

    mctp::MctpRequester& mctpRequester;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    sdbusplus::asio::object_server& objectServer;

    SensorConfigs configs;

    std::string name;

    std::string path;

    std::string fabricPath;

    std::shared_ptr<NvidiaPcieInterface> pcieInterface;
    static std::unordered_map<std::string,
                              std::shared_ptr<sdbusplus::asio::dbus_interface>>
        fabricInterfaces;

    std::vector<std::shared_ptr<NvidiaPciePortInfo>> pciePorts;
    std::vector<std::shared_ptr<NvidiaPciePortErrors>> pciePortErrors;
    std::vector<std::shared_ptr<NvidiaPciePortCounters>> pciePortCounters;
    std::vector<std::shared_ptr<NvidiaPciePortL0ToRecoveryCount>>
        pciePortL0ToRecoveryCounts;
};
