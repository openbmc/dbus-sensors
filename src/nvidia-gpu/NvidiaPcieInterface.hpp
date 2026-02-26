/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

struct NvidiaPcieInterface :
    public std::enable_shared_from_this<NvidiaPcieInterface>
{
  public:
    NvidiaPcieInterface(std::shared_ptr<sdbusplus::asio::connection>& conn,
                        mctp::MctpRequester& mctpRequester,
                        const std::string& name, const std::string& path,
                        uint8_t eid,
                        sdbusplus::asio::object_server& objectServer);

    NvidiaPcieInterface(
        std::shared_ptr<sdbusplus::asio::connection>& conn,
        mctp::MctpRequester& mctpRequester, const std::string& name,
        const std::string& path, uint8_t eid,
        sdbusplus::asio::object_server& objectServer,
        gpu::DeviceIdentification devType, const std::string& processorPath,
        const std::string& chassisPath);

    void update();

    static size_t decodeLinkWidth(uint32_t value);

  private:
    void initInterfaces(const std::string& name,
                        sdbusplus::asio::object_server& objectServer);

    static constexpr size_t maxTelemetryValues = 64;
    static constexpr uint8_t group1Index = 1;

    void processV2Response(const std::error_code& ec,
                           std::span<const uint8_t> response);

    void processV1Response(const std::error_code& ec,
                           std::span<const uint8_t> response);

    static std::string mapPcieGeneration(uint32_t value);

    uint8_t eid{};

    gpu::DeviceIdentification deviceType{
        gpu::DeviceIdentification::DEVICE_PCIE};

    std::string path;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    std::array<uint8_t, sizeof(gpu::QueryScalarGroupTelemetryV2Request)>
        requestV2{};

    std::array<uint8_t, sizeof(gpu::QueryScalarGroupTelemetryV1Request)>
        requestV1{};

    std::shared_ptr<sdbusplus::asio::dbus_interface> pcieDeviceInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> switchInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;

    std::vector<uint32_t> telemetryValues{maxTelemetryValues};
};
