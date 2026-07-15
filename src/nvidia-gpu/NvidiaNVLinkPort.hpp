/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

// Publishes an xyz.openbmc_project.Inventory.Connector.Port object for a single
// GPU NVLink port and polls its status (NSM 0x43) and characteristics
// (NSM 0x42).
struct NvidiaNVLinkPort : public std::enable_shared_from_this<NvidiaNVLinkPort>
{
  public:
    NvidiaNVLinkPort(std::shared_ptr<sdbusplus::asio::connection>& conn,
                     mctp::MctpRequester& mctpRequester,
                     const std::string& gpuName, uint8_t eid, uint8_t portIndex,
                     sdbusplus::asio::object_server& objectServer);

    void update();

  private:
    void sendQueryPortStatus();

    void processPortStatusResponse(const std::error_code& sendRecvMsgResult,
                                   std::span<const uint8_t> response);

    void sendQueryPortCharacteristics();

    void processPortCharacteristicsResponse(
        const std::error_code& sendRecvMsgResult,
        std::span<const uint8_t> response);

    uint8_t eid = 0;

    // 1-based port number on the wire.
    uint8_t portNumber = 0;

    std::string gpuName;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    std::vector<uint8_t> statusRequest;

    std::vector<uint8_t> characteristicsRequest;

    std::shared_ptr<sdbusplus::asio::dbus_interface> portInterface;

    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;
};
