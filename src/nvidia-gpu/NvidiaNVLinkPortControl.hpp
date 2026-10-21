/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

// Publishes a Control.Port.LinkState object for every NVLink port of a GPU and
// polls the link state each port will take after the next GPU reset via Nvidia
// MCTP VDM 0x45 (Get Port Disable Future). The device reports this as a single
// mask covering all of its ports, so one instance serves the whole GPU. Each
// object is associated (controlling/controlled_by) with the Connector.Port
// object of its port.
struct NvidiaNVLinkPortControl :
    public std::enable_shared_from_this<NvidiaNVLinkPortControl>
{
  public:
    NvidiaNVLinkPortControl(sdbusplus::asio::object_server& objectServer,
                            mctp::MctpRequester& mctpRequester,
                            const std::string& gpuName, uint8_t eid,
                            uint8_t numberOfPorts);

    ~NvidiaNVLinkPortControl();

    void update();

  private:
    void processResponse(const std::error_code& sendRecvMsgResult,
                         std::span<const uint8_t> response);

    uint8_t eid = 0;

    sdbusplus::asio::object_server& objectServer;

    mctp::MctpRequester& mctpRequester;

    // Indexed by port index; port index i is port number i + 1 on the wire.
    std::vector<std::shared_ptr<sdbusplus::asio::dbus_interface>>
        linkStateInterfaces;

    std::vector<std::shared_ptr<sdbusplus::asio::dbus_interface>>
        associationInterfaces;

    std::array<uint8_t, gpu::getPortDisableFutureRequestSize> request{};

    bool requestEncoded{false};
};
