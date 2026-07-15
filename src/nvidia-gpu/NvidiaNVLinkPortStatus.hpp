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
#include <system_error>

// Polls the status (Nvidia MCTP VDM 0x43) of a single GPU NVLink port and
// publishes it as the LinkStatus and LinkState properties of the port's
// Inventory.Connector.Port interface, which the GPU device owns.
struct NvidiaNVLinkPortStatus :
    public std::enable_shared_from_this<NvidiaNVLinkPortStatus>
{
  public:
    NvidiaNVLinkPortStatus(
        mctp::MctpRequester& mctpRequester, uint8_t eid, uint8_t portIndex,
        const std::shared_ptr<sdbusplus::asio::dbus_interface>& portInterface);

    void update();

  private:
    void processResponse(const std::error_code& sendRecvMsgResult,
                         std::span<const uint8_t> response);

    uint8_t eid = 0;

    // 1-based port number on the wire.
    uint8_t portNumber = 0;

    mctp::MctpRequester& mctpRequester;

    std::shared_ptr<sdbusplus::asio::dbus_interface> portInterface;

    std::array<uint8_t, gpu::queryPortStatusRequestSize> request{};

    bool requestEncoded{false};
};
