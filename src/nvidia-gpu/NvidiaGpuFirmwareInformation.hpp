/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>

struct NvidiaGpuFirmwareInformation :
    public std::enable_shared_from_this<NvidiaGpuFirmwareInformation>
{
  public:
    NvidiaGpuFirmwareInformation(
        mctp::MctpRequester& mctpRequester, const std::string& name,
        const sdbusplus::object_path& path, uint8_t eid,
        sdbusplus::asio::object_server& objectServer);

    void update();

  private:
    void processResponse(const std::error_code& ec,
                         std::span<const uint8_t> buffer);

    uint8_t eid{};

    mctp::MctpRequester& mctpRequester;

    std::array<uint8_t, gpu::getInventoryInformationRequestSize> request{};

    std::shared_ptr<sdbusplus::asio::dbus_interface> versionInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;
};
