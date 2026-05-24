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
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

struct NvidiaDriverInformation :
    public std::enable_shared_from_this<NvidiaDriverInformation>
{
  public:
    NvidiaDriverInformation(
        std::shared_ptr<sdbusplus::asio::connection>& conn,
        mctp::MctpRequester& mctpRequester, const std::string& name,
        uint8_t eid, sdbusplus::asio::object_server& objectServer,
        const sdbusplus::object_path& associationEndpoint,
        const std::optional<std::string>& manufacturer = std::nullopt);

    void update();

  private:
    void processResponse(const std::error_code& ec,
                         std::span<const uint8_t> buffer);

    uint8_t eid{};

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    std::array<uint8_t, ocp::accelerator_management::commonRequestSize>
        request{};

    bool requestEncoded{false};

    std::shared_ptr<sdbusplus::asio::dbus_interface> versionInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> assetInterface;
};
