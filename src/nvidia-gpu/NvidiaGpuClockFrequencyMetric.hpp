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
#include <string>

class NvidiaGpuClockFrequencyMetric :
    public std::enable_shared_from_this<NvidiaGpuClockFrequencyMetric>
{
  public:
    NvidiaGpuClockFrequencyMetric(mctp::MctpRequester& mctpRequester,
                                  const std::string& name, uint8_t eid,
                                  sdbusplus::asio::object_server& objectServer,
                                  const std::string& inventoryPath);

    ~NvidiaGpuClockFrequencyMetric();

    void update();

  private:
    void handleResponse(const std::error_code& ec,
                        std::span<const uint8_t> buffer);

    mctp::MctpRequester& mctpRequester;
    std::string name;
    uint8_t eid;
    sdbusplus::asio::object_server& objectServer;
    std::shared_ptr<sdbusplus::asio::dbus_interface> metricInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;
    std::array<uint8_t, gpu::getClockFrequencyRequestSize> requestBuffer{};
};
