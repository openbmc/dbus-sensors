/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"

#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <string>

/**
 * @brief DRAM ECC sensor for GPU DRAM memory error monitoring.
 *
 * Creates a Memory D-Bus object with MemoryECC interface to expose
 * GPU DRAM correctable/uncorrectable error counts via Redfish MemoryMetrics.
 *
 * D-Bus Object Path: /xyz/openbmc_project/inventory/system/memory/<gpu>_DRAM_0
 * Interfaces:
 *   - xyz.openbmc_project.Inventory.Item.Dimm (identifies as Memory device)
 *   - xyz.openbmc_project.Memory.MemoryECC (ceCount, ueCount properties)
 */
struct NvidiaGpuDramEccSensor :
    public std::enable_shared_from_this<NvidiaGpuDramEccSensor>
{
  public:
    NvidiaGpuDramEccSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                           mctp::MctpRequester& mctpRequester,
                           const std::string& gpuName, uint8_t eid,
                           sdbusplus::asio::object_server& objectServer);

    ~NvidiaGpuDramEccSensor();

    /**
     * @brief Sends NSM request to get ECC error counts from GPU.
     */
    void update();

  private:
    /**
     * @brief Processes NSM response and updates D-Bus properties.
     */
    void processResponse(const std::error_code& ec,
                         std::span<const uint8_t> buffer);

    uint8_t eid;
    std::string name;
    std::string dramName;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    mctp::MctpRequester& mctpRequester;
    sdbusplus::asio::object_server& objectServer;

    // D-Bus interfaces
    std::shared_ptr<sdbusplus::asio::dbus_interface> dimmInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> memoryEccInterface;

    // NSM request buffer
    std::array<uint8_t, sizeof(gpu::GetEccErrorCountsRequest)> requestMsg{};

    // Cached ECC counts
    int64_t ceCount{0};
    int64_t ueCount{0};
};
