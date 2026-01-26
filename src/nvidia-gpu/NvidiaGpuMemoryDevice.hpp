/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"

#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <string>

/**
 * @brief NVIDIA GPU Memory Device
 *
 * Manages GPU memory-related D-Bus objects and interfaces.
 * Currently supports SRAM ECC error monitoring.
 *
 * SRAM ECC D-Bus:
 *   Object Path: /xyz/openbmc_project/inventory/{gpuName}
 *   Interface: xyz.openbmc_project.Memory.MemoryECC (ceCount, ueCount)
 */
struct NvidiaGpuMemoryDevice :
    public std::enable_shared_from_this<NvidiaGpuMemoryDevice>
{
  public:
    /**
     * @brief Construct GPU Memory Device
     *
     * @param conn          D-Bus connection
     * @param mctpRequester MCTP requester for GPU communication
     * @param gpuName       GPU device name (e.g., "GPU_0_0")
     * @param eid           MCTP endpoint ID for this GPU
     * @param objectServer  D-Bus object server
     */
    NvidiaGpuMemoryDevice(std::shared_ptr<sdbusplus::asio::connection>& conn,
                          mctp::MctpRequester& mctpRequester,
                          const std::string& gpuName, uint8_t eid,
                          sdbusplus::asio::object_server& objectServer);

    ~NvidiaGpuMemoryDevice() = default;

    /**
     * @brief Update ECC error counts from GPU
     *
     * Sends NSM GET_ECC_ERROR_COUNTS request to GPU and updates
     * SRAM ECC properties on D-Bus.
     */
    void update();

  private:
    /**
     * @brief Process NSM response and update D-Bus properties
     */
    void processResponse(const std::error_code& ec,
                         std::span<const uint8_t> buffer);

    uint8_t eid;
    std::string gpuName;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    mctp::MctpRequester& mctpRequester;

    // SRAM ECC interface (on GPU inventory object)
    std::shared_ptr<sdbusplus::asio::dbus_interface> sramEccInterface;

    // NSM request buffer
    std::array<uint8_t, sizeof(gpu::GetEccErrorCountsRequest)> requestBuffer{};
};
