/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"

#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

// GPU DRAM memory speed for {gpuName}_DRAM_0 inventory object.
// Registers Item.Dram (MemoryConfiguredSpeedInMhz, AllowedSpeedsMT),
// Decorator.Location (LocationType=Embedded), and Association.Definitions
// (all_memory / parent_processor). AllowedSpeedsMT is fetched once at init()
// via MCTP VDM GET_INVENTORY_INFORMATION (MIN/MAX_MEMORY_CLOCK).
// MemoryConfiguredSpeedInMhz is polled via GET_CURRENT_CLOCK_FREQUENCY.
// Call init() after construction.
struct NvidiaGpuMemoryDevice :
    public std::enable_shared_from_this<NvidiaGpuMemoryDevice>
{
  public:
    NvidiaGpuMemoryDevice(mctp::MctpRequester& mctpRequester,
                          const std::string& gpuName, uint8_t eid,
                          sdbusplus::asio::object_server& objectServer);

    void init();
    void update();

    ~NvidiaGpuMemoryDevice();

  private:
    void fetchMinMemoryClock();
    void fetchMaxMemoryClock();
    void fetchCurrentClockFrequency();

    void processMinClockResponse(const std::error_code& ec,
                                 std::span<const uint8_t> buffer);
    void processMaxClockResponse(const std::error_code& ec,
                                 std::span<const uint8_t> buffer);
    void processClockFreqResponse(const std::error_code& ec,
                                  std::span<const uint8_t> buffer);

    uint8_t eid;
    std::string gpuName;
    mctp::MctpRequester& mctpRequester;
    sdbusplus::asio::object_server& objectServer;

    std::string dramName;
    std::shared_ptr<sdbusplus::asio::dbus_interface> dramItemInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> dramLocationInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> gpuAssociationInterface;

    std::vector<uint16_t> allowedSpeedsMT{0, 0};

    std::array<uint8_t, sizeof(gpu::GetInventoryInformationRequest)>
        inventoryRequestBuffer{};
    std::array<uint8_t, sizeof(gpu::GetCurrentClockFrequencyRequest)>
        clockFreqRequestBuffer{};
};
