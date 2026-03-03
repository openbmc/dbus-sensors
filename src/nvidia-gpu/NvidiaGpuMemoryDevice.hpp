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

// GPU DRAM memory inventory for {gpuName}_DRAM_0 inventory object.
// Registers xyz.openbmc_project.Inventory.Item.Dram,
// xyz.openbmc_project.Inventory.Decorator.Location, and
// xyz.openbmc_project.Association.Definitions interfaces. Static properties:
// MemoryType=HBM, ECC=SingleBitECC, LocationType=Embedded. Association:
// GPU (all_memory) <-> DRAM (parent_processor). Fetches MemorySizeInKB via
// MCTP VDM GET_INVENTORY_INFORMATION (property_id=MAX_MEMORY_CAPACITY).
// Call init() after construction (weak_from_this() requires a live shared_ptr).
struct NvidiaGpuMemoryDevice :
    public std::enable_shared_from_this<NvidiaGpuMemoryDevice>
{
  public:
    NvidiaGpuMemoryDevice(mctp::MctpRequester& mctpRequester,
                          const std::string& gpuName, uint8_t eid,
                          sdbusplus::asio::object_server& objectServer);

    void init();

    ~NvidiaGpuMemoryDevice();

  private:
    void fetchMemoryCapacity();
    void processInventoryResponse(const std::error_code& ec,
                                  std::span<const uint8_t> buffer);

    uint8_t eid;
    std::string gpuName;
    mctp::MctpRequester& mctpRequester;
    sdbusplus::asio::object_server& objectServer;

    std::string dramName;
    std::shared_ptr<sdbusplus::asio::dbus_interface> dramItemInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> dramLocationInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> gpuAssociationInterface;

    std::array<uint8_t, sizeof(gpu::GetInventoryInformationRequest)>
        inventoryRequestBuffer{};
};
