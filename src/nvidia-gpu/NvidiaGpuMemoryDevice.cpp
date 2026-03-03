/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuMemoryDevice.hpp"

#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"
#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <variant>
#include <vector>

static constexpr auto inventoryPrefix = "/xyz/openbmc_project/inventory/";
static constexpr auto itemDramIfaceName =
    "xyz.openbmc_project.Inventory.Item.Dram";
static constexpr auto locationIfaceName =
    "xyz.openbmc_project.Inventory.Decorator.Location";

NvidiaGpuMemoryDevice::NvidiaGpuMemoryDevice(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& gpuName, uint8_t eid,
    sdbusplus::asio::object_server& objectServer) :
    eid(eid), gpuName(gpuName), conn(conn), mctpRequester(mctpRequester),
    objectServer(objectServer), dramName(gpuName + "_DRAM_0")
{
    std::string gpuPath = std::string(inventoryPrefix) + gpuName;
    std::string dramPath = std::string(inventoryPrefix) + dramName;

    sramEccInterface = objectServer.add_interface(
        gpuPath, "xyz.openbmc_project.Memory.MemoryECC");

    sramEccInterface->register_property("ceCount", sramCeCount);
    sramEccInterface->register_property("ueCount", sramUeCount);

    if (!sramEccInterface->initialize())
    {
        lg2::error("Failed to initialize SRAM ECC interface for {NAME}", "NAME",
                   gpuName);
    }

    lg2::info("Created SRAM ECC interface for {NAME} at {PATH}", "NAME",
              gpuName, "PATH", gpuPath);

    dramItemInterface = objectServer.add_interface(dramPath, itemDramIfaceName);
    dramItemInterface->register_property(
        "MemoryType",
        std::string("xyz.openbmc_project.Inventory.Item.Dram.DeviceType.HBM"));
    dramItemInterface->register_property(
        "ECC", std::string(
                   "xyz.openbmc_project.Inventory.Item.Dram.Ecc.SingleBitECC"));
    dramItemInterface->register_property("MemorySizeInKB", size_t{0});

    if (!dramItemInterface->initialize())
    {
        lg2::error("Failed to initialize Dram interface for {NAME}", "NAME",
                   dramName);
    }

    dramLocationInterface =
        objectServer.add_interface(dramPath, locationIfaceName);
    dramLocationInterface->register_property(
        "LocationType",
        std::string("xyz.openbmc_project.Inventory.Decorator.Location"
                    ".LocationTypes.Embedded"));

    if (!dramLocationInterface->initialize())
    {
        lg2::error("Failed to initialize Location interface for {NAME}", "NAME",
                   dramName);
    }

    std::vector<Association> associations;
    associations.emplace_back("all_memory", "parent_processor", dramPath);

    gpuAssociationInterface =
        objectServer.add_interface(gpuPath, association::interface);
    gpuAssociationInterface->register_property("Associations", associations);

    if (!gpuAssociationInterface->initialize())
    {
        lg2::error("Failed to initialize Association interface for {NAME}",
                   "NAME", gpuName);
    }
}

void NvidiaGpuMemoryDevice::init()
{
    fetchMemoryCapacity();
}

NvidiaGpuMemoryDevice::~NvidiaGpuMemoryDevice()
{
    objectServer.remove_interface(sramEccInterface);
    objectServer.remove_interface(dramItemInterface);
    objectServer.remove_interface(dramLocationInterface);
    objectServer.remove_interface(gpuAssociationInterface);
}

void NvidiaGpuMemoryDevice::update()
{
    auto rc = gpu::encodeGetEccErrorCountsRequest(0, requestBuffer);

    if (rc != 0)
    {
        lg2::error("Error encoding ECC request for {NAME}, eid={EID}, rc={RC}",
                   "NAME", gpuName, "EID", eid, "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, requestBuffer,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaGpuMemoryDevice> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaGpuMemoryDevice");
                return;
            }
            self->processResponse(ec, buffer);
        });
}

void NvidiaGpuMemoryDevice::processResponse(const std::error_code& ec,
                                            std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("MCTP error for {NAME}: {EC}", "NAME", gpuName, "EC",
                   ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    gpu::GetEccErrorCountsResponse eccResponse{};

    auto rc = gpu::decodeGetEccErrorCountsResponse(buffer, cc, reasonCode,
                                                   eccResponse);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding ECC response for {NAME}: rc={RC}, cc={CC}, reason={REASON}",
            "NAME", gpuName, "RC", rc, "CC", static_cast<int>(cc), "REASON",
            reasonCode);
        return;
    }

    sramCeCount = eccResponse.sram_corrected;
    sramUeCount = (eccResponse.sram_uncorrected_secded +
                   eccResponse.sram_uncorrected_parity);

    sramEccInterface->set_property("ceCount", sramCeCount);
    sramEccInterface->set_property("ueCount", sramUeCount);
}

void NvidiaGpuMemoryDevice::fetchMemoryCapacity()
{
    const int rc = gpu::encodeGetInventoryInformationRequest(
        0, static_cast<uint8_t>(gpu::InventoryPropertyId::MAX_MEMORY_CAPACITY),
        inventoryRequestBuffer);
    if (rc != 0)
    {
        lg2::error("Error encoding inventory request for {NAME}: rc={RC}",
                   "NAME", gpuName, "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, inventoryRequestBuffer,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaGpuMemoryDevice> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaGpuMemoryDevice");
                return;
            }
            self->processInventoryResponse(ec, buffer);
        });
}

void NvidiaGpuMemoryDevice::processInventoryResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("Error fetching memory capacity for {NAME}: {EC}", "NAME",
                   gpuName, "EC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    gpu::InventoryValue value;

    const int rc = gpu::decodeGetInventoryInformationResponse(
        buffer, cc, reasonCode, gpu::InventoryPropertyId::MAX_MEMORY_CAPACITY,
        value);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding memory capacity response for {NAME}: rc={RC}, cc={CC}, reason={REASON}",
            "NAME", gpuName, "RC", rc, "CC", static_cast<uint8_t>(cc), "REASON",
            reasonCode);
        return;
    }

    const auto* capacityMib = std::get_if<uint32_t>(&value);
    if (capacityMib == nullptr)
    {
        lg2::error("Unexpected inventory value type for {NAME}", "NAME",
                   gpuName);
        return;
    }

    const size_t memorySizeInKB = static_cast<size_t>(*capacityMib) * 1024;
    dramItemInterface->set_property("MemorySizeInKB", memorySizeInKB);
}
