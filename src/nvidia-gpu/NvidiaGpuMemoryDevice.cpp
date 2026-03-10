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
#include <vector>

static constexpr auto locationIfaceName =
    "xyz.openbmc_project.Inventory.Decorator.Location";
static constexpr auto inventoryPrefix = "/xyz/openbmc_project/inventory/";

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

    dramItemInterface = objectServer.add_interface(
        dramPath, "xyz.openbmc_project.Inventory.Item.Dram");

    dramItemInterface->register_property(
        "MemoryType",
        std::string("xyz.openbmc_project.Inventory.Item.Dram.DeviceType.HBM"));
    dramItemInterface->register_property(
        "ECC", std::string(
                   "xyz.openbmc_project.Inventory.Item.Dram.Ecc.SingleBitECC"));
    dramItemInterface->register_property("MemorySizeInKB", size_t{0});
    dramItemInterface->register_property("MemoryConfiguredSpeedInMhz",
                                         uint32_t{0});
    dramItemInterface->register_property("AllowedSpeedsMHz_Min", uint32_t{0});
    dramItemInterface->register_property("AllowedSpeedsMHz_Max", uint32_t{0});

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

    dramEccInterface = objectServer.add_interface(
        dramPath, "xyz.openbmc_project.Memory.MemoryECC");

    dramEccInterface->register_property("ceCount", dramCeCount);
    dramEccInterface->register_property("ueCount", dramUeCount);

    if (!dramEccInterface->initialize())
    {
        lg2::error("Failed to initialize DRAM ECC interface for {NAME}", "NAME",
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

NvidiaGpuMemoryDevice::~NvidiaGpuMemoryDevice()
{
    objectServer.remove_interface(sramEccInterface);
    objectServer.remove_interface(dramItemInterface);
    objectServer.remove_interface(dramEccInterface);
    objectServer.remove_interface(dramLocationInterface);
    objectServer.remove_interface(gpuAssociationInterface);
}

void NvidiaGpuMemoryDevice::update()
{
    auto rc = gpu::encodeGetEccErrorCountsRequest(0, eccRequestBuffer);

    if (rc != 0)
    {
        lg2::error("Error encoding ECC request for {NAME}, eid={EID}, rc={RC}",
                   "NAME", gpuName, "EID", eid, "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, eccRequestBuffer,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaGpuMemoryDevice> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaGpuMemoryDevice");
                return;
            }
            self->processEccResponse(ec, buffer);
        });

    auto rc2 = gpu::encodeGetCurrentClockFrequencyRequest(
        0, gpu::ClockType::MEMORY_CLOCK,
        std::span<uint8_t>(clockFreqRequestBuffer.data(),
                           clockFreqRequestBuffer.size()));

    if (rc2 != 0)
    {
        lg2::error(
            "Error encoding memory clock request for {NAME}, eid={EID}, rc={RC}",
            "NAME", gpuName, "EID", eid, "RC", rc2);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, clockFreqRequestBuffer,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaGpuMemoryDevice> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaGpuMemoryDevice");
                return;
            }
            self->processClockFreqResponse(ec, buffer);
        });
}

void NvidiaGpuMemoryDevice::processEccResponse(const std::error_code& ec,
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

    dramCeCount = eccResponse.dram_corrected;
    dramUeCount = eccResponse.dram_uncorrected;

    dramEccInterface->set_property("ceCount", dramCeCount);
    dramEccInterface->set_property("ueCount", dramUeCount);
}

void NvidiaGpuMemoryDevice::processClockFreqResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("MCTP error for {NAME} memory clock: {EC}", "NAME", gpuName,
                   "EC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint32_t clockFreq = 0;

    auto rc = gpu::decodeGetCurrentClockFrequencyResponse(
        buffer, cc, reasonCode, clockFreq);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding memory clock response for {NAME}: rc={RC}, cc={CC}, reason={REASON}",
            "NAME", gpuName, "RC", rc, "CC", static_cast<int>(cc), "REASON",
            reasonCode);
        return;
    }

    if (memoryClockMHz != clockFreq)
    {
        memoryClockMHz = clockFreq;
        dramItemInterface->set_property("MemoryConfiguredSpeedInMhz",
                                        clockFreq);
    }
}
