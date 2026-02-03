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

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

static constexpr auto inventoryPrefix = "/xyz/openbmc_project/inventory/";
static constexpr auto itemDimmIfaceName =
    "xyz.openbmc_project.Inventory.Item.Dimm";
static constexpr auto memoryEccIfaceName =
    "xyz.openbmc_project.Memory.MemoryECC";

NvidiaGpuMemoryDevice::NvidiaGpuMemoryDevice(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& gpuName, uint8_t eid,
    sdbusplus::asio::object_server& objectServer) :
    eid(eid), gpuName(gpuName), conn(conn), mctpRequester(mctpRequester),
    objectServer(objectServer), dramName(gpuName + "_DRAM_0")
{
    std::string gpuPath = std::string(inventoryPrefix) + gpuName;

    sramEccInterface = objectServer.add_interface(gpuPath, memoryEccIfaceName);
    sramEccInterface->register_property("ceCount", sramCeCount);
    sramEccInterface->register_property("ueCount", sramUeCount);

    if (!sramEccInterface->initialize())
    {
        lg2::error("Failed to initialize SRAM ECC interface for {NAME}", "NAME",
                   gpuName);
    }

    std::string dramPath = std::string(inventoryPrefix) + dramName;

    dramItemInterface = objectServer.add_interface(dramPath, itemDimmIfaceName);
    dramItemInterface->initialize();

    dramEccInterface = objectServer.add_interface(dramPath, memoryEccIfaceName);
    dramEccInterface->register_property("ceCount", dramCeCount);
    dramEccInterface->register_property("ueCount", dramUeCount);

    if (!dramEccInterface->initialize())
    {
        lg2::error("Failed to initialize DRAM ECC interface for {NAME}", "NAME",
                   dramName);
    }

    std::vector<Association> gpuAssociations;
    gpuAssociations.emplace_back("all_memory", "parent_processor", dramPath);

    gpuAssociationInterface =
        objectServer.add_interface(gpuPath, association::interface);
    gpuAssociationInterface->register_property("Associations", gpuAssociations);

    if (!gpuAssociationInterface->initialize())
    {
        lg2::error("Failed to initialize GPU Association interface for {NAME}",
                   "NAME", gpuName);
    }
}

NvidiaGpuMemoryDevice::~NvidiaGpuMemoryDevice()
{
    objectServer.remove_interface(sramEccInterface);
    objectServer.remove_interface(dramItemInterface);
    objectServer.remove_interface(dramEccInterface);
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

    dramCeCount = eccResponse.dram_corrected;
    dramUeCount = eccResponse.dram_uncorrected;

    dramEccInterface->set_property("ceCount", dramCeCount);
    dramEccInterface->set_property("ueCount", dramUeCount);
}
