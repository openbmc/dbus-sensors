/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuDramEccSensor.hpp"

#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"

#include <MctpRequester.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>

// Inventory path prefix - DRAM objects identified by Item.Dimm interface
static constexpr auto inventoryPrefix = "/xyz/openbmc_project/inventory/";

NvidiaGpuDramEccSensor::NvidiaGpuDramEccSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& gpuName, uint8_t eid,
    sdbusplus::asio::object_server& objectServer) :
    eid(eid), name(gpuName), dramName(gpuName + "_DRAM_0"), conn(conn),
    mctpRequester(mctpRequester), objectServer(objectServer)
{
    // Create D-Bus object path for GPU DRAM memory
    std::string dramPath = std::string(inventoryPrefix) + dramName;

    // Create Item.Dimm interface to identify this as a Memory device
    // This allows bmcweb to discover it as part of Memory Collection
    dimmInterface = objectServer.add_interface(
        dramPath, "xyz.openbmc_project.Inventory.Item.Dimm");

    // Set MemoryType to HBM3 (typical for modern NVIDIA GPUs)
    dimmInterface->register_property(
        "MemoryType",
        std::string("xyz.openbmc_project.Inventory.Item.Dimm.DeviceType.HBM3"));

    if (!dimmInterface->initialize())
    {
        lg2::error("Failed to initialize Dimm interface for {NAME}", "NAME",
                   dramName);
    }

    // Create MemoryECC interface for ECC error counts
    memoryEccInterface = objectServer.add_interface(
        dramPath, "xyz.openbmc_project.Memory.MemoryECC");

    // Register ceCount property (correctable ECC errors)
    memoryEccInterface->register_property("ceCount", ceCount);

    // Register ueCount property (uncorrectable ECC errors)
    memoryEccInterface->register_property("ueCount", ueCount);

    if (!memoryEccInterface->initialize())
    {
        lg2::error("Failed to initialize MemoryECC interface for {NAME}",
                   "NAME", dramName);
    }

    lg2::info("Created DRAM ECC sensor for {NAME} at {PATH}", "NAME", dramName,
              "PATH", dramPath);
}

NvidiaGpuDramEccSensor::~NvidiaGpuDramEccSensor()
{
    objectServer.remove_interface(memoryEccInterface);
    objectServer.remove_interface(dimmInterface);
}

void NvidiaGpuDramEccSensor::update()
{
    auto rc = gpu::encodeGetEccErrorCountsRequest(0, requestMsg);

    if (rc != 0)
    {
        lg2::error(
            "Error encoding ECC request for DRAM sensor {NAME}, eid={EID}, rc={RC}",
            "NAME", dramName, "EID", eid, "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, requestMsg,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            auto self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaGpuDramEccSensor");
                return;
            }
            self->processResponse(ec, buffer);
        });
}

void NvidiaGpuDramEccSensor::processResponse(const std::error_code& ec,
                                             std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error(
            "Error updating DRAM ECC sensor {NAME}: MCTP send/recv failed, rc={RC}",
            "NAME", dramName, "RC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    gpu::NsmEccErrorCounts errorCounts{};

    auto rc = gpu::decodeGetEccErrorCountsResponse(buffer, cc, reasonCode,
                                                   errorCounts);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating DRAM ECC sensor {NAME}: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}",
            "NAME", dramName, "RC", rc, "CC", static_cast<int>(cc), "RESC",
            reasonCode);
        return;
    }

    // Update D-Bus properties with DRAM ECC counts
    // Use dram_corrected and dram_uncorrected from NSM response
    ceCount = static_cast<int64_t>(errorCounts.dram_corrected);
    ueCount = static_cast<int64_t>(errorCounts.dram_uncorrected);

    memoryEccInterface->set_property("ceCount", ceCount);
    memoryEccInterface->set_property("ueCount", ueCount);
}
