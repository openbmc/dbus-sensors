/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuEccSensor.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <system_error>

constexpr const char* inventoryPrefix = "/xyz/openbmc_project/inventory/";

NvidiaGpuEccSensor::NvidiaGpuEccSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name, uint8_t eid,
    sdbusplus::asio::object_server& objectServer) :
    eid(eid), name(name), conn(conn), mctpRequester(mctpRequester)
{
    // Construct D-Bus object path (using existing inventory path)
    std::string eccPath = std::string(inventoryPrefix) + name;

    // Create Memory.MemoryECC D-Bus interface (added to existing path)
    memoryEccInterface = objectServer.add_interface(
        eccPath, "xyz.openbmc_project.Memory.MemoryECC");

    // Register ceCount property (Correctable ECC Error Count)
    memoryEccInterface->register_property(
        "ceCount",
        static_cast<int64_t>(0), // Initial value 0
        sdbusplus::asio::PropertyPermission::readOnly);

    // Register ueCount property (Uncorrectable ECC Error Count)
    memoryEccInterface->register_property(
        "ueCount",
        static_cast<int64_t>(0), // Initial value 0
        sdbusplus::asio::PropertyPermission::readOnly);

    // Initialize interface (publish to D-Bus)
    memoryEccInterface->initialize();

    lg2::info("NvidiaGpuEccSensor created for {NAME} at path {PATH}", "NAME",
              name, "PATH", eccPath);
}

void NvidiaGpuEccSensor::update()
{
    // Encode NSM GET_ECC_ERROR_COUNTS request
    uint8_t instanceId = 0;
    int rc = gpu::encodeGetEccErrorCountsRequest(instanceId, requestBuffer);

    if (rc != 0)
    {
        lg2::error("Failed to encode ECC error counts request for {NAME}: {RC}",
                   "NAME", name, "RC", rc);
        return;
    }

    // Send request to GPU via MCTP
    mctpRequester.sendRecvMsg(
        eid, requestBuffer,
        std::bind_front(&NvidiaGpuEccSensor::processResponse,
                        shared_from_this()));
}

void NvidiaGpuEccSensor::processResponse(const std::error_code& ec,
                                         std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("MCTP communication error for {NAME}: {EC}", "NAME", name,
                   "EC", ec.message());
        return;
    }

    // Decode NSM response
    ocp::accelerator_management::CompletionCode cc =
        ocp::accelerator_management::CompletionCode::SUCCESS;
    uint16_t reasonCode = 0;
    gpu::NsmEccErrorCounts errorCounts{};

    int rc = gpu::decodeGetEccErrorCountsResponse(buffer, cc, reasonCode,
                                                  errorCounts);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error("Failed to decode ECC response for {NAME}: rc={RC}, "
                   "cc={CC}, reason={REASON}",
                   "NAME", name, "RC", rc, "CC", static_cast<int>(cc), "REASON",
                   reasonCode);
        return;
    }

    // Calculate ceCount and ueCount
    // ceCount: SRAM correctable errors only
    int64_t ceCount = errorCounts.sram_corrected;

    // ueCount: Sum of all uncorrectable errors (SRAM SECDED + Parity)
    int64_t ueCount = errorCounts.sram_uncorrected_secded +
                      errorCounts.sram_uncorrected_parity;

    // Update D-Bus properties
    memoryEccInterface->set_property("ceCount", ceCount);
    memoryEccInterface->set_property("ueCount", ueCount);

    lg2::debug("Updated ECC counts for {NAME}: ceCount={CE}, ueCount={UE}",
               "NAME", name, "CE", ceCount, "UE", ueCount);
}
