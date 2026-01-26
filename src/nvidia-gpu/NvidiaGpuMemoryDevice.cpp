/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuMemoryDevice.hpp"

#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"

#include <MctpRequester.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <system_error>

static constexpr auto inventoryPrefix = "/xyz/openbmc_project/inventory/";

NvidiaGpuMemoryDevice::NvidiaGpuMemoryDevice(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& gpuName, uint8_t eid,
    sdbusplus::asio::object_server& objectServer) :
    eid(eid), gpuName(gpuName), conn(conn), mctpRequester(mctpRequester),
    objectServer(objectServer)
{
    std::string gpuPath = std::string(inventoryPrefix) + gpuName;

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
}

NvidiaGpuMemoryDevice::~NvidiaGpuMemoryDevice()
{
    objectServer.remove_interface(sramEccInterface);
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

    std::weak_ptr<NvidiaGpuMemoryDevice> weakSelf = weak_from_this();
    mctpRequester.sendRecvMsg(
        eid, requestBuffer,
        [weakSelf](const std::error_code& ec, std::span<const uint8_t> buffer) {
            if (auto self = weakSelf.lock())
            {
                self->processResponse(ec, buffer);
            }
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
    gpu::NsmEccErrorCounts errorCounts{};

    auto rc = gpu::decodeGetEccErrorCountsResponse(buffer, cc, reasonCode,
                                                   errorCounts);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding ECC response for {NAME}: rc={RC}, cc={CC}, reason={REASON}",
            "NAME", gpuName, "RC", rc, "CC", static_cast<int>(cc), "REASON",
            reasonCode);
        return;
    }

    sramCeCount = errorCounts.sram_corrected;
    sramUeCount = (errorCounts.sram_uncorrected_secded +
                   errorCounts.sram_uncorrected_parity);

    sramEccInterface->set_property("ceCount", sramCeCount);
    sramEccInterface->set_property("ueCount", sramUeCount);
}
