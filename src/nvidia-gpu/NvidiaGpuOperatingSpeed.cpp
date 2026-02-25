/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuOperatingSpeed.hpp"

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

static constexpr auto inventoryPrefix = "/xyz/openbmc_project/inventory/";
static constexpr uint8_t graphicsClockId = 0x00;

NvidiaGpuOperatingSpeed::NvidiaGpuOperatingSpeed(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& gpuName, uint8_t eid,
    sdbusplus::asio::object_server& objectServer) :
    eid(eid), gpuName(gpuName), conn(conn), mctpRequester(mctpRequester),
    objectServer(objectServer)
{
    const std::string gpuPath = std::string(inventoryPrefix) + gpuName;

    operatingConfigInterface = objectServer.add_interface(
        gpuPath,
        "xyz.openbmc_project.Inventory.Item.Accelerator.OperatingConfig");
    operatingConfigInterface->register_property("OperatingSpeed",
                                                operatingSpeed);

    if (!operatingConfigInterface->initialize())
    {
        lg2::error("Failed to initialize OperatingConfig interface for {NAME}",
                   "NAME", gpuName);
    }
}

NvidiaGpuOperatingSpeed::~NvidiaGpuOperatingSpeed()
{
    objectServer.remove_interface(operatingConfigInterface);
}

void NvidiaGpuOperatingSpeed::update()
{
    auto rc = gpu::encodeGetCurrentClockFrequencyRequest(
        0, graphicsClockId,
        std::span<uint8_t>(requestBuffer.data(), requestBuffer.size()));

    if (rc != 0)
    {
        lg2::error(
            "Error encoding clock frequency request for {NAME}, eid={EID}, rc={RC}",
            "NAME", gpuName, "EID", eid, "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, requestBuffer,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaGpuOperatingSpeed> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaGpuOperatingSpeed");
                return;
            }
            self->processResponse(ec, buffer);
        });
}

void NvidiaGpuOperatingSpeed::processResponse(const std::error_code& ec,
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
    uint32_t clockFreqMHz = 0;

    auto rc = gpu::decodeGetCurrentClockFrequencyResponse(
        buffer, cc, reasonCode, clockFreqMHz);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding clock frequency response for {NAME}: rc={RC}, cc={CC}, reason={REASON}",
            "NAME", gpuName, "RC", rc, "CC", static_cast<int>(cc), "REASON",
            reasonCode);
        return;
    }

    if (operatingSpeed != clockFreqMHz)
    {
        operatingSpeed = clockFreqMHz;
        operatingConfigInterface->set_property("OperatingSpeed", clockFreqMHz);
    }
}
