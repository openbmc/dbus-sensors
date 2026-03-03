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
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
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
    mctp::MctpRequester& mctpRequester, const std::string& gpuName, uint8_t eid,
    sdbusplus::asio::object_server& objectServer) :
    eid(eid), gpuName(gpuName), mctpRequester(mctpRequester),
    objectServer(objectServer), dramName(gpuName + "_DRAM_0")
{
    std::string dramPath = std::string(inventoryPrefix) + dramName;
    std::string gpuPath = std::string(inventoryPrefix) + gpuName;

    dramItemInterface = objectServer.add_interface(dramPath, itemDramIfaceName);
    dramItemInterface->register_property("MemoryConfiguredSpeedInMhz",
                                         static_cast<uint16_t>(0));
    dramItemInterface->register_property("AllowedSpeedsMT",
                                         std::vector<uint16_t>(2, 0));

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
    fetchMinMemoryClock();
    fetchMaxMemoryClock();
}

void NvidiaGpuMemoryDevice::update()
{
    fetchCurrentClockFrequency();
}

NvidiaGpuMemoryDevice::~NvidiaGpuMemoryDevice()
{
    objectServer.remove_interface(dramItemInterface);
    objectServer.remove_interface(dramLocationInterface);
    objectServer.remove_interface(gpuAssociationInterface);
}

void NvidiaGpuMemoryDevice::fetchMinMemoryClock()
{
    const int rc = gpu::encodeGetInventoryInformationRequest(
        0, static_cast<uint8_t>(gpu::InventoryPropertyId::MIN_MEMORY_CLOCK),
        inventoryRequestBuffer);
    if (rc != 0)
    {
        lg2::error("Error encoding min clock request for {NAME}: rc={RC}",
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
            self->processMinClockResponse(ec, buffer);
        });
}

void NvidiaGpuMemoryDevice::processMinClockResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("Error fetching min clock for {NAME}: {EC}", "NAME", gpuName,
                   "EC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    gpu::InventoryValue value;

    const int rc = gpu::decodeGetInventoryInformationResponse(
        buffer, cc, reasonCode, gpu::InventoryPropertyId::MIN_MEMORY_CLOCK,
        value);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding min clock for {NAME}: rc={RC}, cc={CC}, reason={REASON}",
            "NAME", gpuName, "RC", rc, "CC", static_cast<uint8_t>(cc), "REASON",
            reasonCode);
        return;
    }

    const auto* minClock = std::get_if<uint32_t>(&value);
    if (minClock == nullptr)
    {
        lg2::error("Unexpected min clock value type for {NAME}", "NAME",
                   gpuName);
        return;
    }

    allowedSpeedsMT[0] = static_cast<uint16_t>(*minClock);
    dramItemInterface->set_property("AllowedSpeedsMT", allowedSpeedsMT);
}

void NvidiaGpuMemoryDevice::fetchMaxMemoryClock()
{
    const int rc = gpu::encodeGetInventoryInformationRequest(
        0, static_cast<uint8_t>(gpu::InventoryPropertyId::MAX_MEMORY_CLOCK),
        inventoryRequestBuffer);
    if (rc != 0)
    {
        lg2::error("Error encoding max clock request for {NAME}: rc={RC}",
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
            self->processMaxClockResponse(ec, buffer);
        });
}

void NvidiaGpuMemoryDevice::processMaxClockResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("Error fetching max clock for {NAME}: {EC}", "NAME", gpuName,
                   "EC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    gpu::InventoryValue value;

    const int rc = gpu::decodeGetInventoryInformationResponse(
        buffer, cc, reasonCode, gpu::InventoryPropertyId::MAX_MEMORY_CLOCK,
        value);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding max clock for {NAME}: rc={RC}, cc={CC}, reason={REASON}",
            "NAME", gpuName, "RC", rc, "CC", static_cast<uint8_t>(cc), "REASON",
            reasonCode);
        return;
    }

    const auto* maxClock = std::get_if<uint32_t>(&value);
    if (maxClock == nullptr)
    {
        lg2::error("Unexpected max clock value type for {NAME}", "NAME",
                   gpuName);
        return;
    }

    allowedSpeedsMT[1] = static_cast<uint16_t>(*maxClock);
    dramItemInterface->set_property("AllowedSpeedsMT", allowedSpeedsMT);
}

void NvidiaGpuMemoryDevice::fetchCurrentClockFrequency()
{
    const int rc = gpu::encodeGetCurrentClockFrequencyRequest(
        0, gpu::ClockType::MEMORY_CLOCK, clockFreqRequestBuffer);
    if (rc != 0)
    {
        lg2::error("Error encoding clock freq request for {NAME}: rc={RC}",
                   "NAME", gpuName, "RC", rc);
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

void NvidiaGpuMemoryDevice::processClockFreqResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("Error fetching clock freq for {NAME}: {EC}", "NAME",
                   gpuName, "EC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint32_t clockFreq = 0;

    const int rc = gpu::decodeGetCurrentClockFrequencyResponse(
        buffer, cc, reasonCode, clockFreq);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding clock freq for {NAME}: rc={RC}, cc={CC}, reason={REASON}",
            "NAME", gpuName, "RC", rc, "CC", static_cast<uint8_t>(cc), "REASON",
            reasonCode);
        return;
    }

    dramItemInterface->set_property("MemoryConfiguredSpeedInMhz",
                                    static_cast<uint16_t>(clockFreq));
}
