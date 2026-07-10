/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaPcieInterface.hpp"

#include "Utils.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaUtils.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <vector>

using std::string;

using namespace std::literals;

NvidiaPcieInterface::NvidiaPcieInterface(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& path, uint8_t eid,
    sdbusplus::asio::object_server& objectServer,
    gpu::DeviceIdentification deviceType,
    const std::optional<std::string>& networkAdapterName) :
    eid(eid), path(path), conn(conn), mctpRequester(mctpRequester),
    objectServer(objectServer), deviceType(deviceType)
{
    int rc = 0;
    switch (deviceType)
    {
        case gpu::DeviceIdentification::DEVICE_GPU:
            request.resize(gpu::queryScalarGroupTelemetryV1RequestSize);
            rc = gpu::encodeQueryScalarGroupTelemetryV1Request(
                0, 0, gpu::PcieScalarGroupId::LinkSpeedWidth, request);
            break;
        case gpu::DeviceIdentification::DEVICE_PCIE:
            request.resize(gpu::queryScalarGroupTelemetryV2RequestSize);
            rc = gpu::encodeQueryScalarGroupTelemetryV2Request(
                0, {}, 0, 0, gpu::PcieScalarGroupId::LinkSpeedWidth, request);
            break;
        default:
            break;
    }
    if (rc != 0)
    {
        request.clear();
        lg2::error(
            "Failed to encode PCIe Interface request: rc={RC}, EID={EID}", "RC",
            rc, "EID", eid);
    }

    const sdbusplus::object_path dbusPath = inventoryPrefix / escapeName(name);

    pcieDeviceInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Inventory.Item.PCIeDevice");

    std::vector<Association> associations;
    associations.emplace_back("contained_by", "containing",
                              sdbusplus::object_path(path).parent_path());

    switch (deviceType)
    {
        case gpu::DeviceIdentification::DEVICE_GPU:
            pcieDeviceInterface->register_property(
                "DeviceType",
                std::string("xyz.openbmc_project.Inventory.Item.PCIeDevice."
                            "DeviceTypes.SingleFunction"));
            break;
        case gpu::DeviceIdentification::DEVICE_PCIE:
            switchInterface = objectServer.add_interface(
                dbusPath, "xyz.openbmc_project.Inventory.Item.PCIeSwitch");
            break;
        default:
            break;
    }

    if (networkAdapterName.has_value())
    {
        const sdbusplus::object_path networkAdapterPath =
            inventoryPrefix / (*networkAdapterName + "_NIC");
        associations.emplace_back("connected_to", "connecting",
                                  networkAdapterPath);
    }

    associationInterface =
        objectServer.add_interface(dbusPath, association::interface);
    associationInterface->register_property("Associations", associations);

    pcieDeviceInterface->register_property(
        "GenerationInUse",
        std::string(
            "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Unknown"));

    pcieDeviceInterface->register_property("LanesInUse",
                                           std::numeric_limits<size_t>::max());

    pcieDeviceInterface->register_property(
        "GenerationSupported",
        std::string(
            "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Unknown"));

    pcieDeviceInterface->register_property("MaxLanes", static_cast<size_t>(0));

    if (!pcieDeviceInterface->initialize())
    {
        lg2::error("Error initializing PCIe Device interface, eid={EID}", "EID",
                   eid);
    }

    if (switchInterface && !switchInterface->initialize())
    {
        lg2::error("Error initializing Switch interface, eid={EID}", "EID",
                   eid);
    }

    if (associationInterface && !associationInterface->initialize())
    {
        lg2::error(
            "Error initializing Association interface for PCIeSwitch, eid={EID}",
            "EID", eid);
    }
}

NvidiaPcieInterface::~NvidiaPcieInterface()
{
    objectServer.remove_interface(pcieDeviceInterface);
    if (switchInterface)
    {
        objectServer.remove_interface(switchInterface);
    }
    objectServer.remove_interface(associationInterface);
}

string NvidiaPcieInterface::mapPcieGeneration(uint32_t value)
{
    switch (value)
    {
        case 1:
            return "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Gen1";
        case 2:
            return "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Gen2";
        case 3:
            return "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Gen3";
        case 4:
            return "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Gen4";
        case 5:
            return "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Gen5";
        case 6:
            return "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Gen6";
        default:
            return "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Unknown";
    }
}

size_t NvidiaPcieInterface::decodeLinkWidth(uint32_t value)
{
    return (value > 0) ? pow(2, value - 1) : 0;
}

void NvidiaPcieInterface::processResponse(const std::error_code& ec,
                                          std::span<const uint8_t> response)
{
    if (ec)
    {
        lg2::error(
            "Error updating PCIe Interface: sending message over MCTP failed, "
            "rc={RC}, EID={EID}",
            "RC", ec.value(), "EID", eid);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    size_t numTelemetryValue = 0;

    int rc = 0;
    switch (deviceType)
    {
        case gpu::DeviceIdentification::DEVICE_GPU:
            rc = gpu::decodeQueryScalarGroupTelemetryV1Response(
                response, cc, reasonCode, numTelemetryValue, telemetryValues);
            break;
        case gpu::DeviceIdentification::DEVICE_PCIE:
            rc = gpu::decodeQueryScalarGroupTelemetryV2Response(
                response, cc, reasonCode, numTelemetryValue, telemetryValues);
            break;
        default:
            return;
    }

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error("Error updating PCIe Interface: decode failed, "
                   "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}",
                   "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode,
                   "EID", eid);
        return;
    }

    if (!telemetryValues.empty())
    {
        pcieDeviceInterface->set_property(
            "GenerationInUse", mapPcieGeneration(telemetryValues[0]));
    }

    if (telemetryValues.size() > 1)
    {
        pcieDeviceInterface->set_property(
            "LanesInUse",
            decodeLinkWidth(static_cast<size_t>(telemetryValues[1])));
    }

    if (telemetryValues.size() > 3)
    {
        pcieDeviceInterface->set_property(
            "GenerationSupported", mapPcieGeneration(telemetryValues[3]));
    }

    if (telemetryValues.size() > 4)
    {
        pcieDeviceInterface->set_property(
            "MaxLanes",
            decodeLinkWidth(static_cast<size_t>(telemetryValues[4])));
    }
}

void NvidiaPcieInterface::update()
{
    if (request.empty())
    {
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [eid{this->eid}, weak{weak_from_this()}](
            const std::error_code& ec, std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaPcieInterface> self = weak.lock();
            if (!self)
            {
                lg2::error(
                    "Invalid reference to NvidiaPcieInterface for EID {EID}",
                    "EID", eid);
                return;
            }
            self->processResponse(ec, buffer);
        });
}
