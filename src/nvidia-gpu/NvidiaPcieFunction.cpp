/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaPcieFunction.hpp"

#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaPcieDevice.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <tuple>
#include <vector>

using Association = std::tuple<std::string, std::string, std::string>;

NvidiaPcieFunction::NvidiaPcieFunction(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& pcieDeviceName,
    const std::string& path, uint8_t eid, uint8_t functionNumber,
    sdbusplus::asio::object_server& objectServer,
    gpu::DeviceIdentification deviceType) :
    eid(eid), path(path), conn(conn), mctpRequester(mctpRequester),
    deviceType(deviceType)
{
    const sdbusplus::message::object_path dbusPath =
        sdbusplus::message::object_path(pcieDevicePathPrefix) / pcieDeviceName /
        ("Function" + std::to_string(functionNumber));

    pcieFunctionInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Inventory.Item.PCIeFunction");

    pcieFunctionInterface->register_property("FunctionNumber", functionNumber);
    pcieFunctionInterface->register_property("VendorId",
                                             static_cast<uint16_t>(0));
    pcieFunctionInterface->register_property("DeviceId",
                                             static_cast<uint16_t>(0));
    pcieFunctionInterface->register_property("SubsystemVendorId",
                                             static_cast<uint16_t>(0));
    pcieFunctionInterface->register_property("SubsystemId",
                                             static_cast<uint16_t>(0));
    if (deviceType == gpu::DeviceIdentification::DEVICE_GPU)
    {
        pcieFunctionInterface->register_property(
            "DeviceClass",
            std::string("xyz.openbmc_project.Inventory.Item.PCIeFunction"
                        ".DeviceClasses.ProcessingAccelerators"));
        pcieFunctionInterface->register_property(
            "FunctionType",
            std::string("xyz.openbmc_project.Inventory.Item.PCIeFunction"
                        ".FunctionTypes.Physical"));
    }

    if (!pcieFunctionInterface->initialize())
    {
        lg2::error("Error initializing PCIe Function Interface for EID={EID}",
                   "EID", eid);
    }

    const std::string pcieDevicePath = pcieDevicePathPrefix + pcieDeviceName;

    std::vector<Association> associations;
    associations.emplace_back("exposed_by", "exposing", pcieDevicePath);

    associationInterface =
        objectServer.add_interface(dbusPath, association::interface);
    associationInterface->register_property("Associations", associations);

    if (!associationInterface->initialize())
    {
        lg2::error(
            "Error initializing Association Interface for PCIe Function for EID={EID}",
            "EID", eid);
    }
}

void NvidiaPcieFunction::processResponse(const std::error_code& ec,
                                         std::span<const uint8_t> response)
{
    if (ec)
    {
        lg2::error(
            "Error updating PCIe Function: sending message over MCTP failed, "
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
        lg2::error("Error updating PCIe Function: decode failed, "
                   "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}",
                   "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode,
                   "EID", eid);
        return;
    }

    if (numTelemetryValue < 4)
    {
        lg2::error(
            "Error updating PCIe Function: insufficient telemetry values, "
            "NumValues={NUM}, EID={EID}",
            "NUM", numTelemetryValue, "EID", eid);
        return;
    }

    pcieFunctionInterface->set_property(
        "VendorId", static_cast<uint16_t>(telemetryValues[0]));
    pcieFunctionInterface->set_property(
        "DeviceId", static_cast<uint16_t>(telemetryValues[1]));
    pcieFunctionInterface->set_property(
        "SubsystemVendorId", static_cast<uint16_t>(telemetryValues[2]));
    pcieFunctionInterface->set_property(
        "SubsystemId", static_cast<uint16_t>(telemetryValues[3]));
}

void NvidiaPcieFunction::update()
{
    int rc = 0;
    std::span<uint8_t> buf;

    switch (deviceType)
    {
        case gpu::DeviceIdentification::DEVICE_GPU:
            rc = gpu::encodeQueryScalarGroupTelemetryV1Request(
                0, 0, gpu::PcieScalarGroupId::PciIdentity, requestV1);
            buf = requestV1;
            break;
        case gpu::DeviceIdentification::DEVICE_PCIE:
            rc = gpu::encodeQueryScalarGroupTelemetryV2Request(
                0, {}, 0, 0, gpu::PcieScalarGroupId::PciIdentity, requestV2);
            buf = requestV2;
            break;
        default:
            return;
    }

    if (rc != 0)
    {
        lg2::error(
            "Error updating PCIe Function: encode failed, rc={RC}, EID={EID}",
            "RC", rc, "EID", eid);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, buf,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaPcieFunction> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaPcieFunction");
                return;
            }
            self->processResponse(ec, buffer);
        });
}
