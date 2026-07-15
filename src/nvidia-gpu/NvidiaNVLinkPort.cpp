/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaNVLinkPort.hpp"

#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaUtils.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

namespace
{

std::string mapPortStateToLinkStatus(uint8_t portState)
{
    const char* linkStatus = nullptr;

    switch (portState)
    {
        case 2:
            linkStatus = "LinkUp";
            break;
        case 6:
            linkStatus = "Starting";
            break;
        case 7:
        case 10:
            linkStatus = "Training";
            break;
        case 1:
        case 4:
        case 5:
        case 8:
        case 9:
            linkStatus = "LinkDown";
            break;
        case 3:
        default:
            linkStatus = "NoLink";
            break;
    }

    return std::string(
               "xyz.openbmc_project.Inventory.Connector.Port.LinkStatus.") +
           linkStatus;
}

std::string mapPortStatusToLinkState(uint8_t portStatus)
{
    const char* linkState = nullptr;

    switch (portStatus)
    {
        case 1:
            linkState = "Disabled";
            break;
        case 2:
            linkState = "Enabled";
            break;
        default:
            linkState = "Unknown";
            break;
    }

    return std::string(
               "xyz.openbmc_project.Inventory.Connector.Port.LinkState.") +
           linkState;
}

} // namespace

NvidiaNVLinkPort::NvidiaNVLinkPort(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& gpuName, uint8_t eid,
    uint8_t portIndex, sdbusplus::asio::object_server& objectServer) :
    eid(eid), portNumber(static_cast<uint8_t>(portIndex + 1)), gpuName(gpuName),
    conn(conn), mctpRequester(mctpRequester), objectServer(objectServer)
{
    if (gpu::encodeQueryPortStatusRequest(0, portNumber, statusRequest) != 0)
    {
        lg2::error(
            "Failed to encode NVLink Port status request, eid={EID}, portNumber={PN}",
            "EID", eid, "PN", portNumber);
    }
    else
    {
        statusRequestEncoded = true;
    }

    if (gpu::encodeQueryPortCharacteristicsRequest(0, portNumber,
                                                   characteristicsRequest) != 0)
    {
        lg2::error(
            "Failed to encode NVLink Port characteristics request, eid={EID}, portNumber={PN}",
            "EID", eid, "PN", portNumber);
    }
    else
    {
        characteristicsRequestEncoded = true;
    }

    const std::string portName = std::format("NVLink_{}", portIndex);

    const sdbusplus::object_path gpuPath = inventoryPrefix / gpuName;
    const sdbusplus::object_path dbusPath = gpuPath / portName;

    portInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Inventory.Connector.Port");

    portInterface->register_property(
        "PortProtocol",
        std::string("xyz.openbmc_project.Inventory.Connector.Port."
                    "PortProtocol.NVLink"));

    portInterface->register_property(
        "PortType", std::string("xyz.openbmc_project.Inventory.Connector.Port."
                                "PortType.Bidirectional"));

    portInterface->register_property("Speed",
                                     std::numeric_limits<uint64_t>::max());

    portInterface->register_property("MaxSpeed",
                                     std::numeric_limits<uint64_t>::max());

    portInterface->register_property("Width",
                                     std::numeric_limits<size_t>::max());

    portInterface->register_property(
        "LinkStatus",
        std::string("xyz.openbmc_project.Inventory.Connector.Port."
                    "LinkStatus.NoLink"));

    portInterface->register_property(
        "LinkState", std::string("xyz.openbmc_project.Inventory.Connector.Port."
                                 "LinkState.Unknown"));

    if (!portInterface->initialize())
    {
        lg2::error(
            "Error initializing NVLink Port interface, eid={EID}, portNumber={PN}",
            "EID", eid, "PN", portNumber);
    }

    std::vector<Association> associations;
    associations.emplace_back("connected_to", "connecting", gpuPath);

    associationInterface =
        objectServer.add_interface(dbusPath, association::interface);
    associationInterface->register_property("Associations", associations);

    if (!associationInterface->initialize())
    {
        lg2::error(
            "Error initializing Association interface for NVLink Port, eid={EID}, portNumber={PN}",
            "EID", eid, "PN", portNumber);
    }
}

NvidiaNVLinkPort::~NvidiaNVLinkPort()
{
    objectServer.remove_interface(portInterface);
    objectServer.remove_interface(associationInterface);
}

void NvidiaNVLinkPort::update()
{
    sendQueryPortStatus();
    sendQueryPortCharacteristics();
}

void NvidiaNVLinkPort::sendQueryPortStatus()
{
    if (!statusRequestEncoded)
    {
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, statusRequest,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaNVLinkPort> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaNVLinkPort");
                return;
            }
            self->processPortStatusResponse(ec, buffer);
        });
}

void NvidiaNVLinkPort::processPortStatusResponse(
    const std::error_code& sendRecvMsgResult, std::span<const uint8_t> response)
{
    if (sendRecvMsgResult)
    {
        lg2::error(
            "Error updating NVLink Port status: sending message over MCTP failed, rc={RC}, EID={EID}, portNumber={PN}",
            "RC", sendRecvMsgResult.message(), "EID", eid, "PN", portNumber);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint8_t portState = 0;
    uint8_t portStatus = 0;

    const int rc = gpu::decodeQueryPortStatusResponse(response, cc, reasonCode,
                                                      portState, portStatus);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating NVLink Port status: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}, portNumber={PN}",
            "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID",
            eid, "PN", portNumber);
        return;
    }

    portInterface->set_property("LinkStatus",
                                mapPortStateToLinkStatus(portState));
    portInterface->set_property("LinkState",
                                mapPortStatusToLinkState(portStatus));
}

void NvidiaNVLinkPort::sendQueryPortCharacteristics()
{
    if (!characteristicsRequestEncoded)
    {
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, characteristicsRequest,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaNVLinkPort> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaNVLinkPort");
                return;
            }
            self->processPortCharacteristicsResponse(ec, buffer);
        });
}

void NvidiaNVLinkPort::processPortCharacteristicsResponse(
    const std::error_code& sendRecvMsgResult, std::span<const uint8_t> response)
{
    if (sendRecvMsgResult)
    {
        lg2::error(
            "Error updating NVLink Port characteristics: sending message over MCTP failed, rc={RC}, EID={EID}, portNumber={PN}",
            "RC", sendRecvMsgResult.message(), "EID", eid, "PN", portNumber);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint32_t status = 0;
    uint32_t nvportLineRateMbps = 0;
    uint32_t nvportDataRateKbps = 0;
    uint32_t statusLaneInfo = 0;

    const int rc = gpu::decodeQueryPortCharacteristicsResponse(
        response, cc, reasonCode, status, nvportLineRateMbps,
        nvportDataRateKbps, statusLaneInfo);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating NVLink Port characteristics: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}, portNumber={PN}",
            "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID",
            eid, "PN", portNumber);
        return;
    }

    // The line rate is reported in Mbps and the data rate in Kbps, while both
    // Speed properties are in bits per second.
    const uint64_t maxSpeedBps =
        static_cast<uint64_t>(nvportLineRateMbps) * 1000000ULL;
    const uint64_t speedBps =
        static_cast<uint64_t>(nvportDataRateKbps) * 1000ULL;
    const size_t width = static_cast<size_t>(statusLaneInfo & 0x0F);

    portInterface->set_property("MaxSpeed", maxSpeedBps);
    portInterface->set_property("Speed", speedBps);
    portInterface->set_property("Width", width);
}
