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

constexpr const char* portInterfaceName =
    "xyz.openbmc_project.Inventory.Connector.Port";

std::string mapPortStateToLinkStatus(uint8_t portState)
{
    constexpr const char* prefix =
        "xyz.openbmc_project.Inventory.Connector.Port.LinkStatus.";

    switch (portState)
    {
        case 2:
            return std::string(prefix) + "LinkUp";
        case 6:
            return std::string(prefix) + "Starting";
        case 7:
        case 10:
            return std::string(prefix) + "Training";
        case 1:
        case 4:
        case 5:
        case 8:
        case 9:
            return std::string(prefix) + "LinkDown";
        case 3:
        default:
            return std::string(prefix) + "NoLink";
    }
}

std::string mapPortStatusToLinkState(uint8_t portStatus)
{
    constexpr const char* prefix =
        "xyz.openbmc_project.Inventory.Connector.Port.LinkState.";

    switch (portStatus)
    {
        case 1:
            return std::string(prefix) + "Disabled";
        case 2:
            return std::string(prefix) + "Enabled";
        default:
            return std::string(prefix) + "Unknown";
    }
}

} // namespace

NvidiaNVLinkPort::NvidiaNVLinkPort(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& gpuName, uint8_t eid,
    uint8_t portIndex, sdbusplus::asio::object_server& objectServer) :
    eid(eid), portNumber(static_cast<uint8_t>(portIndex + 1)), gpuName(gpuName),
    conn(conn), mctpRequester(mctpRequester)
{
    statusRequest.resize(gpu::queryPortStatusRequestSize);
    characteristicsRequest.resize(gpu::queryPortCharacteristicsRequestSize);

    const std::string portName = std::format("NVLink_{}", portIndex);

    const sdbusplus::object_path gpuPath = inventoryPrefix / gpuName;
    const sdbusplus::object_path dbusPath = gpuPath / portName;

    portInterface = objectServer.add_interface(dbusPath, portInterfaceName);

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

    portInterface->register_property("LinkStatus", mapPortStateToLinkStatus(3));

    portInterface->register_property("LinkState", mapPortStatusToLinkState(0));

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

void NvidiaNVLinkPort::update()
{
    sendQueryPortStatus();
}

void NvidiaNVLinkPort::sendQueryPortStatus()
{
    const int rc =
        gpu::encodeQueryPortStatusRequest(0, portNumber, statusRequest);
    if (rc != 0)
    {
        lg2::error(
            "Failed to encode NVLink Port status request: rc={RC}, EID={EID}, portNumber={PN}",
            "RC", rc, "EID", eid, "PN", portNumber);
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

    sendQueryPortCharacteristics();
}

void NvidiaNVLinkPort::sendQueryPortCharacteristics()
{
    const int rc = gpu::encodeQueryPortCharacteristicsRequest(
        0, portNumber, characteristicsRequest);
    if (rc != 0)
    {
        lg2::error(
            "Failed to encode NVLink Port characteristics request: rc={RC}, EID={EID}, portNumber={PN}",
            "RC", rc, "EID", eid, "PN", portNumber);
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

    const uint64_t maxSpeedBps =
        static_cast<uint64_t>(nvportLineRateMbps) * 1000000ULL;
    const uint64_t speedBps =
        static_cast<uint64_t>(nvportDataRateKbps) * 1000ULL;
    const size_t width = static_cast<size_t>(statusLaneInfo & 0x0F);

    portInterface->set_property("MaxSpeed", maxSpeedBps);
    portInterface->set_property("Speed", speedBps);
    portInterface->set_property("Width", width);
}
