/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaNVLinkPortCharacteristics.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <system_error>

NvidiaNVLinkPortCharacteristics::NvidiaNVLinkPortCharacteristics(
    mctp::MctpRequester& mctpRequester, uint8_t eid, uint8_t portIndex,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& portInterface) :
    eid(eid), portNumber(static_cast<uint8_t>(portIndex + 1)),
    mctpRequester(mctpRequester), portInterface(portInterface)
{
    if (gpu::encodeQueryPortCharacteristicsRequest(0, portNumber, request) != 0)
    {
        lg2::error(
            "Failed to encode NVLink Port characteristics request, eid={EID}, portNumber={PN}",
            "EID", eid, "PN", portNumber);
    }
    else
    {
        requestEncoded = true;
    }
}

void NvidiaNVLinkPortCharacteristics::update()
{
    if (!requestEncoded)
    {
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaNVLinkPortCharacteristics> self = weak.lock();
            if (!self)
            {
                lg2::error(
                    "Invalid reference to NvidiaNVLinkPortCharacteristics");
                return;
            }
            self->processResponse(ec, buffer);
        });
}

void NvidiaNVLinkPortCharacteristics::processResponse(
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
