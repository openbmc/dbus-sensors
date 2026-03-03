/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuCurrentUtilization.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaLongRunningHandler.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <system_error>

NvidiaGpuCurrentUtilization::NvidiaGpuCurrentUtilization(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester,
    [[maybe_unused]] const std::string& deviceName, uint8_t eid,
    const std::shared_ptr<NvidiaLongRunningResponseHandler>&
        longRunningResponseHandler,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        operatingConfigInterface) :
    eid(eid), conn(conn), mctpRequester(mctpRequester),
    longRunningResponseHandler(longRunningResponseHandler),
    operatingConfigInterface(operatingConfigInterface)
{}

void NvidiaGpuCurrentUtilization::update()
{
    const int rc = gpu::encodeGetCurrentUtilizationModeRequest(0, request);

    if (rc != 0)
    {
        lg2::error(
            "Error updating GPU Current Utilization: encode failed, rc={RC}, EID={EID}",
            "RC", rc, "EID", eid);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaGpuCurrentUtilization> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaGpuCurrentUtilization");
                return;
            }
            self->processResponse(ec, buffer);
        });
}

void NvidiaGpuCurrentUtilization::processResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error(
            "Error updating GPU Current Utilization: sending message over MCTP failed, "
            "rc={RC}, EID={EID}",
            "RC", ec.message(), "EID", eid);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;

    int rc = ocp::accelerator_management::decodeReasonCodeAndCC(
        buffer, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::ACCEPTED)
    {
        lg2::error("Error updating GPU Current Utilization: decode failed, "
                   "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}",
                   "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode,
                   "EID", eid);
        return;
    }

    rc = longRunningResponseHandler->registerResponseHandler(
        gpu::MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(
            gpu::PlatformEnvironmentalCommands::GET_CURRENT_UTILIZATION),
        [weak{weak_from_this()}](ocp::accelerator_management::CompletionCode cc,
                                 uint16_t reasonCode,
                                 std::span<const uint8_t> responseData) {
            std::shared_ptr<NvidiaGpuCurrentUtilization> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaGpuCurrentUtilization");
                return;
            }

            self->processLongRunningResponse(cc, reasonCode, responseData);
        });

    if (rc != 0)
    {
        lg2::error(
            "Error updating GPU Current Utilization: failed to register long running response handler, "
            "rc={RC}, EID={EID}",
            "RC", rc, "EID", eid);
        return;
    }
}

void NvidiaGpuCurrentUtilization::processLongRunningResponse(
    ocp::accelerator_management::CompletionCode cc, uint16_t reasonCode,
    std::span<const uint8_t> responseData)
{
    if (cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating GPU Current Utilization: long running response indicated failure, "
            "cc={CC}, reasonCode={RESC}, EID={EID}",
            "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID", eid);
        return;
    }

    if (responseData.size() < 4)
    {
        lg2::error(
            "Error updating GPU Current Utilization: invalid long running response data size, "
            "size={SIZE}, EID={EID}",
            "SIZE", responseData.size(), "EID", eid);
        return;
    }

    uint32_t utilization = 0;

    std::memcpy(&utilization, responseData.data(), sizeof(uint32_t));

    operatingConfigInterface->set_property("Utilization",
                                           static_cast<double>(utilization));
}
