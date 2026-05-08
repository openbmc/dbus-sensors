/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuClockSpeedControl.hpp"

#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Common/Device/error.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

static constexpr uint64_t mhzToHzFactor = 1'000'000;

NvidiaGpuClockSpeedControl::NvidiaGpuClockSpeedControl(
    sdbusplus::asio::object_server& objectServer, const std::string& deviceName,
    const std::string& inventoryPath, mctp::MctpRequester& mctpRequester,
    uint8_t eid,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        controlClockSpeedIface) :
    controlClockSpeedInterface(controlClockSpeedIface),
    mctpRequester(mctpRequester), name(escapeName(deviceName)),
    objectServer(objectServer), eid(eid)
{
    const sdbusplus::object_path objPath(
        controlClockSpeedIface->get_object_path());

    associationInterface =
        objectServer.add_interface(objPath.str, association::interface);
    std::vector<Association> associations;
    associations.emplace_back("controlling", "controlled_by", inventoryPath);
    associationInterface->register_property("Associations", associations);
    associationInterface->initialize();
    controlClockSpeedInterface->register_deferred_method<>(
        "Reset", [this](sdbusplus::asio::deferred_reply<> reply) {
            reset(std::move(reply));
        });
    controlClockSpeedInterface->initialize();
}

NvidiaGpuClockSpeedControl::~NvidiaGpuClockSpeedControl()
{
    objectServer.remove_interface(associationInterface);
}

void NvidiaGpuClockSpeedControl::update()
{
    int rc = gpu::encodeGetClockLimitRequest(0, gpu::ClockType::GRAPHICS_CLOCK,
                                             requestBuffer);
    if (rc != 0)
    {
        lg2::error("Failed to encode clock limit request for {NAME}: rc={RC}",
                   "NAME", name, "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, requestBuffer,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            auto self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid NvidiaGpuClockSpeedControl reference");
                return;
            }
            self->handleResponse(ec, buffer);
        });
}

void NvidiaGpuClockSpeedControl::handleResponse(const std::error_code& ec,
                                                std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("Error reading clock limit for {NAME}: MCTP failed, rc={RC}",
                   "NAME", name, "RC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint32_t requestedLimitMin = 0;
    uint32_t requestedLimitMax = 0;
    uint32_t presentLimitMin = 0;
    uint32_t presentLimitMax = 0;

    int rc = gpu::decodeGetClockLimitResponse(
        buffer, cc, reasonCode, requestedLimitMin, requestedLimitMax,
        presentLimitMin, presentLimitMax);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding clock limit for {NAME}: rc={RC}, cc={CC}, reasonCode={REASON}",
            "NAME", name, "RC", rc, "CC", static_cast<uint8_t>(cc), "REASON",
            reasonCode);
        return;
    }

    uint64_t presentMaxHz =
        static_cast<uint64_t>(presentLimitMax) * mhzToHzFactor;
    uint64_t presentMinHz =
        static_cast<uint64_t>(presentLimitMin) * mhzToHzFactor;
    controlClockSpeedInterface->set_property("PresentSpeedLimitMaxHz",
                                             presentMaxHz);
    controlClockSpeedInterface->set_property("PresentSpeedLimitMinHz",
                                             presentMinHz);

    uint64_t reqMaxHz =
        static_cast<uint64_t>(requestedLimitMax) * mhzToHzFactor;
    uint64_t reqMinHz =
        static_cast<uint64_t>(requestedLimitMin) * mhzToHzFactor;
    controlClockSpeedInterface->set_property("RequestedSpeedLimitMaxHz",
                                             reqMaxHz);
    controlClockSpeedInterface->set_property("RequestedSpeedLimitMinHz",
                                             reqMinHz);
}

void NvidiaGpuClockSpeedControl::reset(sdbusplus::asio::deferred_reply<> reply)
{
    std::array<uint8_t, gpu::setClockLimitRequestSize> reqBuf{};
    int rc = gpu::encodeSetClockLimitRequest(
        0, static_cast<uint8_t>(gpu::ClockType::GRAPHICS_CLOCK),
        gpu::clockLimitFlagClear, 0, 0, reqBuf);
    if (rc != 0)
    {
        lg2::error(
            "Failed to encode SET_CLOCK_LIMIT request for {NAME}: rc={RC}",
            "NAME", name, "RC", rc);
        reply.send_error(sdbusplus::error::xyz::openbmc_project::common::
                             device::WriteFailure());
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, reqBuf,
        [weak{weak_from_this()},
         reply = std::move(reply)](const std::error_code& ec,
                                   std::span<const uint8_t> buffer) mutable {
            auto self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid NvidiaGpuClockSpeedControl reference");
                reply.send_error(sdbusplus::error::xyz::openbmc_project::
                                     common::device::WriteFailure());
                return;
            }
            if (ec)
            {
                lg2::error("Error sending SET_CLOCK_LIMIT for {NAME}: rc={RC}",
                           "NAME", self->name, "RC", ec.message());
                reply.send_error(sdbusplus::error::xyz::openbmc_project::
                                     common::device::WriteFailure());
                return;
            }

            ocp::accelerator_management::CompletionCode cc{};
            uint16_t reasonCode = 0;
            int rc = gpu::decodeSetClockLimitResponse(buffer, cc, reasonCode);
            if (rc != 0 ||
                cc != ocp::accelerator_management::CompletionCode::SUCCESS)
            {
                lg2::error(
                    "Error decoding SET_CLOCK_LIMIT for {NAME}: rc={RC}, "
                    "cc={CC}, reasonCode={REASON}",
                    "NAME", self->name, "RC", rc, "CC",
                    static_cast<uint8_t>(cc), "REASON", reasonCode);
                reply.send_error(sdbusplus::error::xyz::openbmc_project::
                                     common::device::WriteFailure());
                return;
            }

            reply.send();
        });
}
