/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuClockSpeedControl.hpp"

#include "NvidiaUtils.hpp"
#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/error.hpp>
#include <boost/system/error_code.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/completion.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Common/Device/error.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

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
    mctp::MctpRequester& mctpRequester, uint8_t eid,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        controlClockSpeedIface) :
    controlClockSpeedInterface(controlClockSpeedIface),
    mctpRequester(mctpRequester), name(escapeName(deviceName)),
    objectServer(objectServer), eid(eid)
{
    const int rc = gpu::encodeGetClockLimitRequest(
        0, gpu::ClockType::GRAPHICS_CLOCK, requestBuffer);
    if (rc == 0)
    {
        requestEncoded = true;
    }
    else
    {
        lg2::error("Failed to encode clock limit request for {NAME}: rc={RC}",
                   "NAME", name, "RC", rc);
    }

    const sdbusplus::object_path inventoryPath = inventoryPrefix / name;
    const sdbusplus::object_path objPath(
        controlClockSpeedIface->get_object_path());

    associationInterface =
        objectServer.add_interface(objPath.str, association::interface);
    std::vector<Association> associations;
    associations.emplace_back("controlling", "controlled_by", inventoryPath);
    associationInterface->register_property("Associations", associations);
    if (!associationInterface->initialize())
    {
        lg2::error(
            "Error initializing Association interface for Clock Speed Control for {NAME}, eid={EID}",
            "NAME", name, "EID", eid);
    }
}

NvidiaGpuClockSpeedControl::~NvidiaGpuClockSpeedControl()
{
    objectServer.remove_interface(associationInterface);
}

void NvidiaGpuClockSpeedControl::update()
{
    if (!requestEncoded)
    {
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

void NvidiaGpuClockSpeedControl::reset(sdbusplus::asio::completion<> done)
{
    if (resetInFlight)
    {
        throw sdbusplus::error::xyz::openbmc_project::common::Unavailable();
    }

    int rc = gpu::encodeSetClockLimitRequest(
        0, gpu::ClockType::GRAPHICS_CLOCK, gpu::ClockLimitFlag::CLEAR, 0, 0,
        resetRequestBuffer);
    if (rc != 0)
    {
        lg2::error("Failed to encode clock limit reset for {NAME}: rc={RC}",
                   "NAME", name, "RC", rc);
        throw sdbusplus::error::xyz::openbmc_project::common::device::
            WriteFailure();
    }

    resetInFlight = true;
    mctpRequester.sendRecvMsg(
        eid, resetRequestBuffer,
        [weak{weak_from_this()},
         done = std::move(done)](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) mutable {
            auto self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid NvidiaGpuClockSpeedControl reference");
                done(boost::asio::error::operation_aborted);
                return;
            }
            self->completeReset(ec, buffer, std::move(done));
        });
}

void NvidiaGpuClockSpeedControl::completeReset(
    const std::error_code& ec, std::span<const uint8_t> buffer,
    sdbusplus::asio::completion<> done)
{
    resetInFlight = false;

    if (ec)
    {
        lg2::error(
            "Error resetting clock limit for {NAME}: MCTP failed, rc={RC}",
            "NAME", name, "RC", ec.message());
        done(boost::system::errc::make_error_code(
            boost::system::errc::io_error));
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;

    int rc = gpu::decodeSetClockLimitResponse(buffer, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding clock limit reset for {NAME}: rc={RC}, cc={CC}, reasonCode={REASON}",
            "NAME", name, "RC", rc, "CC", static_cast<uint8_t>(cc), "REASON",
            reasonCode);
        done(boost::system::errc::make_error_code(
            boost::system::errc::protocol_error));
        return;
    }

    done(boost::system::error_code{}); // success
}
