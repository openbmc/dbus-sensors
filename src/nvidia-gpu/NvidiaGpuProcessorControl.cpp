/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuProcessorControl.hpp"

#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/async_result.hpp>
#include <boost/asio/spawn.hpp>
#include <boost/system/error_code.hpp>
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

NvidiaGpuProcessorControl::NvidiaGpuProcessorControl(
    sdbusplus::asio::object_server& objectServer, const std::string& deviceName,
    const std::string& inventoryPath, mctp::MctpRequester& mctpRequester,
    uint8_t eid,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        controlProcessorIface) :
    controlProcessorInterface(controlProcessorIface),
    mctpRequester(mctpRequester), name(escapeName(deviceName)),
    objectServer(objectServer), eid(eid)
{
    const sdbusplus::message::object_path objPath(
        controlProcessorIface->get_object_path());

    associationInterface =
        objectServer.add_interface(objPath.str, association::interface);
    std::vector<Association> associations;
    associations.emplace_back("controlling", "controlled_by", inventoryPath);
    associationInterface->register_property("Associations", associations);
    associationInterface->initialize();

    controlProcessorInterface->register_method(
        "ResetToDefaults", [this](const boost::asio::yield_context& yield) {
            resetToDefaults(yield);
        });
    controlProcessorInterface->initialize();
}

NvidiaGpuProcessorControl::~NvidiaGpuProcessorControl()
{
    objectServer.remove_interface(associationInterface);
}

void NvidiaGpuProcessorControl::update()
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
                lg2::error("Invalid NvidiaGpuProcessorControl reference");
                return;
            }
            self->handleResponse(ec, buffer);
        });
}

void NvidiaGpuProcessorControl::handleResponse(const std::error_code& ec,
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

    uint64_t limitHz = static_cast<uint64_t>(presentLimitMax) * mhzToHzFactor;
    controlProcessorInterface->set_property("SpeedLimitHz", limitHz);
    controlProcessorInterface->set_property("SpeedLimitLocked",
                                            presentLimitMax == presentLimitMin);

    uint64_t reqMaxHz =
        static_cast<uint64_t>(requestedLimitMax) * mhzToHzFactor;
    uint64_t reqMinHz =
        static_cast<uint64_t>(requestedLimitMin) * mhzToHzFactor;
    controlProcessorInterface->set_property("RequestedSpeedLimitMaxHz",
                                            reqMaxHz);
    controlProcessorInterface->set_property("RequestedSpeedLimitMinHz",
                                            reqMinHz);
}

void NvidiaGpuProcessorControl::resetToDefaults(
    const boost::asio::yield_context& yield)
{
    // prevent destruction while coroutine is suspended
    auto self = shared_from_this();

    std::array<uint8_t, gpu::setClockLimitRequestSize> reqBuf{};
    int rc = gpu::encodeSetClockLimitRequest(
        0, static_cast<uint8_t>(gpu::ClockType::GRAPHICS_CLOCK),
        gpu::clockLimitFlagClear, 0, 0, reqBuf);
    if (rc != 0)
    {
        lg2::error(
            "Failed to encode SET_CLOCK_LIMIT request for {NAME}: rc={RC}",
            "NAME", name, "RC", rc);
        throw sdbusplus::error::xyz::openbmc_project::common::device::
            WriteFailure();
    }

    // Bridge sendRecvMsg callback into yield_context coroutine.
    // Coroutine suspends here; event loop continues other tasks.
    // Resumes when MCTP response arrives.
    boost::system::error_code ec;
    std::vector<uint8_t> responseData = boost::asio::async_initiate<void(
        boost::system::error_code, std::vector<uint8_t>)>(
        [self, &reqBuf](auto handler) {
            self->mctpRequester.sendRecvMsg(
                self->eid, reqBuf,
                [h = std::move(handler)](const std::error_code& ec,
                                         std::span<const uint8_t> buf) mutable {
                    std::vector<uint8_t> data(buf.begin(), buf.end());
                    std::move(h)(boost::system::error_code(ec),
                                 std::move(data));
                });
        },
        yield[ec]);

    if (ec)
    {
        lg2::error("Error sending SET_CLOCK_LIMIT for {NAME}: rc={RC}", "NAME",
                   name, "RC", ec.message());
        throw sdbusplus::error::xyz::openbmc_project::common::device::
            WriteFailure();
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    rc = gpu::decodeSetClockLimitResponse(responseData, cc, reasonCode);
    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding SET_CLOCK_LIMIT for {NAME}: rc={RC}, cc={CC}, "
            "reasonCode={REASON}",
            "NAME", name, "RC", rc, "CC", static_cast<uint8_t>(cc), "REASON",
            reasonCode);
        throw sdbusplus::error::xyz::openbmc_project::common::device::
            WriteFailure();
    }
    // coroutine returns -> sdbusplus sends D-Bus success reply to bmcweb
}
