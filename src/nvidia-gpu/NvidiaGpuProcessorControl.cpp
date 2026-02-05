/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuProcessorControl.hpp"

#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
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
}
