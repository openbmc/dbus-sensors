/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuEccMode.hpp"

#include <MctpRequester.hpp>
#include <MessagePackUnpackUtils.hpp>
#include <NvidiaGpuLongRunningCommand.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaLongRunningHandler.hpp>
#include <OcpMctpVdm.hpp>
#include <SerialQueue.hpp>
#include <boost/system/error_code.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>

namespace
{
constexpr const char* inventoryPrefix = "/xyz/openbmc_project/inventory/";
constexpr const char* eccModeInterfaceName =
    "xyz.openbmc_project.Control.Processor.EccMode";

// Decode the ECC mode flag byte that both the synchronous SUCCESS payload
// and the long-running event payload carry. Bit 0 is the active mode, bit
// 1 is the mode the GPU will apply on next reset.
int unpackEccFlags(UnpackBuffer& buf, bool& active, bool& enabled)
{
    uint8_t flags = 0;
    const int rc = buf.unpack(flags);
    if (rc != 0)
    {
        return rc;
    }
    active = (flags & 0x01U) != 0;
    enabled = (flags & 0x02U) != 0;
    return 0;
}
} // namespace

NvidiaGpuEccMode::NvidiaGpuEccMode(
    mctp::MctpRequester& mctpRequester,
    sdbusplus::asio::object_server& objectServer, const std::string& deviceName,
    uint8_t eid, std::shared_ptr<SerialQueue> longRunningQueue,
    std::shared_ptr<NvidiaLongRunningResponseHandler>
        longRunningResponseHandler) :
    enabledValue(std::make_shared<bool>(false)), eid(eid),
    mctpRequester(mctpRequester), longRunningQueue(std::move(longRunningQueue)),
    longRunningResponseHandler(std::move(longRunningResponseHandler))
{
    const std::string inventoryPath = std::string(inventoryPrefix) + deviceName;

    eccModeInterface =
        objectServer.add_interface(inventoryPath, eccModeInterfaceName);

    // Active is read-only. sdbusplus owns the storage; refresh from
    // hardware uses set_property("Active", ...) which updates and
    // signals without invoking any setter callback.
    eccModeInterface->register_property(
        "Active", false, sdbusplus::asio::PropertyPermission::readOnly);

    // Enabled is writable. External writes dispatch an NSM Set ECC Mode
    // via onEnabledSetRequested; hardware-driven refreshes from the GET
    // pipeline bypass the setter by updating *enabledValue directly and
    // calling signal_property, so they cannot retrigger the setter.
    eccModeInterface->register_property<bool>(
        "Enabled", false,
        [this](const bool& req, bool& existing) {
            existing = req;
            *enabledValue = req;
            onEnabledSetRequested(req);
            return true;
        },
        [enabledPtr{enabledValue}](bool&) { return *enabledPtr; });

    if (!eccModeInterface->initialize())
    {
        lg2::error("Failed to initialize ECC mode interface for GPU {NAME}",
                   "NAME", deviceName);
    }

    getCmd = std::make_shared<NvidiaGpuLongRunningCommand>(
        eid, this->mctpRequester, this->longRunningQueue,
        this->longRunningResponseHandler,
        NvidiaGpuLongRunningCommand::Config{
            .metricName = "GPU ECC Mode",
            .messageType = gpu::MessageType::PLATFORM_ENVIRONMENTAL,
            .commandId = gpu::PlatformEnvironmentalCommands::GET_ECC_MODE,
            .requestSize = gpu::getEccModeRequestSize,
            .encodeRequest =
                [](std::span<uint8_t> buf) {
                    return gpu::encodeGetEccModeRequest(0, buf);
                },
            .onImmediateSuccess =
                std::bind_front(&NvidiaGpuEccMode::onGetImmediateSuccess,
                                eccModeInterface, enabledValue, eid),
            .onLongRunningPayload =
                std::bind_front(&NvidiaGpuEccMode::onGetLongRunningPayload,
                                eccModeInterface, enabledValue, eid),
        });
}

void NvidiaGpuEccMode::update()
{
    getCmd->update();
}

void NvidiaGpuEccMode::onGetImmediateSuccess(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& eccModeInterface,
    const std::shared_ptr<bool>& enabledValue, uint8_t eid,
    std::span<const uint8_t> fullBuffer)
{
    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    bool active = false;
    bool enabled = false;
    const int rc = gpu::decodeGetEccModeResponse(fullBuffer, cc, reasonCode,
                                                 active, enabled);
    if (rc != 0)
    {
        lg2::error("Error updating GPU ECC Mode: decode failed, "
                   "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}",
                   "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode,
                   "EID", eid);
        return;
    }

    applyEccModeToDbus(eccModeInterface, enabledValue, active, enabled);
}

void NvidiaGpuEccMode::onGetLongRunningPayload(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& eccModeInterface,
    const std::shared_ptr<bool>& enabledValue, uint8_t eid,
    std::span<const uint8_t> payload)
{
    UnpackBuffer buf{payload};
    bool active = false;
    bool enabled = false;
    const int rc = unpackEccFlags(buf, active, enabled);
    if (rc != 0)
    {
        lg2::error("Error updating GPU ECC Mode: "
                   "failed to unpack long running flags, rc={RC}, EID={EID}",
                   "RC", rc, "EID", eid);
        return;
    }

    applyEccModeToDbus(eccModeInterface, enabledValue, active, enabled);
}

void NvidiaGpuEccMode::applyEccModeToDbus(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& eccModeInterface,
    const std::shared_ptr<bool>& enabledValue, bool active, bool enabled)
{
    // Active is owned by sdbusplus; set_property updates and signals.
    eccModeInterface->set_property("Active", active);

    // Enabled is member-backed; update the value and emit the signal
    // manually. signal_property does not invoke the registered setter,
    // so this hardware-driven refresh never retriggers an NSM Set.
    *enabledValue = enabled;
    eccModeInterface->signal_property("Enabled");
}

void NvidiaGpuEccMode::onEnabledSetRequested(bool desired)
{
    longRunningQueue->submit(
        [weak{weak_from_this()}, desired](SerialQueue::ReleaseHandle handle) {
            std::shared_ptr<NvidiaGpuEccMode> self = weak.lock();
            if (!self)
            {
                return;
            }
            self->doSet(std::move(handle), desired);
        });
}

void NvidiaGpuEccMode::doSet(SerialQueue::ReleaseHandle handle, bool desired)
{
    const int rc = gpu::encodeSetEccModeRequest(0, desired, setRequest);

    if (rc != 0)
    {
        lg2::error(
            "Error setting GPU ECC Mode: encode failed, rc={RC}, EID={EID}",
            "RC", rc, "EID", eid);
        // The writable setter has already applied the requested value
        // optimistically to the Enabled property. The SET never reached
        // the device, so refresh from hardware to overwrite the stale
        // optimistic value rather than wait ~30s for the next LR poll.
        getCmd->update();
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, setRequest,
        [weak{weak_from_this()},
         handle = std::move(handle)](const std::error_code& ec,
                                     std::span<const uint8_t> buffer) mutable {
            std::shared_ptr<NvidiaGpuEccMode> self = weak.lock();
            if (!self)
            {
                return;
            }
            self->processSetResponse(std::move(handle), ec, buffer);
        });
}

void NvidiaGpuEccMode::processSetResponse(SerialQueue::ReleaseHandle handle,
                                          const std::error_code& ec,
                                          std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("Error setting GPU ECC Mode: sending SET over MCTP failed, "
                   "rc={RC}, EID={EID}",
                   "RC", ec.message(), "EID", eid);
        getCmd->update();
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;

    int rc = gpu::decodeSetEccModeResponse(buffer, cc, reasonCode);

    if (rc != 0)
    {
        lg2::error("Error setting GPU ECC Mode: SET decode failed, "
                   "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}",
                   "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode,
                   "EID", eid);
        getCmd->update();
        return;
    }

    switch (cc)
    {
        case ocp::accelerator_management::CompletionCode::SUCCESS:
            return;

        case ocp::accelerator_management::CompletionCode::ACCEPTED:
        {
            uint8_t responseInstanceId = 0;
            rc = ocp::accelerator_management::decodeInstanceId(
                buffer, responseInstanceId);
            if (rc != 0)
            {
                lg2::error("Error setting GPU ECC Mode: failed to decode "
                           "instance id, rc={RC}, EID={EID}",
                           "RC", rc, "EID", eid);
                getCmd->update();
                return;
            }

            rc = longRunningResponseHandler->registerResponseHandler(
                gpu::MessageType::PLATFORM_ENVIRONMENTAL,
                static_cast<uint8_t>(
                    gpu::PlatformEnvironmentalCommands::SET_ECC_MODE),
                responseInstanceId,
                [weak{weak_from_this()}, handle = std::move(handle)](
                    boost::system::error_code lrEc,
                    ocp::accelerator_management::CompletionCode lrCc,
                    uint16_t lrReason, std::span<const uint8_t> data) mutable {
                    std::shared_ptr<NvidiaGpuEccMode> self = weak.lock();
                    if (!self)
                    {
                        return;
                    }
                    self->processSetLongRunningResponse(lrEc, lrCc, lrReason,
                                                        data);
                });

            if (rc != 0)
            {
                lg2::error("Error setting GPU ECC Mode: failed to register "
                           "long running handler, rc={RC}, EID={EID}, "
                           "IID={IID}",
                           "RC", rc, "EID", eid, "IID", responseInstanceId);
                // The device may still apply the requested mode, but we
                // will not catch its long-running event. Refresh now to
                // converge the property to hardware reality.
                getCmd->update();
            }
            return;
        }

        default:
            lg2::error("Error setting GPU ECC Mode: SET unexpected completion "
                       "code, cc={CC}, reasonCode={RESC}, EID={EID}",
                       "CC", static_cast<uint8_t>(cc), "RESC", reasonCode,
                       "EID", eid);
            getCmd->update();
            return;
    }
}

void NvidiaGpuEccMode::processSetLongRunningResponse(
    boost::system::error_code ec,
    ocp::accelerator_management::CompletionCode cc, uint16_t reasonCode,
    std::span<const uint8_t> /*responseData*/)
{
    if (ec)
    {
        lg2::error("GPU ECC Mode SET long running event failed, "
                   "ec={EC}, EID={EID}",
                   "EC", ec.message(), "EID", eid);
        getCmd->update();
        return;
    }

    if (cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error("GPU ECC Mode SET long running indicated failure, "
                   "cc={CC}, reasonCode={RESC}, EID={EID}",
                   "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID",
                   eid);
        getCmd->update();
    }
}
