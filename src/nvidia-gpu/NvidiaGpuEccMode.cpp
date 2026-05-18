/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuEccMode.hpp"

#include <MctpRequester.hpp>
#include <MessagePackUnpackUtils.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaLongRunningHandler.hpp>
#include <OcpMctpVdm.hpp>
#include <SerialQueue.hpp>
#include <boost/system/error_code.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>

namespace
{
constexpr const char* controlProcessorBase =
    "/xyz/openbmc_project/control/processor";
constexpr const char* objectEnableInterface =
    "xyz.openbmc_project.Object.Enable";
} // namespace

NvidiaGpuEccMode::NvidiaGpuEccMode(
    std::shared_ptr<sdbusplus::asio::connection>& /*conn*/,
    mctp::MctpRequester& mctpRequester,
    sdbusplus::asio::object_server& objectServer, const std::string& deviceName,
    uint8_t eid, std::shared_ptr<SerialQueue> longRunningQueue,
    std::shared_ptr<NvidiaLongRunningResponseHandler>
        longRunningResponseHandler) :
    eid(eid), mctpRequester(mctpRequester),
    longRunningQueue(std::move(longRunningQueue)),
    longRunningResponseHandler(std::move(longRunningResponseHandler))
{
    const sdbusplus::object_path eccModeBase =
        sdbusplus::object_path(controlProcessorBase) / deviceName / "ecc_mode";

    currentInterface = objectServer.add_interface(eccModeBase / "current",
                                                  objectEnableInterface);
    currentInterface->register_property(
        "Enabled", false, sdbusplus::asio::PropertyPermission::readOnly);

    if (!currentInterface->initialize())
    {
        lg2::error(
            "Failed to initialize ECC mode current interface for GPU {NAME}",
            "NAME", deviceName);
    }

    pendingInterface = objectServer.add_interface(eccModeBase / "pending",
                                                  objectEnableInterface);

    pendingInterface->register_property(
        "Enabled", false, [this](const bool& req, bool& existing) -> bool {
            existing = req;
            if (!internalUpdateInProgress)
            {
                onPendingSetRequested(req);
            }
            return true;
        });

    if (!pendingInterface->initialize())
    {
        lg2::error(
            "Failed to initialize ECC mode pending interface for GPU {NAME}",
            "NAME", deviceName);
    }
}

void NvidiaGpuEccMode::update()
{
    longRunningQueue->submit(
        [weak{weak_from_this()}](SerialQueue::ReleaseHandle handle) {
            std::shared_ptr<NvidiaGpuEccMode> self = weak.lock();
            if (!self)
            {
                return;
            }
            self->doUpdate(std::move(handle));
        });
}

void NvidiaGpuEccMode::doUpdate(SerialQueue::ReleaseHandle handle)
{
    const int rc = gpu::encodeGetEccModeRequest(0, getRequest);

    if (rc != 0)
    {
        lg2::error(
            "Error updating GPU ECC Mode: encode failed, rc={RC}, EID={EID}",
            "RC", rc, "EID", eid);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, getRequest,
        [weak{weak_from_this()},
         handle = std::move(handle)](const std::error_code& ec,
                                     std::span<const uint8_t> buffer) mutable {
            std::shared_ptr<NvidiaGpuEccMode> self = weak.lock();
            if (!self)
            {
                return;
            }
            self->processGetResponse(std::move(handle), ec, buffer);
        });
}

void NvidiaGpuEccMode::processGetResponse(SerialQueue::ReleaseHandle handle,
                                          const std::error_code& ec,
                                          std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("Error updating GPU ECC Mode: sending GET over MCTP failed, "
                   "rc={RC}, EID={EID}",
                   "RC", ec.message(), "EID", eid);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;

    UnpackBuffer unpackBuffer(buffer);

    int rc = gpu::decodeResponseCommonHeader(
        unpackBuffer, gpu::MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(gpu::PlatformEnvironmentalCommands::GET_ECC_MODE),
        cc, reasonCode);

    if (rc != 0)
    {
        lg2::error("Error updating GPU ECC Mode: GET decode failed, "
                   "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}",
                   "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode,
                   "EID", eid);
        return;
    }

    switch (cc)
    {
        case ocp::accelerator_management::CompletionCode::SUCCESS:
        {
            bool current = false;
            bool pending = false;
            rc = gpu::decodeGetEccModeResponse(buffer, cc, reasonCode, current,
                                               pending);
            if (rc != 0)
            {
                lg2::error("Error updating GPU ECC Mode: GET decode failed, "
                           "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}",
                           "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC",
                           reasonCode, "EID", eid);
                return;
            }
            applyEccModeToDbus(current, pending);
            return;
        }

        case ocp::accelerator_management::CompletionCode::ACCEPTED:
        {
            uint8_t responseInstanceId = 0;
            rc = ocp::accelerator_management::decodeInstanceId(
                buffer, responseInstanceId);
            if (rc != 0)
            {
                lg2::error("Error updating GPU ECC Mode: failed to decode "
                           "instance id, rc={RC}, EID={EID}",
                           "RC", rc, "EID", eid);
                return;
            }

            rc = longRunningResponseHandler->registerResponseHandler(
                gpu::MessageType::PLATFORM_ENVIRONMENTAL,
                static_cast<uint8_t>(
                    gpu::PlatformEnvironmentalCommands::GET_ECC_MODE),
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
                    self->processGetLongRunningResponse(lrEc, lrCc, lrReason,
                                                        data);
                });

            if (rc != 0)
            {
                lg2::error("Error updating GPU ECC Mode: failed to register "
                           "long running handler, rc={RC}, EID={EID}, "
                           "IID={IID}",
                           "RC", rc, "EID", eid, "IID", responseInstanceId);
            }
            return;
        }

        default:
            lg2::error("Error updating GPU ECC Mode: GET unexpected completion "
                       "code, cc={CC}, reasonCode={RESC}, EID={EID}",
                       "CC", static_cast<uint8_t>(cc), "RESC", reasonCode,
                       "EID", eid);
            return;
    }
}

void NvidiaGpuEccMode::processGetLongRunningResponse(
    boost::system::error_code ec,
    ocp::accelerator_management::CompletionCode cc, uint16_t reasonCode,
    std::span<const uint8_t> responseData)
{
    if (ec)
    {
        lg2::error("GPU ECC Mode GET long running event failed, "
                   "ec={EC}, EID={EID}",
                   "EC", ec.message(), "EID", eid);
        return;
    }

    if (cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error("GPU ECC Mode GET long running indicated failure, "
                   "cc={CC}, reasonCode={RESC}, EID={EID}",
                   "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID",
                   eid);
        return;
    }

    if (responseData.empty())
    {
        lg2::error(
            "GPU ECC Mode GET long running response too short, EID={EID}",
            "EID", eid);
        return;
    }

    const uint8_t flags = responseData[0];
    const bool current = (flags & 0b01) != 0;
    const bool pending = (flags & 0b10) != 0;

    applyEccModeToDbus(current, pending);
}

void NvidiaGpuEccMode::applyEccModeToDbus(bool current, bool pending)
{
    currentInterface->set_property("Enabled", current);

    // internalUpdateInProgress suppresses the writable setter callback so
    // refreshing pending from hardware does not retrigger an NSM Set.
    internalUpdateInProgress = true;
    pendingInterface->set_property("Enabled", pending);
    internalUpdateInProgress = false;
}

void NvidiaGpuEccMode::onPendingSetRequested(bool desired)
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
            }
            return;
        }

        default:
            lg2::error("Error setting GPU ECC Mode: SET unexpected completion "
                       "code, cc={CC}, reasonCode={RESC}, EID={EID}",
                       "CC", static_cast<uint8_t>(cc), "RESC", reasonCode,
                       "EID", eid);
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
        return;
    }

    if (cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error("GPU ECC Mode SET long running indicated failure, "
                   "cc={CC}, reasonCode={RESC}, EID={EID}",
                   "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID",
                   eid);
    }
}
