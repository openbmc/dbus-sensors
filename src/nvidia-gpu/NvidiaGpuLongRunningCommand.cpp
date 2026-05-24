/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuLongRunningCommand.hpp"

#include <MctpRequester.hpp>
#include <MessagePackUnpackUtils.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaLongRunningHandler.hpp>
#include <OcpMctpVdm.hpp>
#include <SerialQueue.hpp>
#include <boost/system/error_code.hpp>
#include <phosphor-logging/lg2.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <system_error>
#include <utility>

NvidiaGpuLongRunningCommand::NvidiaGpuLongRunningCommand(
    uint8_t eid, mctp::MctpRequester& mctpRequester,
    std::shared_ptr<SerialQueue> longRunningQueue,
    std::shared_ptr<NvidiaLongRunningResponseHandler>
        longRunningResponseHandler,
    Config config) :
    eid(eid), mctpRequester(mctpRequester),
    longRunningQueue(std::move(longRunningQueue)),
    longRunningResponseHandler(std::move(longRunningResponseHandler)),
    config(std::move(config)), request(this->config.requestSize, 0)
{}

void NvidiaGpuLongRunningCommand::update()
{
    longRunningQueue->submit(
        [weak{weak_from_this()}](SerialQueue::ReleaseHandle handle) {
            std::shared_ptr<NvidiaGpuLongRunningCommand> self = weak.lock();
            if (!self)
            {
                return;
            }
            self->doUpdate(std::move(handle));
        });
}

void NvidiaGpuLongRunningCommand::doUpdate(SerialQueue::ReleaseHandle handle)
{
    const int rc = config.encodeRequest(request);

    if (rc != 0)
    {
        lg2::error("Error updating {NAME}: encode failed, rc={RC}, EID={EID}",
                   "NAME", config.metricName, "RC", rc, "EID", eid);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()},
         handle = std::move(handle)](const std::error_code& ec,
                                     std::span<const uint8_t> buffer) mutable {
            std::shared_ptr<NvidiaGpuLongRunningCommand> self = weak.lock();
            if (!self)
            {
                return;
            }
            self->processResponse(std::move(handle), ec, buffer);
        });
}

void NvidiaGpuLongRunningCommand::processResponse(
    SerialQueue::ReleaseHandle handle, const std::error_code& ec,
    std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("Error updating {NAME}: sending message over MCTP failed, "
                   "rc={RC}, EID={EID}",
                   "NAME", config.metricName, "RC", ec.message(), "EID", eid);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;

    UnpackBuffer unpackBuffer(buffer);

    int rc = gpu::decodeResponseCommonHeader(
        unpackBuffer, config.messageType,
        static_cast<uint8_t>(config.commandId), cc, reasonCode);

    if (rc != 0)
    {
        lg2::error("Error updating {NAME}: decode failed, "
                   "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}",
                   "NAME", config.metricName, "RC", rc, "CC",
                   static_cast<uint8_t>(cc), "RESC", reasonCode, "EID", eid);
        return;
    }

    switch (cc)
    {
        case ocp::accelerator_management::CompletionCode::SUCCESS:
        {
            config.onImmediateSuccess(buffer);
            return;
        }

        case ocp::accelerator_management::CompletionCode::ACCEPTED:
        {
            uint8_t instanceId = 0;
            rc = ocp::accelerator_management::decodeInstanceId(buffer,
                                                               instanceId);
            if (rc != 0)
            {
                lg2::error(
                    "Error updating {NAME}: failed to decode instance id, "
                    "rc={RC}, EID={EID}",
                    "NAME", config.metricName, "RC", rc, "EID", eid);
                return;
            }

            rc = longRunningResponseHandler->registerResponseHandler(
                config.messageType, static_cast<uint8_t>(config.commandId),
                instanceId,
                [weak{weak_from_this()}, handle = std::move(handle)](
                    boost::system::error_code longRunningEc,
                    ocp::accelerator_management::CompletionCode longRunningCc,
                    uint16_t longRunningReasonCode,
                    std::span<const uint8_t> responseData) mutable {
                    std::shared_ptr<NvidiaGpuLongRunningCommand> self =
                        weak.lock();
                    if (!self)
                    {
                        return;
                    }

                    self->processLongRunningResponse(
                        longRunningEc, longRunningCc, longRunningReasonCode,
                        responseData);
                });

            if (rc != 0)
            {
                lg2::error(
                    "Error updating {NAME}: failed to register long running "
                    "response handler, rc={RC}, EID={EID}",
                    "NAME", config.metricName, "RC", rc, "EID", eid);
            }

            return;
        }

        default:
            lg2::error(
                "Error updating {NAME}: received unexpected completion code, "
                "cc={CC}, reasonCode={RESC}, EID={EID}",
                "NAME", config.metricName, "CC", static_cast<uint8_t>(cc),
                "RESC", reasonCode, "EID", eid);
            return;
    }
}

void NvidiaGpuLongRunningCommand::processLongRunningResponse(
    boost::system::error_code ec,
    ocp::accelerator_management::CompletionCode cc, uint16_t reasonCode,
    std::span<const uint8_t> responseData)
{
    if (ec)
    {
        lg2::error("Error updating {NAME}: long running response failed, "
                   "rc={RC}, EID={EID}",
                   "NAME", config.metricName, "RC", ec.message(), "EID", eid);
        return;
    }

    if (cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating {NAME}: long running response indicated failure, "
            "cc={CC}, reasonCode={RESC}, EID={EID}",
            "NAME", config.metricName, "CC", static_cast<uint8_t>(cc), "RESC",
            reasonCode, "EID", eid);
        return;
    }

    UnpackBuffer buf(responseData);
    config.onLongRunningPayload(buf);
}
