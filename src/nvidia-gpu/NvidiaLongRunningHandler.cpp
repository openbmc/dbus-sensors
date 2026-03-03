/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaLongRunningHandler.hpp"

#include <NvidiaEventReporting.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>

#include <cerrno>
#include <cstdint>
#include <span>

int NvidiaLongRunningResponseHandler::registerResponseHandler(
    gpu::MessageType messageType, uint8_t commandCode,
    const ResponseHandler& handler)
{
    const auto handlerIt = responseHandlers.find({messageType, commandCode});
    if (handlerIt != responseHandlers.end())
    {
        return EEXIST;
    }

    responseHandlers[{messageType, commandCode}] = handler;

    return 0;
}

void NvidiaLongRunningResponseHandler::handler(
    const EventInfo& eventInfo, std::span<const uint8_t> eventData)
{
    if (eventInfo.eventClass != longRunningResponseEventClass)
    {
        lg2::error(
            "Received long running response event with unsupported event class: {EVENT_CLASS}",
            "EVENT_CLASS", eventInfo.eventClass);
        return;
    }

    const gpu::MessageType messageType =
        static_cast<gpu::MessageType>(eventInfo.eventState & 0xFF);
    const uint8_t commandCode =
        static_cast<uint8_t>((eventInfo.eventState >> 8) & 0xFF);

    auto handlerIt = responseHandlers.find({messageType, commandCode});

    if (handlerIt == responseHandlers.end())
    {
        lg2::error("No handler registered for long running response event with "
                   "MessageType={MT} and CommandCode={CC}",
                   "MT", static_cast<uint8_t>(messageType), "CC", commandCode);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint8_t instanceId = 0;
    std::span<const uint8_t> responseData;

    const int rc = gpu::decodeLongRunningResponseEvent(
        eventData, cc, reasonCode, instanceId, responseData);

    if (rc == 0)
    {
        handlerIt->second(cc, reasonCode, responseData);
    }
    else
    {
        lg2::error("Failed to decode long running response event, rc={RC}, "
                   "MessageType={MT}, CommandCode={CC}",
                   "RC", rc, "MT", static_cast<uint8_t>(messageType), "CC",
                   commandCode);
    }

    responseHandlers.erase(handlerIt);
}
