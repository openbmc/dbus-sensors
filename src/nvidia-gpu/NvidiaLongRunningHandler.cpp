/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaLongRunningHandler.hpp"

#include <NvidiaEventReporting.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/system/error_code.hpp>
#include <phosphor-logging/lg2.hpp>

#include <cerrno>
#include <cstdint>
#include <span>
#include <system_error>
#include <utility>

NvidiaLongRunningResponseHandler::NvidiaLongRunningResponseHandler(
    boost::asio::io_context& io) : io(io)
{}

int NvidiaLongRunningResponseHandler::registerResponseHandler(
    gpu::MessageType messageType, uint8_t commandCode, uint8_t instanceId,
    ResponseHandler handler)
{
    const ResponseKey key{messageType, commandCode, instanceId};

    auto [it, inserted] = entries.try_emplace(key, std::move(handler), io);
    if (!inserted)
    {
        return EEXIST;
    }

    it->second.timer.expires_after(longRunningResponseTimeout);
    it->second.timer.async_wait(
        [weak = weak_from_this(),
         key](const boost::system::error_code& ec) mutable {
            if (ec == boost::asio::error::operation_aborted)
            {
                return;
            }
            if (auto self = weak.lock())
            {
                self->onTimeout(key);
            }
        });

    return 0;
}

void NvidiaLongRunningResponseHandler::onTimeout(ResponseKey key)
{
    auto entryIt = entries.find(key);
    if (entryIt == entries.end())
    {
        return;
    }

    ResponseHandler localHandler = std::move(entryIt->second.handler);
    entries.erase(entryIt);

    lg2::error(
        "Long running response timed out: MessageType={MT}, CommandCode={CC}, InstanceId={IID}",
        "MT", static_cast<uint8_t>(std::get<0>(key)), "CC", std::get<1>(key),
        "IID", std::get<2>(key));

    localHandler(make_error_code(std::errc::timed_out),
                 ocp::accelerator_management::CompletionCode::ERROR, 0,
                 std::span<const uint8_t>{});
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

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint8_t instanceId = 0;
    std::span<const uint8_t> responseData;

    const int rc = gpu::decodeLongRunningResponseEvent(
        eventData, cc, reasonCode, instanceId, responseData);

    if (rc != 0)
    {
        lg2::error("Failed to decode long running response event, rc={RC}, "
                   "MessageType={MT}, CommandCode={CC}",
                   "RC", rc, "MT", static_cast<uint8_t>(messageType), "CC",
                   commandCode);
        return;
    }

    const ResponseKey key{messageType, commandCode, instanceId};
    auto entryIt = entries.find(key);

    if (entryIt == entries.end())
    {
        lg2::error("No handler registered for long running response event with "
                   "MessageType={MT}, CommandCode={CC}, InstanceId={IID}",
                   "MT", static_cast<uint8_t>(messageType), "CC", commandCode,
                   "IID", instanceId);
        return;
    }

    entryIt->second.timer.cancel();
    ResponseHandler localHandler = std::move(entryIt->second.handler);
    entries.erase(entryIt);

    localHandler(boost::system::error_code{}, cc, reasonCode, responseData);
}
