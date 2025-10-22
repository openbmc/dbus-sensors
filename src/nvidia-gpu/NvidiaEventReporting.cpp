#include "NvidiaGpuMctpVdm.hpp"

#include <MctpRequester.hpp>
#include <NvidiaEventReporting.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <span>
#include <string_view>
#include <system_error>

static constexpr uint8_t xidEventCode = 1;
static constexpr uint8_t xidEventMsgType = 3;

struct EventDescriptor
{
    gpu::MessageType messageType;
    uint8_t bitOffset;
};

static EventDescriptor getEventDescriptor(EventType event)
{
    switch (event)
    {
        case EventType::Xid:
            return {gpu::MessageType::PLATFORM_ENVIRONMENTAL, 1};
    }
    // Unreachable, but needed for compiler
    return {gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY, 0};
}

static std::array<uint64_t, messageTypeCount> buildEventSourceMasks(
    std::initializer_list<EventType> events)
{
    std::array<uint64_t, messageTypeCount> masks{};
    for (EventType event : events)
    {
        auto desc = getEventDescriptor(event);
        auto idx = static_cast<size_t>(desc.messageType);
        masks[idx] |= (1ULL << desc.bitOffset);
    }
    return masks;
}

NvidiaEventReportingConfig::NvidiaEventReportingConfig(
    uint8_t eid, mctp::MctpRequester& req,
    std::initializer_list<EventType> events) :
    eid{eid}, requester{req}, eventMasks{buildEventSourceMasks(events)}
{}

void NvidiaEventReportingConfig::init()
{
    int rc = gpu::encodeSetEventSubscriptionRequest(bmc_eid, subscriptionReq);
    if (rc != 0)
    {
        lg2::error("Failed to setup device subscription for eid {EID}", "EID",
                   eid);
        return;
    }

    requester.sendRecvMsg(
        eid, subscriptionReq,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            auto self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaEventReportingConfig");
            }
            self->handleSetupSubscription(ec, buffer);
        });
}

void NvidiaEventReportingConfig::handleSetupSubscription(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("failed to setup event subscription on eid {EID}: {EC}",
                   "EID", eid, "EC", ec.message());
        return;
    }

    uint8_t cc = 0;
    int rc = gpu::decodeSetEventSubscriptionResponse(buffer, cc);
    if ((rc != 0) || (cc != 0U))
    {
        lg2::error("failed to setup event subscription on eid {EID} cc: {CC}",
                   "EID", eid, "CC", cc);
        return;
    }

    // Device is set up for push notifications, now configure event sources
    currentMessageTypeIdx = 0;
    sendNextEventSource();
}

void NvidiaEventReportingConfig::sendNextEventSource()
{
    // Find next message type with events that can be encoded
    while (currentMessageTypeIdx < messageTypeCount)
    {
        // Skip message types with no events
        if (eventMasks[currentMessageTypeIdx] == 0)
        {
            currentMessageTypeIdx++;
            continue;
        }

        uint64_t events = eventMasks[currentMessageTypeIdx];
        auto messageType = static_cast<uint8_t>(currentMessageTypeIdx);

        int rc =
            gpu::encodeSetEventSourcesRequest(events, messageType, sourcesReq);
        if (rc != 0)
        {
            lg2::error(
                "Failed to encode event sources request for EID {EID} messageType {MSG}",
                "EID", eid, "MSG", messageType);
            currentMessageTypeIdx++;
            continue;
        }

        break;
    }

    if (currentMessageTypeIdx >= messageTypeCount)
    {
        // All event sources processed
        return;
    }

    requester.sendRecvMsg(
        eid, sourcesReq,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            auto self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaEventReportingConfig");
                return;
            }
            self->handleSetupEvents(ec, buffer);
        });
}

void NvidiaEventReportingConfig::handleSetupEvents(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("failed to set up events for eid {EID}: {MSG}", "EID", eid,
                   "MSG", ec.message());
    }
    else
    {
        uint8_t cc = 0;
        int rc = gpu::decodeSetEventSourcesResponse(buffer, cc);
        if ((rc != 0) || (cc != 0U))
        {
            lg2::error(
                "failed to set event sources on eid {EID}, rc {RC} cc {CC}",
                "EID", eid, "RC", rc, "CC", cc);
        }
    }

    // Move to next message type and continue
    currentMessageTypeIdx++;
    sendNextEventSource();
}

void NvidiaEventHandler::handleEvent(uint8_t eid,
                                     std::span<const uint8_t> buffer)
{
    ocp::accelerator_management::Event event = {};
    std::span<const uint8_t> eventData;

    lg2::info("received event on eid {EID}", "EID", eid);

    int rc = gpu::decodeEvent(buffer, event, eventData);
    if (rc != 0)
    {
        lg2::error("Failed to decode event log");
        return;
    }

    switch (event.hdr.ocp_accelerator_management_msg_type)
    {
        case xidEventMsgType:
            NvidiaEventHandler::handlePlatformEvent(eid, eventData,
                                                    event.eventId);
            break;
        default:
            lg2::error("Invalid message type received on eid {EID}", "EID",
                       eid);
    }
}

void NvidiaEventHandler::handlePlatformEvent(
    uint8_t eid, std::span<const uint8_t> buffer, uint8_t messageType)
{
    if (messageType != xidEventCode)
    {
        lg2::error("unsupported message type {MSG} on eid {EID}", "MSG",
                   messageType, "EID", eid);
    }

    gpu::XidEvent event{};
    std::string_view message;
    int rc = gpu::decodeXidEvent(buffer, event, message);
    if (rc != 0)
    {
        lg2::error("Failed to decode xid event");
        return;
    }

    uint32_t reason = event.reason;
    lg2::info("eid: {EID}, reason: {RSN}, message from xid: {XID}", "EID", eid,
              "RSN", reason, "XID", message);
}
