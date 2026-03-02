#include "NvidiaGpuMctpVdm.hpp"

#include <MctpRequester.hpp>
#include <NvidiaEventReporting.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/container_hash/hash.hpp>
#include <phosphor-logging/lg2.hpp>

#include <array>
#include <cstdint>
#include <initializer_list>
#include <span>
#include <system_error>
#include <unordered_map>

std::unordered_map<NvidiaEventHandler::EventKey, EventHandler,
                   boost::hash<NvidiaEventHandler::EventKey>>
    NvidiaEventHandler::eventHandlers;

static std::array<uint64_t, messageTypeCount> buildEventSourceMasks(
    std::initializer_list<EventDescriptor> events)
{
    std::array<uint64_t, messageTypeCount> masks{};

    for (const auto& event : events)
    {
        const uint8_t idx = static_cast<uint8_t>(event.messageType);

        if (idx >= messageTypeCount)
        {
            lg2::error(
                "Invalid message type {MSG} for event code {CODE}, skipping",
                "MSG", idx, "CODE", event.eventCode);
            continue;
        }

        masks[idx] |= (1ULL << event.eventCode);
    }

    return masks;
}

NvidiaEventReportingConfig::NvidiaEventReportingConfig(
    uint8_t eid, mctp::MctpRequester& req,
    std::initializer_list<EventDescriptor> events) :
    eid{eid}, requester{req}, eventMasks{buildEventSourceMasks(events)}
{
    for (const auto& event : events)
    {
        NvidiaEventHandler::registerEventHandler(
            eid, event.messageType, event.eventCode, event.eventHandler);
    }
}

void NvidiaEventReportingConfig::init()
{
    int rc = gpu::encodeSetEventSubscriptionRequest(generationSettingEnablePush,
                                                    bmc_eid, subscriptionReq);
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

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    int rc = gpu::decodeSetEventSubscriptionResponse(buffer, cc, reasonCode);
    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error("failed to setup event subscription on eid {EID}: "
                   "rc={RC}, cc={CC}, reasonCode={RESC}",
                   "EID", eid, "RC", rc, "CC", cc, "RESC", reasonCode);
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

        const int rc =
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
        lg2::info("Finished setting up event sources for eid {EID}", "EID",
                  eid);
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
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;

    const int rc = gpu::decodeSetEventSourcesResponse(buffer, cc, reasonCode);
    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error("failed to set event sources on eid {EID}: "
                   "rc={RC}, cc={CC}, reasonCode={RESC}",
                   "EID", eid, "RC", rc, "CC", cc, "RESC", reasonCode);
        return;
    }

    // Move to next message type and continue
    currentMessageTypeIdx++;
    sendNextEventSource();
}

void NvidiaEventHandler::handleEvent(uint8_t eid,
                                     std::span<const uint8_t> buffer)
{
    EventInfo eventInfo{};
    uint8_t messageType = 0;
    uint8_t eventId = 0;
    uint8_t eventSize = 0;
    std::span<const uint8_t> eventData;

    const int rc = ocp::accelerator_management::decodeEvent(
        buffer, messageType, eventInfo.ackRequired, eventInfo.version, eventId,
        eventInfo.eventClass, eventInfo.eventState, eventSize, eventData);

    if (rc != 0)
    {
        lg2::error("Failed to decode event log");
        return;
    }

    EventKey key{eid, static_cast<gpu::MessageType>(messageType), eventId};
    const auto itr = eventHandlers.find(key);
    if (itr == eventHandlers.end())
    {
        lg2::error(
            "No handler registered for event code {CODE} message type {MSG} from eid {EID}",
            "CODE", eventId, "MSG", messageType, "EID", eid);
        return;
    }

    itr->second(eventInfo, eventData);
}
