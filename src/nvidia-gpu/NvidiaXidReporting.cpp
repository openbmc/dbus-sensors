#include "NvidiaGpuMctpVdm.hpp"

#include <MctpRequester.hpp>
#include <NvidiaXidReporting.hpp>
#include <phosphor-logging/lg2.hpp>

#include <cstdint>
#include <span>
#include <string_view>
#include <system_error>

static constexpr uint8_t xidEventCode = 1;
static constexpr uint8_t xidEventMsgType = 3;

NvidiaEventReportingConfig::NvidiaEventReportingConfig(
    uint8_t eid, mctp::MctpRequester& req) : eid{eid}, requester{req}
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

    // if we got here, the device is set up for push notifications
    // set up our events
    // for now, we only support xid eventing
    // if we need to support more than just xid events,
    // this will need to be expanded
    uint64_t events = 1 << xidEventCode;
    rc = gpu::encodeSetEventSourcesRequest(events, xidEventMsgType, sourcesReq);
    if (rc != 0)
    {
        lg2::error("Failed to encode event sources request for EID {EID}",
                   "EID", eid);
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

    uint8_t cc = 0;
    int rc = gpu::decodeSetEventSourcesResponse(buffer, cc);
    if ((rc != 0) || (cc != 0U))
    {
        lg2::error("failed to set event sources on eid {EID}, rc {RC} cc {CC}",
                   "EID", eid, "RC", rc, "CC", cc);
    }
}

void NvidiaEventHandler::handleEvent(uint8_t eid,
                                     std::span<const uint8_t> buffer)
{
    gpu::Event event = {};
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
