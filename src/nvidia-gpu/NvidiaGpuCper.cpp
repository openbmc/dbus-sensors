/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuCper.hpp"

#include "NvidiaGpuMctpVdm.hpp"

#include <MctpRequester.hpp>
#include <NvidiaEventReporting.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace
{

constexpr auto requestInterval = std::chrono::milliseconds{150};
constexpr uint16_t pendingEventHandle = 0;
constexpr uint16_t noMoreEventHandles = std::numeric_limits<uint16_t>::max();

std::string encodeBase64(std::span<const uint8_t> data)
{
    static constexpr std::string_view alphabet =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string encoded;
    encoded.reserve(((data.size() + 2) / 3) * 4);

    for (size_t index = 0; index < data.size(); index += 3)
    {
        const uint32_t first = data[index];
        const uint32_t second = index + 1 < data.size() ? data[index + 1] : 0;
        const uint32_t third = index + 2 < data.size() ? data[index + 2] : 0;
        const uint32_t value = (first << 16) | (second << 8) | third;

        encoded.push_back(alphabet[(value >> 18) & 0x3F]);
        encoded.push_back(alphabet[(value >> 12) & 0x3F]);
        encoded.push_back(
            index + 1 < data.size() ? alphabet[(value >> 6) & 0x3F] : '=');
        encoded.push_back(
            index + 2 < data.size() ? alphabet[value & 0x3F] : '=');
    }
    return encoded;
}

std::string cperSeverityName(uint32_t severity)
{
    switch (severity)
    {
        case 0:
            return "Recoverable";
        case 1:
            return "Fatal";
        case 2:
            return "Corrected";
        case 3:
            return "Informational";
        default:
            return "Unknown";
    }
}

std::string dbusSeverity(uint32_t severity)
{
    switch (severity)
    {
        case 0:
            return "xyz.openbmc_project.Logging.Entry.Level.Warning";
        case 1:
            return "xyz.openbmc_project.Logging.Entry.Level.Critical";
        case 2:
        case 3:
            return "xyz.openbmc_project.Logging.Entry.Level.Informational";
        default:
            return "xyz.openbmc_project.Logging.Entry.Level.Warning";
    }
}

} // namespace

NvidiaCperEventHandler::NvidiaCperEventHandler(
    uint8_t eid, boost::asio::io_context& io, mctp::MctpRequester& requester,
    const std::shared_ptr<sdbusplus::asio::connection>& conn) :
    eid(eid), requestTimer(io), requester(requester), conn(conn)
{
    recordData.reserve(gpu::maxCperRecordSize);
}

void NvidiaCperEventHandler::handleCperEvent(const EventInfo& eventInfo,
                                             std::span<const uint8_t> eventData)
{
    if ((eventInfo.version != 0 && eventInfo.version != 1) ||
        eventInfo.eventClass != static_cast<uint8_t>(gpu::EventClass::POLLED) ||
        !eventData.empty())
    {
        lg2::error(
            "Invalid CPER event from EID {EID}: version={VER}, class={CLASS}, dataSize={SIZE}",
            "EID", eid, "VER", eventInfo.version, "CLASS", eventInfo.eventClass,
            "SIZE", eventData.size());
        return;
    }

    if (eventInfo.eventState == noMoreEventHandles)
    {
        lg2::warning(
            "Ignoring reserved CPER event handle 0xFFFF from EID {EID}", "EID",
            eid);
        return;
    }

    if (eventHandles.size() >= maxEventHandles)
    {
        const uint16_t droppedHandle = eventHandles.front();
        eventHandles.pop_front();
        lg2::warning(
            "CPER event queue full for EID {EID}; dropping oldest handle {HANDLE}",
            "EID", eid, "HANDLE", droppedHandle);
    }
    eventHandles.push_back(eventInfo.eventState);

    if (!collectionInProgress)
    {
        startNextRecord();
    }
}

void NvidiaCperEventHandler::startNextRecord()
{
    if (eventHandles.empty())
    {
        collectionInProgress = false;
        return;
    }

    requestedEventHandle = eventHandles.front();
    eventHandles.pop_front();
    recordEventHandle = requestedEventHandle;
    transferHandle = 0;
    getAttempts = 1;
    ackAttempts = 0;
    shouldLogRecord = false;
    mode = gpu::EventLogRecordV2Mode::GET_DATA;
    recordData.clear();
    transferHandles.clear();
    recordInfo.reset();
    collectionInProgress = true;
    scheduleRequest();
}

void NvidiaCperEventHandler::scheduleRequest()
{
    requestTimer.expires_after(requestInterval);
    requestTimer.async_wait(
        [weak{weak_from_this()}](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                return;
            }
            std::shared_ptr<NvidiaCperEventHandler> self = weak.lock();
            if (!self)
            {
                return;
            }
            if (ec)
            {
                if (self->mode == gpu::EventLogRecordV2Mode::GET_DATA)
                {
                    self->handleGetFailure(ec.message());
                }
                else
                {
                    self->handleAckFailure(ec.message());
                }
                return;
            }
            self->sendRequest();
        });
}

void NvidiaCperEventHandler::sendRequest()
{
    const uint16_t eventHandle =
        mode == gpu::EventLogRecordV2Mode::ACKNOWLEDGEMENT
            ? recordEventHandle
            : requestedEventHandle;
    const uint16_t requestTransferHandle =
        mode == gpu::EventLogRecordV2Mode::ACKNOWLEDGEMENT ? 0 : transferHandle;

    const int rc = gpu::encodeGetEventLogRecordV2Request(
        0, mode, eventHandle, requestTransferHandle, request);
    if (rc != 0)
    {
        const std::string reason =
            std::format("failed to encode request: rc={}", rc);
        if (mode == gpu::EventLogRecordV2Mode::GET_DATA)
        {
            handleGetFailure(reason);
        }
        else
        {
            handleAckFailure(reason);
        }
        return;
    }

    requester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> response) {
            std::shared_ptr<NvidiaCperEventHandler> self = weak.lock();
            if (!self)
            {
                return;
            }
            self->handleResponse(ec, response);
        });
}

void NvidiaCperEventHandler::handleResponse(const std::error_code& ec,
                                            std::span<const uint8_t> response)
{
    if (ec)
    {
        if (mode == gpu::EventLogRecordV2Mode::GET_DATA)
        {
            handleGetFailure(ec.message());
        }
        else
        {
            handleAckFailure(ec.message());
        }
        return;
    }

    if (mode == gpu::EventLogRecordV2Mode::GET_DATA)
    {
        handleGetResponse(response);
    }
    else
    {
        handleAckResponse(response);
    }
}

void NvidiaCperEventHandler::handleGetResponse(
    std::span<const uint8_t> response)
{
    ocp::accelerator_management::CompletionCode completionCode{};
    uint16_t reasonCode = 0;

    if (transferHandle == 0)
    {
        gpu::EventLogRecordV2FirstResponse fields{};
        const int rc = gpu::decodeGetEventLogRecordV2FirstResponse(
            response, completionCode, reasonCode, fields);
        if (rc != 0 || completionCode !=
                           ocp::accelerator_management::CompletionCode::SUCCESS)
        {
            handleGetFailure(std::format(
                "first response failed: rc={}, cc={}, reason={}", rc,
                static_cast<uint8_t>(completionCode), reasonCode));
            return;
        }

        if (!fields.hasEventRecord)
        {
            eventHandles.clear();
            recordData.clear();
            collectionInProgress = false;
            startNextRecord();
            return;
        }

        if (requestedEventHandle != pendingEventHandle &&
            fields.eventHandle != requestedEventHandle)
        {
            handleGetFailure(std::format(
                "first response event handle mismatch: event={}, expected={}",
                fields.eventHandle, requestedEventHandle));
            return;
        }

        recordEventHandle = fields.eventHandle;
        if (fields.nvidiaMessageType !=
                static_cast<uint8_t>(
                    gpu::MessageType::PLATFORM_ENVIRONMENTAL) ||
            fields.eventId !=
                static_cast<uint8_t>(gpu::PlatformEnvironmentalEvent::CPER) ||
            fields.eventClass !=
                static_cast<uint8_t>(gpu::EventClass::POLLED) ||
            (fields.eventVersion != 0 && fields.eventVersion != 1))
        {
            lg2::error(
                "Invalid CPER record metadata from EID {EID}: messageType={TYPE}, version={VER}, eventId={EVENT}, class={CLASS}",
                "EID", eid, "TYPE", fields.nvidiaMessageType, "VER",
                fields.eventVersion, "EVENT", fields.eventId, "CLASS",
                fields.eventClass);
            recordData.clear();
            beginAcknowledgement(false);
            return;
        }

        if (!appendRecordData(fields.eventData))
        {
            beginAcknowledgement(false);
            return;
        }
        transferHandle = fields.nextTransferHandle;
        if (transferHandle != 0)
        {
            transferHandles.insert(transferHandle);
        }
    }
    else
    {
        gpu::EventLogRecordV2NextResponse fields{};
        const int rc = gpu::decodeGetEventLogRecordV2NextResponse(
            response, completionCode, reasonCode, fields);
        if (rc != 0 || completionCode !=
                           ocp::accelerator_management::CompletionCode::SUCCESS)
        {
            handleGetFailure(std::format(
                "continuation response failed: rc={}, cc={}, reason={}", rc,
                static_cast<uint8_t>(completionCode), reasonCode));
            return;
        }

        if (fields.eventHandle != recordEventHandle ||
            (fields.nextTransferHandle != 0 &&
             !transferHandles.insert(fields.nextTransferHandle).second))
        {
            handleGetFailure(std::format(
                "invalid continuation handles: event={}, expected={}, transfer={}",
                fields.eventHandle, recordEventHandle,
                fields.nextTransferHandle));
            return;
        }
        if (!appendRecordData(fields.eventData))
        {
            beginAcknowledgement(false);
            return;
        }
        transferHandle = fields.nextTransferHandle;
    }

    if (transferHandle != 0)
    {
        scheduleRequest();
        return;
    }

    gpu::CperRecordInfo info{};
    const int rc = gpu::decodeCperRecord(recordData, info);
    if (rc != 0)
    {
        lg2::error(
            "Discarding malformed CPER record from EID {EID}: rc={RC}, size={SIZE}",
            "EID", eid, "RC", rc, "SIZE", recordData.size());
        beginAcknowledgement(false);
        return;
    }
    if (info.timestampInvalid)
    {
        lg2::warning(
            "CPER record from EID {EID} has an invalid BCD timestamp; omitting timestamp metadata",
            "EID", eid);
    }
    recordInfo.emplace(std::move(info));
    beginAcknowledgement(true);
}

void NvidiaCperEventHandler::handleAckResponse(
    std::span<const uint8_t> response)
{
    ocp::accelerator_management::CompletionCode completionCode{};
    uint16_t reasonCode = 0;
    gpu::EventLogRecordV2NextResponse fields{};
    const int rc = gpu::decodeGetEventLogRecordV2NextResponse(
        response, completionCode, reasonCode, fields);
    if (rc != 0 ||
        completionCode !=
            ocp::accelerator_management::CompletionCode::SUCCESS ||
        fields.nextTransferHandle != 0 || !fields.eventData.empty())
    {
        handleAckFailure(
            std::format("response failed: rc={}, cc={}, reason={}", rc,
                        static_cast<uint8_t>(completionCode), reasonCode));
        return;
    }

    if (fields.eventHandle == noMoreEventHandles)
    {
        eventHandles.clear();
    }
    else if (fields.eventHandle == pendingEventHandle && eventHandles.empty())
    {
        eventHandles.push_back(pendingEventHandle);
    }
    else if (fields.eventHandle != pendingEventHandle)
    {
        handleAckFailure(
            std::format("invalid next event handle: {}", fields.eventHandle));
        return;
    }

    finishRecord();
}

void NvidiaCperEventHandler::handleGetFailure(const std::string& reason)
{
    lg2::error(
        "Failed to retrieve CPER record from EID {EID}, handle={HANDLE}, attempt={ATTEMPT}: {REASON}",
        "EID", eid, "HANDLE", requestedEventHandle, "ATTEMPT", getAttempts,
        "REASON", reason);

    if (getAttempts < maxGetAttempts)
    {
        ++getAttempts;
        transferHandle = 0;
        recordEventHandle = requestedEventHandle;
        recordData.clear();
        transferHandles.clear();
        recordInfo.reset();
        scheduleRequest();
        return;
    }

    recordData.clear();
    recordInfo.reset();
    beginAcknowledgement(false);
}

void NvidiaCperEventHandler::handleAckFailure(const std::string& reason)
{
    lg2::error(
        "Failed to acknowledge CPER record from EID {EID}, handle={HANDLE}, attempt={ATTEMPT}: {REASON}",
        "EID", eid, "HANDLE", recordEventHandle, "ATTEMPT", ackAttempts,
        "REASON", reason);

    if (ackAttempts < maxAckAttempts)
    {
        ++ackAttempts;
        scheduleRequest();
        return;
    }

    finishRecord();
}

bool NvidiaCperEventHandler::appendRecordData(std::span<const uint8_t> data)
{
    if (recordData.size() > gpu::maxCperRecordSize ||
        data.size() > gpu::maxCperRecordSize - recordData.size())
    {
        lg2::error(
            "CPER record from EID {EID} exceeds maximum size {MAX}; dropping record",
            "EID", eid, "MAX", gpu::maxCperRecordSize);
        recordData.clear();
        recordInfo.reset();
        return false;
    }

    recordData.insert(recordData.end(), data.begin(), data.end());
    return true;
}

void NvidiaCperEventHandler::beginAcknowledgement(bool logRecord)
{
    shouldLogRecord = logRecord;
    mode = gpu::EventLogRecordV2Mode::ACKNOWLEDGEMENT;
    transferHandle = 0;
    ackAttempts = 1;
    scheduleRequest();
}

void NvidiaCperEventHandler::finishRecord()
{
    if (shouldLogRecord && recordInfo.has_value())
    {
        logRecord();
    }

    shouldLogRecord = false;
    recordData.clear();
    transferHandles.clear();
    recordInfo.reset();
    collectionInProgress = false;
    startNextRecord();
}

void NvidiaCperEventHandler::logRecord()
{
    if (!recordInfo.has_value() || recordData.empty())
    {
        return;
    }
    const gpu::CperRecordInfo& info = recordInfo.value();

    std::map<std::string, std::string> additionalData{
        {"REDFISH_MESSAGE_ID", "Platform.1.0.PlatformError"},
        {"cperSeverity", cperSeverityName(info.severity)},
        {"cperSeverityCode", std::to_string(info.severity)},
        {"diagnosticData", encodeBase64(recordData)},
        {"diagnosticDataType", "CPER"},
        {"notificationType", info.notificationType},
    };
    if (info.timestamp.has_value())
    {
        additionalData.emplace("timestamp", info.timestamp.value());
    }

    for (const std::string& sectionType : info.sectionTypes)
    {
        additionalData["sectionType"] = sectionType;
        conn->async_method_call(
            [weak{weak_from_this()}](const boost::system::error_code& ec) {
                std::shared_ptr<NvidiaCperEventHandler> self = weak.lock();
                if (!self)
                {
                    lg2::error("Invalid reference to NvidiaCperEventHandler");
                    return;
                }
                self->handleLoggingComplete(ec);
            },
            "xyz.openbmc_project.Logging", "/xyz/openbmc_project/logging",
            "xyz.openbmc_project.Logging.Create", "Create", "A CPER was logged",
            dbusSeverity(info.severity), additionalData);
    }
}

void NvidiaCperEventHandler::handleLoggingComplete(
    const boost::system::error_code& ec)
{
    if (ec)
    {
        lg2::error("Failed to log CPER event from EID {EID}: {EC}", "EID", eid,
                   "EC", ec.message());
    }
}
