/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "BulkTelemetryMessage.hpp"

#include "OcpVdmIana.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <vector>

namespace ocp::bulk
{

namespace
{
// Vendor Command Code(1) + Completion Code(1) + Telemetry Count(2).
constexpr size_t responseFixedFieldsSize = 4;

constexpr uint8_t explicitSizeBit = 0x80;
constexpr uint8_t validBit = 0x01;
constexpr uint8_t encodedLengthShift = 1;
constexpr uint8_t encodedLengthMask = 0x07;

bool decodeResponseFixedFields(
    std::span<const uint8_t> buf, uint8_t commandCode,
    CompletionCode& completionCode, uint16_t& telemetryCount)
{
    Header header{};
    if (!decodeHeader(buf, header) || header.type != MessageType::response)
    {
        return false;
    }

    std::span<const uint8_t> fields = buf.subspan(messageHeaderSize);
    if (fields.size() < responseFixedFieldsSize || fields[0] != commandCode)
    {
        return false;
    }

    completionCode = static_cast<CompletionCode>(fields[1]);
    telemetryCount = readLe<uint16_t>(fields.subspan(2, 2));
    return true;
}
} // namespace

std::optional<std::vector<uint8_t>> encodeRequestMessage(
    uint8_t instanceId, uint8_t commandCode, std::span<const uint8_t> payload)
{
    if (payload.size() > std::numeric_limits<uint16_t>::max())
    {
        return std::nullopt;
    }

    std::vector<uint8_t> buf(messageHeaderSize);
    if (!encodeHeader(Header{.type = MessageType::request,
                             .instanceId = instanceId},
                      buf))
    {
        return std::nullopt;
    }

    buf.push_back(commandCode);
    buf.push_back(0); // Reserved.

    const auto dataSize = static_cast<uint16_t>(payload.size());
    buf.push_back(static_cast<uint8_t>(dataSize));
    buf.push_back(static_cast<uint8_t>(dataSize >> 8));

    buf.insert(buf.end(), payload.begin(), payload.end());
    return buf;
}

bool decodeRecords(std::span<const uint8_t> buf, uint16_t count,
                   std::vector<Record>& records)
{
    records.clear();

    // Record tag(1) + flags(1).
    constexpr size_t recordHeaderSize = 2;

    size_t offset = 0;
    for (uint16_t i = 0; i < count; ++i)
    {
        if (buf.size() - offset < recordHeaderSize)
        {
            return false;
        }

        const uint8_t tag = buf[offset];
        const uint8_t flags = buf[offset + 1];
        offset += recordHeaderSize;

        size_t dataSize =
            size_t{1} << ((flags >> encodedLengthShift) & encodedLengthMask);
        if ((flags & explicitSizeBit) != 0)
        {
            if (buf.size() - offset < sizeof(uint16_t))
            {
                return false;
            }

            dataSize = readLe<uint16_t>(buf.subspan(offset));
            offset += sizeof(uint16_t);
        }

        if (buf.size() - offset < dataSize)
        {
            return false;
        }

        records.push_back(Record{
            .tag = tag,
            .valid = (flags & validBit) != 0,
            .data = buf.subspan(offset, dataSize),
        });
        offset += dataSize;
    }

    return true;
}

bool decodeResponseMessage(std::span<const uint8_t> buf, uint8_t commandCode,
                           CompletionCode& completionCode,
                           std::vector<Record>& records)
{
    uint16_t telemetryCount = 0;
    if (!decodeResponseFixedFields(buf, commandCode, completionCode,
                                   telemetryCount))
    {
        return false;
    }

    return decodeRecords(
        buf.subspan(messageHeaderSize + responseFixedFieldsSize),
        telemetryCount, records);
}

bool decodeEmptyResponse(std::span<const uint8_t> buf, uint8_t commandCode,
                         CompletionCode& completionCode)
{
    uint16_t telemetryCount = 0;
    return decodeResponseFixedFields(buf, commandCode, completionCode,
                                     telemetryCount) &&
           telemetryCount == 0;
}

bool isValidRecord(const Record& record, uint8_t tag)
{
    return record.tag == tag && record.valid;
}

} // namespace ocp::bulk
