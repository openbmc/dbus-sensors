/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "BulkTelemetryMessage.hpp"
#include "OcpVdmIana.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include <gtest/gtest.h>

namespace ocp::bulk
{
namespace
{

// Enterprise ID 0xA67F big endian, Rq/D/IID, OCP type and version, vendor
// message type.
constexpr std::array<uint8_t, messageHeaderSize> requestHeaderBytes{
    0x00, 0x00, 0xA6, 0x7F, 0x83, 0x89, 0x01};
constexpr std::array<uint8_t, messageHeaderSize> responseHeaderBytes{
    0x00, 0x00, 0xA6, 0x7F, 0x03, 0x89, 0x01};

std::vector<uint8_t> withResponseHeader(std::span<const uint8_t> body)
{
    std::vector<uint8_t> buf(responseHeaderBytes.begin(),
                             responseHeaderBytes.end());
    buf.insert(buf.end(), body.begin(), body.end());
    return buf;
}

TEST(OcpVdmIana, EncodesRequestHeader)
{
    std::array<uint8_t, messageHeaderSize> buf{};
    ASSERT_TRUE(encodeHeader(
        Header{.type = MessageType::request, .instanceId = 3}, buf));
    EXPECT_EQ(buf, requestHeaderBytes);
}

TEST(OcpVdmIana, RejectsShortBuffer)
{
    std::array<uint8_t, messageHeaderSize - 1> buf{};
    Header header{};
    EXPECT_FALSE(encodeHeader(
        Header{.type = MessageType::request, .instanceId = 0}, buf));
    EXPECT_FALSE(decodeHeader(buf, header));
}

TEST(OcpVdmIana, RoundTripsEveryMessageType)
{
    for (const MessageType type :
         {MessageType::request, MessageType::response, MessageType::event,
          MessageType::eventAcknowledgment})
    {
        std::array<uint8_t, messageHeaderSize> buf{};
        ASSERT_TRUE(encodeHeader(Header{.type = type, .instanceId = 17}, buf));

        Header decoded{};
        ASSERT_TRUE(decodeHeader(buf, decoded));
        EXPECT_EQ(decoded.type, type);
        EXPECT_EQ(decoded.instanceId, 17);
    }
}

TEST(OcpVdmIana, RejectsForeignMessages)
{
    Header header{};

    auto buf = requestHeaderBytes;
    buf[3] = 0x7E;
    EXPECT_FALSE(decodeHeader(buf, header));

    buf = requestHeaderBytes;
    buf[5] &= 0x7F;
    EXPECT_FALSE(decodeHeader(buf, header));

    buf = requestHeaderBytes;
    buf[5] = 0x88;
    EXPECT_FALSE(decodeHeader(buf, header));

    buf = requestHeaderBytes;
    buf[6] = 0x02;
    EXPECT_FALSE(decodeHeader(buf, header));
}

TEST(BulkTelemetryMessage, EncodesRequestMessage)
{
    constexpr std::array<uint8_t, 2> payload{0xAA, 0xBB};
    const auto buf = encodeRequestMessage(3, 0x13, payload);
    ASSERT_TRUE(buf.has_value());

    const std::vector<uint8_t> expected{0x00, 0x00, 0xA6, 0x7F, 0x83,
                                        0x89, 0x01, 0x13, 0x00, 0x02,
                                        0x00, 0xAA, 0xBB};
    EXPECT_EQ(*buf, expected);
}

TEST(BulkTelemetryMessage, DecodesRecords)
{
    constexpr std::array<uint8_t, 16> body{
        0x20, 0x00, 0x03, 0x00,             // command, cc, count
        0x00, 0x05, 0x01, 0x02, 0x03, 0x04, // tag 0, 4 bytes
        0x03, 0x02, 0x00, 0x00,             // tag 3, invalid
        0x05, 0x81,                         // tag 5, explicit size
    };
    auto buf = withResponseHeader(body);
    buf.insert(buf.end(), {0x03, 0x00, 0xCA, 0xFE, 0xBE});

    CompletionCode completionCode = CompletionCode::error;
    std::vector<Record> records;
    ASSERT_TRUE(decodeResponseMessage(buf, 0x20, completionCode, records));
    EXPECT_EQ(completionCode, CompletionCode::success);
    ASSERT_EQ(records.size(), 3U);

    EXPECT_EQ(records[0].tag, 0);
    EXPECT_TRUE(records[0].valid);
    EXPECT_EQ(records[0].data.size(), 4U);

    EXPECT_EQ(records[1].tag, 3);
    EXPECT_FALSE(records[1].valid);
    EXPECT_EQ(records[1].data.size(), 2U);

    EXPECT_EQ(records[2].tag, 5);
    EXPECT_TRUE(records[2].valid);
    ASSERT_EQ(records[2].data.size(), 3U);
    const std::array<uint8_t, 3> tail{0xCA, 0xFE, 0xBE};
    EXPECT_TRUE(std::ranges::equal(records[2].data, tail));
}

TEST(BulkTelemetryMessage, RejectsTruncatedRecords)
{
    CompletionCode completionCode = CompletionCode::success;
    std::vector<Record> records;

    constexpr std::array<uint8_t, 8> shortData{0x20, 0x00, 0x01, 0x00,
                                               0x00, 0x05, 0x01, 0x02};
    EXPECT_FALSE(decodeResponseMessage(withResponseHeader(shortData), 0x20,
                                       completionCode, records));

    constexpr std::array<uint8_t, 6> missingRecord{0x20, 0x00, 0x02,
                                                   0x00, 0x00, 0x03};
    EXPECT_FALSE(decodeResponseMessage(withResponseHeader(missingRecord), 0x20,
                                       completionCode, records));
}

TEST(BulkTelemetryMessage, RejectsMismatchedCommandCode)
{
    constexpr std::array<uint8_t, 4> body{0x20, 0x00, 0x00, 0x00};
    CompletionCode completionCode = CompletionCode::success;
    std::vector<Record> records;
    EXPECT_FALSE(decodeResponseMessage(withResponseHeader(body), 0x13,
                                       completionCode, records));
}

TEST(BulkTelemetryMessage, DecodesEmptyResponse)
{
    constexpr std::array<uint8_t, 4> body{0x14, 0x04, 0x00, 0x00};
    CompletionCode completionCode = CompletionCode::success;
    EXPECT_TRUE(
        decodeEmptyResponse(withResponseHeader(body), 0x14, completionCode));
    EXPECT_EQ(completionCode, CompletionCode::errNotReady);
}

TEST(BulkTelemetryMessage, ValidatesRecord)
{
    constexpr std::array<uint8_t, 10> body{0x20, 0x00, 0x02, 0x00, 0x00,
                                           0x05, 0x01, 0x02, 0x03, 0x04};
    auto buf = withResponseHeader(body);
    buf.insert(buf.end(), {0x01, 0x02, 0x11, 0x22}); // flags without validBit

    CompletionCode completionCode = CompletionCode::error;
    std::vector<Record> records;
    ASSERT_TRUE(decodeResponseMessage(buf, 0x20, completionCode, records));
    ASSERT_EQ(records.size(), 2U);

    EXPECT_TRUE(isValidRecord(records[0], 0));
    EXPECT_FALSE(isValidRecord(records[0], 1));
    EXPECT_FALSE(isValidRecord(records[1], 1));
}

} // namespace
} // namespace ocp::bulk
