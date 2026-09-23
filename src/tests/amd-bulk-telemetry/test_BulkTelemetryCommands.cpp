/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "BulkTelemetryCommands.hpp"
#include "OcpVdmIana.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

namespace ocp::bulk
{
namespace
{

constexpr uint8_t testInstanceId = 3;

constexpr std::array<uint8_t, messageHeaderSize> responseHeaderBytes{
    0x00, 0x00, 0xA6, 0x7F, 0x03, 0x89, 0x01};

void append(std::vector<uint8_t>& buf, std::span<const uint8_t> bytes)
{
    buf.insert(buf.end(), bytes.begin(), bytes.end());
}

// Compact record: the data size is a power of two encoded in the flags.
std::vector<uint8_t> compactRecord(uint8_t tag, std::span<const uint8_t> data)
{
    std::vector<uint8_t> record{
        tag, static_cast<uint8_t>((std::countr_zero(data.size()) << 1) | 1)};
    append(record, data);
    return record;
}

// ByteLength record: the data size follows the flags explicitly.
std::vector<uint8_t> byteLengthRecord(uint8_t tag,
                                      std::span<const uint8_t> data)
{
    std::vector<uint8_t> record{tag, 0x81, static_cast<uint8_t>(data.size()),
                                static_cast<uint8_t>(data.size() >> 8)};
    append(record, data);
    return record;
}

std::vector<uint8_t> makeResponse(Command command, CompletionCode code,
                                  uint16_t recordCount,
                                  std::span<const uint8_t> records)
{
    std::vector<uint8_t> buf(responseHeaderBytes.begin(),
                             responseHeaderBytes.end());
    buf.push_back(static_cast<uint8_t>(command));
    buf.push_back(static_cast<uint8_t>(code));
    buf.push_back(static_cast<uint8_t>(recordCount));
    buf.push_back(static_cast<uint8_t>(recordCount >> 8));
    append(buf, records);
    return buf;
}

std::vector<uint8_t> informationRecords(bool withDynamicTags)
{
    constexpr std::array<uint8_t, 1> categoryCount{2};
    constexpr std::array<uint8_t, 1> detailCount{7};
    constexpr std::array<uint8_t, 2> maxTransferLength{0x00, 0x10};
    constexpr std::array<uint8_t, 1> dynamicTags{1};

    std::vector<uint8_t> records;
    append(records, compactRecord(0, categoryCount));
    append(records, compactRecord(1, detailCount));
    append(records, compactRecord(2, maxTransferLength));
    if (withDynamicTags)
    {
        append(records, compactRecord(3, dynamicTags));
    }
    return records;
}

// Tags keep counting up across categories, so the first tag is a parameter.
std::vector<uint8_t> descriptionRecords(
    uint8_t firstTag, uint8_t categoryId, std::string_view name,
    uint8_t categoryLength, bool withDynamicTags)
{
    std::array<uint8_t, 64> nameBytes{};
    std::ranges::copy(name, nameBytes.begin());

    const std::array<uint8_t, 1> id{categoryId};
    const std::array<uint8_t, 1> elementCount{4};
    const std::array<uint8_t, 1> instanceCount{1};
    const std::array<uint8_t, 2> length{categoryLength, 0x00};
    constexpr std::array<uint8_t, 4> version{0x00, 0x00, 0x00, 0x01};
    constexpr std::array<uint8_t, 1> dynamicTags{1};

    std::vector<uint8_t> records;
    append(records, compactRecord(firstTag, id));
    append(records, compactRecord(firstTag + 1, elementCount));
    append(records, compactRecord(firstTag + 2, instanceCount));
    append(records, compactRecord(firstTag + 3, nameBytes));
    append(records, compactRecord(firstTag + 4, length));
    append(records, compactRecord(firstTag + 5, version));
    if (withDynamicTags)
    {
        append(records, compactRecord(firstTag + 6, dynamicTags));
    }
    return records;
}

TEST(BulkTelemetryCommands, EncodesVersionRequest)
{
    const auto request = encodeGetVersionRequest(testInstanceId);
    EXPECT_EQ(request, std::optional(std::vector<uint8_t>{
                           0x00, 0x00, 0xA6, 0x7F, 0x83, 0x89, 0x01, 0x01, 0x00,
                           0x00, 0x00}));
}

TEST(BulkTelemetryCommands, DecodesVersionResponse)
{
    constexpr std::array<uint8_t, 4> versionBytes{0x00, 0x00, 0x00, 0x01};
    const auto buf = makeResponse(Command::getVendorMessageTypeVersion,
                                  CompletionCode::success, 1,
                                  compactRecord(0, versionBytes));

    CompletionCode completionCode = CompletionCode::error;
    uint32_t version = 0;
    ASSERT_TRUE(decodeGetVersionResponse(buf, completionCode, version));
    EXPECT_EQ(completionCode, CompletionCode::success);
    EXPECT_EQ(version, 0x01000000U);
}

TEST(BulkTelemetryCommands, RejectsVersionResponseWithWrongLength)
{
    constexpr std::array<uint8_t, 2> versionBytes{0x00, 0x01};
    const auto buf = makeResponse(Command::getVendorMessageTypeVersion,
                                  CompletionCode::success, 1,
                                  compactRecord(0, versionBytes));

    CompletionCode completionCode = CompletionCode::error;
    uint32_t version = 0;
    EXPECT_FALSE(decodeGetVersionResponse(buf, completionCode, version));
}

TEST(BulkTelemetryCommands, ReportsUnsuccessfulCompletionCode)
{
    const auto buf = makeResponse(Command::getVendorMessageTypeVersion,
                                  CompletionCode::errNotReady, 0, {});

    CompletionCode completionCode = CompletionCode::success;
    uint32_t version = 0;
    EXPECT_TRUE(decodeGetVersionResponse(buf, completionCode, version));
    EXPECT_EQ(completionCode, CompletionCode::errNotReady);
    EXPECT_EQ(version, 0U);
}

TEST(BulkTelemetryCommands, DecodesCategoryInformation)
{
    const auto buf =
        makeResponse(Command::retrieveCategoryInformation,
                     CompletionCode::success, 4, informationRecords(true));

    CompletionCode completionCode = CompletionCode::error;
    CategoryInformation info{};
    ASSERT_TRUE(decodeCategoryInformationResponse(buf, completionCode, info));
    EXPECT_EQ(info.categoryCount, 2);
    EXPECT_EQ(info.categoryDetailCount, 7);
    EXPECT_EQ(info.maxTransferLength, 4096);
    EXPECT_TRUE(info.hasDynamicTags);
}

TEST(BulkTelemetryCommands, DecodesCategoryInformationWithoutDynamicTags)
{
    const auto buf =
        makeResponse(Command::retrieveCategoryInformation,
                     CompletionCode::success, 3, informationRecords(false));

    CompletionCode completionCode = CompletionCode::error;
    CategoryInformation info{};
    ASSERT_TRUE(decodeCategoryInformationResponse(buf, completionCode, info));
    EXPECT_EQ(info.maxTransferLength, 4096);
    EXPECT_FALSE(info.hasDynamicTags);
}

TEST(BulkTelemetryCommands, RejectsCategoryInformationWithMissingRecords)
{
    std::vector<uint8_t> records = informationRecords(false);
    records.resize(records.size() - 4); // Drop MAX_TRANSFER_LENGTH.
    const auto buf = makeResponse(Command::retrieveCategoryInformation,
                                  CompletionCode::success, 2, records);

    CompletionCode completionCode = CompletionCode::error;
    CategoryInformation info{};
    EXPECT_FALSE(decodeCategoryInformationResponse(buf, completionCode, info));
}

TEST(BulkTelemetryCommands, RejectsCategoryInformationWithUnexpectedTag)
{
    std::vector<uint8_t> records = informationRecords(false);
    records[6] = 9; // Tag of the MAX_TRANSFER_LENGTH record.
    const auto buf = makeResponse(Command::retrieveCategoryInformation,
                                  CompletionCode::success, 3, records);

    CompletionCode completionCode = CompletionCode::error;
    CategoryInformation info{};
    EXPECT_FALSE(decodeCategoryInformationResponse(buf, completionCode, info));
}

TEST(BulkTelemetryCommands, EncodesCategoryDescriptionRequest)
{
    const auto request = encodeCategoryDescriptionRequest(testInstanceId, 1);
    EXPECT_EQ(request, std::optional(std::vector<uint8_t>{
                           0x00, 0x00, 0xA6, 0x7F, 0x83, 0x89, 0x01, 0x11, 0x00,
                           0x01, 0x00, 0x01}));
}

TEST(BulkTelemetryCommands, DecodesCategoryDescriptions)
{
    std::vector<uint8_t> records = descriptionRecords(0, 1, "MAC", 8, true);
    append(records, descriptionRecords(7, 2, "PHY", 4, true));
    const auto buf = makeResponse(Command::retrieveCategoryDescription,
                                  CompletionCode::success, 14, records);

    CompletionCode completionCode = CompletionCode::error;
    std::vector<CategoryDescription> descriptions;
    ASSERT_TRUE(decodeCategoryDescriptionResponse(buf, 7, completionCode,
                                                  descriptions));
    ASSERT_EQ(descriptions.size(), 2U);
    EXPECT_EQ(descriptions[0].categoryId, 1);
    EXPECT_EQ(descriptions[0].elementCount, 4);
    EXPECT_EQ(descriptions[0].instanceCount, 1);
    EXPECT_EQ(descriptions[0].name, "MAC");
    EXPECT_EQ(descriptions[0].categoryLength, 8);
    EXPECT_EQ(descriptions[0].version, 0x01000000U);
    EXPECT_TRUE(descriptions[0].dynamicTags);
    EXPECT_EQ(descriptions[1].name, "PHY");
    EXPECT_EQ(descriptions[1].categoryLength, 4);
}

TEST(BulkTelemetryCommands, DecodesCategoryDescriptionWithoutDynamicTags)
{
    const std::vector<uint8_t> records =
        descriptionRecords(0, 1, "MAC", 8, false);
    const auto buf = makeResponse(Command::retrieveCategoryDescription,
                                  CompletionCode::success, 6, records);

    CompletionCode completionCode = CompletionCode::error;
    std::vector<CategoryDescription> descriptions;
    ASSERT_TRUE(decodeCategoryDescriptionResponse(buf, 6, completionCode,
                                                  descriptions));
    ASSERT_EQ(descriptions.size(), 1U);
    EXPECT_EQ(descriptions[0].name, "MAC");
    EXPECT_FALSE(descriptions[0].dynamicTags);
}

TEST(BulkTelemetryCommands, RejectsPartialCategoryDescription)
{
    const std::vector<uint8_t> records =
        descriptionRecords(0, 1, "MAC", 8, false);
    const auto buf = makeResponse(Command::retrieveCategoryDescription,
                                  CompletionCode::success, 6, records);

    CompletionCode completionCode = CompletionCode::error;
    std::vector<CategoryDescription> descriptions;
    EXPECT_FALSE(decodeCategoryDescriptionResponse(buf, 7, completionCode,
                                                   descriptions));
}

TEST(BulkTelemetryCommands, DecodesCategoryTags)
{
    constexpr std::array<uint8_t, 8> tagBytes{0x00, 0x00, 0x01, 0x00,
                                              0x02, 0x00, 0x01, 0x00};
    const auto buf =
        makeResponse(Command::retrieveCategoryTags, CompletionCode::success, 1,
                     byteLengthRecord(0, tagBytes));

    CompletionCode completionCode = CompletionCode::error;
    std::vector<uint32_t> uniqueIds;
    ASSERT_TRUE(decodeCategoryTagsResponse(buf, completionCode, uniqueIds));
    EXPECT_EQ(uniqueIds, (std::vector<uint32_t>{0x00010000, 0x00010002}));
}

TEST(BulkTelemetryCommands, RejectsCategoryTagsWithPartialUniqueId)
{
    constexpr std::array<uint8_t, 6> tagBytes{0x00, 0x00, 0x01,
                                              0x00, 0x02, 0x00};
    const auto buf =
        makeResponse(Command::retrieveCategoryTags, CompletionCode::success, 1,
                     byteLengthRecord(0, tagBytes));

    CompletionCode completionCode = CompletionCode::error;
    std::vector<uint32_t> uniqueIds;
    EXPECT_FALSE(decodeCategoryTagsResponse(buf, completionCode, uniqueIds));
}

TEST(BulkTelemetryCommands, EncodesTelemetryDataRequest)
{
    const auto request =
        encodeTelemetryDataRequest(testInstanceId, 1, false, 0, 0, 2);
    EXPECT_EQ(request, std::optional(std::vector<uint8_t>{
                           0x00, 0x00, 0xA6, 0x7F, 0x83, 0x89, 0x01, 0x20, 0x00,
                           0x06, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00}));
}

TEST(BulkTelemetryCommands, DecodesTelemetryData)
{
    constexpr std::array<uint8_t, 8> dataBytes{0x23, 0x10, 0x00, 0x00,
                                               0x00, 0x00, 0x00, 0x00};
    const auto buf =
        makeResponse(Command::retrieveTelemetryData, CompletionCode::success, 1,
                     byteLengthRecord(0, dataBytes));

    CompletionCode completionCode = CompletionCode::error;
    std::vector<uint8_t> telemetryData;
    ASSERT_TRUE(
        decodeTelemetryDataResponse(buf, completionCode, telemetryData));
    EXPECT_EQ(telemetryData,
              std::vector<uint8_t>(dataBytes.begin(), dataBytes.end()));
}

TEST(BulkTelemetryCommands, DecodesOpenAndCloseResponses)
{
    CompletionCode completionCode = CompletionCode::error;
    EXPECT_TRUE(decodeOpenTelemetryDataResponse(
        makeResponse(Command::openTelemetryData, CompletionCode::success, 0,
                     {}),
        completionCode));
    EXPECT_EQ(completionCode, CompletionCode::success);

    EXPECT_TRUE(decodeCloseTelemetryDataResponse(
        makeResponse(Command::closeTelemetryData, CompletionCode::errBusy, 0,
                     {}),
        completionCode));
    EXPECT_EQ(completionCode, CompletionCode::errBusy);
}

TEST(BulkTelemetryCommands, RejectsMismatchedCommandCode)
{
    const auto buf =
        makeResponse(Command::retrieveCategoryInformation,
                     CompletionCode::success, 4, informationRecords(true));

    CompletionCode completionCode = CompletionCode::error;
    uint32_t version = 0;
    EXPECT_FALSE(decodeGetVersionResponse(buf, completionCode, version));
}

} // namespace
} // namespace ocp::bulk
