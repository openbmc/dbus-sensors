/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "BulkTelemetryCommands.hpp"

#include "BulkTelemetryMessage.hpp"
#include "OcpVdmIana.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace ocp::bulk
{

namespace
{

enum InformationItem
{
    categoryCountItem = 0,
    categoryDetailCountItem,
    maxTransferLengthItem,
    hasDynamicTagsItem,
    informationItemCount,
};

constexpr std::array<size_t, informationItemCount> informationItemSizes{
    1, // categoryCountItem
    1, // categoryDetailCountItem
    2, // maxTransferLengthItem
    1, // hasDynamicTagsItem
};
constexpr size_t minInformationItems = hasDynamicTagsItem;

enum DescriptionItem
{
    categoryIdItem = 0,
    elementCountItem,
    instanceCountItem,
    nameItem,
    categoryLengthItem,
    versionItem,
    dynamicTagsItem,
    descriptionItemCount,
};

constexpr std::array<size_t, descriptionItemCount> descriptionItemSizes{
    0,  // categoryIdItem is checked separately
    1,  // elementCountItem
    1,  // instanceCountItem
    64, // nameItem
    2,  // categoryLengthItem
    4,  // versionItem
    1,  // dynamicTagsItem
};
constexpr size_t minDescriptionItems = dynamicTagsItem;

// Item IDs keep counting up across categories, so only sizes can be checked.
bool isValidDescription(std::span<const Record> records, size_t offset,
                        size_t detailCount)
{
    const Record& categoryIdRecord = records[offset + categoryIdItem];
    if (!categoryIdRecord.valid || (categoryIdRecord.data.size() != 1 &&
                                    categoryIdRecord.data.size() != 2))
    {
        return false;
    }

    for (size_t i = categoryIdItem + 1;
         i < std::min<size_t>(detailCount, descriptionItemCount); ++i)
    {
        const Record& record = records[offset + i];
        if (!record.valid || record.data.size() != descriptionItemSizes[i])
        {
            return false;
        }
    }
    return true;
}

// A parameter is a record tagged by its position, just like a response item.
struct Parameter
{
    uint8_t byteCount;
    uint16_t value;
};

std::vector<uint8_t> encodeParameters(std::span<const Parameter> parameters)
{
    std::vector<uint8_t> payload;
    for (size_t i = 0; i < parameters.size(); ++i)
    {
        const std::array<uint8_t, sizeof(uint16_t)> data{
            static_cast<uint8_t>(parameters[i].value),
            static_cast<uint8_t>(parameters[i].value >> 8)};
        encodeRecord(static_cast<uint8_t>(i),
                     std::span(data).first(parameters[i].byteCount), payload);
    }
    return payload;
}

} // namespace

std::optional<std::vector<uint8_t>> encodeGetVersionRequest()
{
    return encodeRequestMessage(
        static_cast<uint8_t>(Command::getVendorMessageTypeVersion), {});
}

bool decodeGetVersionResponse(std::span<const uint8_t> buf,
                              CompletionCode& completionCode, uint32_t& version)
{
    std::vector<Record> records;
    if (!decodeResponseMessage(
            buf, static_cast<uint8_t>(Command::getVendorMessageTypeVersion),
            completionCode, records))
    {
        return false;
    }

    if (completionCode != CompletionCode::success)
    {
        return true;
    }

    if (records.size() != 1 || !isValidRecord(records[0], 0) ||
        records[0].data.size() != sizeof(uint32_t))
    {
        return false;
    }

    version = readLe<uint32_t>(records[0].data);
    return true;
}

std::optional<std::vector<uint8_t>> encodeCategoryInformationRequest()
{
    return encodeRequestMessage(
        static_cast<uint8_t>(Command::retrieveCategoryInformation), {});
}

bool decodeCategoryInformationResponse(std::span<const uint8_t> buf,
                                       CompletionCode& completionCode,
                                       CategoryInformation& info)
{
    std::vector<Record> records;
    if (!decodeResponseMessage(
            buf, static_cast<uint8_t>(Command::retrieveCategoryInformation),
            completionCode, records))
    {
        return false;
    }

    if (completionCode != CompletionCode::success)
    {
        return true;
    }

    // HAS_DYNAMIC_TAGS is an optional extension, so a three-record response
    // is still well formed.
    if (records.size() < minInformationItems ||
        records.size() > informationItemCount)
    {
        return false;
    }

    for (size_t i = 0; i < records.size(); ++i)
    {
        if (!isValidRecord(records[i], static_cast<uint8_t>(i)) ||
            records[i].data.size() != informationItemSizes[i])
        {
            return false;
        }
    }

    info.categoryCount = records[categoryCountItem].data[0];
    info.categoryDetailCount = records[categoryDetailCountItem].data[0];
    info.maxTransferLength =
        readLe<uint16_t>(records[maxTransferLengthItem].data);
    info.hasDynamicTags = records.size() > hasDynamicTagsItem &&
                          records[hasDynamicTagsItem].data[0] != 0;
    return true;
}

std::optional<std::vector<uint8_t>> encodeCategoryDescriptionRequest(
    uint8_t categoryIndex)
{
    const std::array<Parameter, 1> parameters{{{1, categoryIndex}}};
    return encodeRequestMessage(
        static_cast<uint8_t>(Command::retrieveCategoryDescription),
        encodeParameters(parameters));
}

bool decodeCategoryDescriptionResponse(
    std::span<const uint8_t> buf, uint8_t detailCount,
    CompletionCode& completionCode,
    std::vector<CategoryDescription>& descriptions)
{
    std::vector<Record> records;
    if (!decodeResponseMessage(
            buf, static_cast<uint8_t>(Command::retrieveCategoryDescription),
            completionCode, records))
    {
        return false;
    }

    if (completionCode != CompletionCode::success)
    {
        return true;
    }

    if (detailCount < minDescriptionItems || records.empty() ||
        records.size() % detailCount != 0)
    {
        return false;
    }

    descriptions.clear();
    for (size_t offset = 0; offset < records.size(); offset += detailCount)
    {
        if (!isValidDescription(records, offset, detailCount))
        {
            return false;
        }

        const std::span<const uint8_t> nameData =
            records[offset + nameItem].data;
        const auto nameEnd =
            std::find(nameData.begin(), nameData.end(), uint8_t{0});

        const uint16_t categoryLength =
            readLe<uint16_t>(records[offset + categoryLengthItem].data);
        const uint32_t version =
            readLe<uint32_t>(records[offset + versionItem].data);

        const std::span<const uint8_t> categoryIdData =
            records[offset + categoryIdItem].data;
        const auto categoryId = static_cast<uint16_t>(
            categoryIdData.size() == 1 ? categoryIdData[0]
                                       : readLe<uint16_t>(categoryIdData));

        descriptions.push_back(CategoryDescription{
            .categoryId = categoryId,
            .elementCount = records[offset + elementCountItem].data[0],
            .instanceCount = records[offset + instanceCountItem].data[0],
            .name = std::string(nameData.begin(), nameEnd),
            .categoryLength = categoryLength,
            .version = version,
            .dynamicTags = detailCount > dynamicTagsItem &&
                           records[offset + dynamicTagsItem].data[0] != 0,
        });
    }

    return true;
}

std::optional<std::vector<uint8_t>> encodeCategoryTagsRequest(
    uint8_t categoryIndex)
{
    const std::array<Parameter, 1> parameters{{{1, categoryIndex}}};
    return encodeRequestMessage(
        static_cast<uint8_t>(Command::retrieveCategoryTags),
        encodeParameters(parameters));
}

bool decodeCategoryTagsResponse(std::span<const uint8_t> buf,
                                CompletionCode& completionCode,
                                std::vector<uint32_t>& uniqueIds)
{
    std::vector<Record> records;
    if (!decodeResponseMessage(
            buf, static_cast<uint8_t>(Command::retrieveCategoryTags),
            completionCode, records))
    {
        return false;
    }

    if (completionCode != CompletionCode::success)
    {
        return true;
    }

    if (records.size() != 1 || !isValidRecord(records[0], 0) ||
        records[0].data.size() % sizeof(uint32_t) != 0)
    {
        return false;
    }

    uniqueIds.clear();
    uniqueIds.resize(records[0].data.size() / sizeof(uint32_t));

    std::span<const uint8_t> data = records[0].data;
    for (uint32_t& uniqueId : uniqueIds)
    {
        uniqueId = readLe<uint32_t>(data);
        data = data.subspan(sizeof(uint32_t));
    }

    return true;
}

std::optional<std::vector<uint8_t>> encodeOpenTelemetryDataRequest()
{
    // CATEGORY_MASK is optional and omitting it locks every category.
    return encodeRequestMessage(
        static_cast<uint8_t>(Command::openTelemetryData), {});
}

bool decodeOpenTelemetryDataResponse(std::span<const uint8_t> buf,
                                     CompletionCode& completionCode)
{
    return decodeEmptyResponse(
        buf, static_cast<uint8_t>(Command::openTelemetryData), completionCode);
}

std::optional<std::vector<uint8_t>> encodeCloseTelemetryDataRequest()
{
    return encodeRequestMessage(
        static_cast<uint8_t>(Command::closeTelemetryData), {});
}

bool decodeCloseTelemetryDataResponse(std::span<const uint8_t> buf,
                                      CompletionCode& completionCode)
{
    return decodeEmptyResponse(
        buf, static_cast<uint8_t>(Command::closeTelemetryData), completionCode);
}

std::optional<std::vector<uint8_t>> encodeTelemetryDataRequest(
    uint8_t categoryIndex, bool byElement, uint8_t instanceIndex,
    uint8_t elementIndex, uint16_t count)
{
    const std::array<Parameter, 5> parameters{{
        {1, categoryIndex},
        {1, static_cast<uint16_t>(byElement ? 1 : 0)},
        {1, instanceIndex},
        {1, elementIndex},
        {2, count},
    }};

    return encodeRequestMessage(
        static_cast<uint8_t>(Command::retrieveTelemetryData),
        encodeParameters(parameters));
}

bool decodeTelemetryDataResponse(std::span<const uint8_t> buf,
                                 CompletionCode& completionCode,
                                 std::vector<uint8_t>& telemetryData)
{
    std::vector<Record> records;
    if (!decodeResponseMessage(
            buf, static_cast<uint8_t>(Command::retrieveTelemetryData),
            completionCode, records))
    {
        return false;
    }

    if (completionCode != CompletionCode::success)
    {
        return true;
    }

    if (records.size() != 1 || !isValidRecord(records[0], 0))
    {
        return false;
    }

    telemetryData.assign(records[0].data.begin(), records[0].data.end());
    return true;
}

} // namespace ocp::bulk
