/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "OcpVdmIana.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace ocp::bulk
{

enum class Command : uint8_t
{
    getVendorMessageTypeVersion = 0x01,
    retrieveCategoryInformation = 0x10,
    retrieveCategoryDescription = 0x11,
    retrieveCategoryTags = 0x12,
    openTelemetryData = 0x13,
    closeTelemetryData = 0x14,
    retrieveTelemetryData = 0x20,
};

struct CategoryInformation
{
    uint8_t categoryCount;
    uint8_t categoryDetailCount;
    uint16_t maxTransferLength;
    bool hasDynamicTags;
};

struct CategoryDescription
{
    uint8_t categoryId;
    uint8_t elementCount;
    uint8_t instanceCount;
    std::string name;
    uint16_t categoryLength;
    uint32_t version;
    bool dynamicTags;
};

std::optional<std::vector<uint8_t>> encodeGetVersionRequest(uint8_t instanceId);
bool decodeGetVersionResponse(std::span<const uint8_t> buf,
                              CompletionCode& completionCode,
                              uint32_t& version);

std::optional<std::vector<uint8_t>> encodeCategoryInformationRequest(
    uint8_t instanceId);
bool decodeCategoryInformationResponse(std::span<const uint8_t> buf,
                                       CompletionCode& completionCode,
                                       CategoryInformation& info);

std::optional<std::vector<uint8_t>> encodeCategoryDescriptionRequest(
    uint8_t instanceId, uint8_t categoryIndex);
bool decodeCategoryDescriptionResponse(
    std::span<const uint8_t> buf, uint8_t detailCount,
    CompletionCode& completionCode,
    std::vector<CategoryDescription>& descriptions);

std::optional<std::vector<uint8_t>> encodeCategoryTagsRequest(
    uint8_t instanceId, uint8_t categoryIndex);
bool decodeCategoryTagsResponse(std::span<const uint8_t> buf,
                                CompletionCode& completionCode,
                                std::vector<uint32_t>& uniqueIds);

std::optional<std::vector<uint8_t>> encodeOpenTelemetryDataRequest(
    uint8_t instanceId);
bool decodeOpenTelemetryDataResponse(std::span<const uint8_t> buf,
                                     CompletionCode& completionCode);

std::optional<std::vector<uint8_t>> encodeCloseTelemetryDataRequest(
    uint8_t instanceId);
bool decodeCloseTelemetryDataResponse(std::span<const uint8_t> buf,
                                      CompletionCode& completionCode);

std::optional<std::vector<uint8_t>> encodeTelemetryDataRequest(
    uint8_t instanceId, uint8_t categoryIndex, bool byElement,
    uint8_t instanceIndex, uint8_t elementIndex, uint16_t count);
bool decodeTelemetryDataResponse(std::span<const uint8_t> buf,
                                 CompletionCode& completionCode,
                                 std::vector<uint8_t>& telemetryData);

} // namespace ocp::bulk
