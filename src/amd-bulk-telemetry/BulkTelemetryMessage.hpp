/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "OcpVdmIana.hpp"

#include <bit>
#include <concepts>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <vector>

namespace ocp::bulk
{

// One decoded record of a Bulk Telemetry response.
struct Record
{
    uint8_t tag;
    bool valid;
    std::span<const uint8_t> data;
};

// Reads a little-endian field.
template <std::integral T>
T readLe(std::span<const uint8_t> data)
{
    T value{};
    std::memcpy(&value, data.data(), sizeof(T));

    if constexpr (sizeof(T) > 1 && std::endian::native != std::endian::little)
    {
        value = std::byteswap(value);
    }

    return value;
}

std::optional<std::vector<uint8_t>> encodeRequestMessage(
    uint8_t instanceId, uint8_t commandCode, std::span<const uint8_t> payload);

bool decodeResponseMessage(std::span<const uint8_t> buf, uint8_t commandCode,
                           CompletionCode& completionCode,
                           std::vector<Record>& records);

bool decodeEmptyResponse(std::span<const uint8_t> buf, uint8_t commandCode,
                         CompletionCode& completionCode);

bool decodeRecords(std::span<const uint8_t> buf, uint16_t count,
                   std::vector<Record>& records);

bool isValidRecord(const Record& record, uint8_t tag);

} // namespace ocp::bulk
