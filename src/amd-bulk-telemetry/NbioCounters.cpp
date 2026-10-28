/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NbioCounters.hpp"

#include "BulkTelemetryMessage.hpp"
#include "NetworkAdapter.hpp"

#include <phosphor-logging/lg2.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace ocp::bulk::nbio_counters
{

namespace
{

using Unit = MetricInfo::Unit;

constexpr size_t counterSize = sizeof(uint64_t);

constexpr uint32_t recoveries = 0x0005B018;
constexpr uint32_t rxErrors = 0x00057078;
constexpr uint32_t unsupportedRequests = 0x0005337E;
constexpr uint32_t naksReceived = 0x00057076;
constexpr uint32_t invalidCrc = 0x0005707B;
constexpr uint32_t replays = 0x0005907C;

struct CounterInfo
{
    std::string_view name;
    Unit unit;
    uint32_t uniqueId;
};

constexpr std::array<CounterInfo, 6> counterTable{{
    {"l0_to_recovery_count", Unit::Count, recoveries},
    {"rx_errors", Unit::Count, rxErrors},
    {"unsupported_request_count", Unit::Count, unsupportedRequests},
    {"nak_received_count", Unit::Count, naksReceived},
    {"dllp_crc_error_count", Unit::Count, invalidCrc},
    {"replay_count", Unit::Count, replays},
}};

} // namespace

std::vector<PortCounter> decode(std::span<const uint32_t> uniqueIds,
                                std::span<const uint8_t> data)
{
    const size_t expectedSize = uniqueIds.size() * counterSize;
    if (data.size() != expectedSize)
    {
        lg2::error("NBIO block is {SIZE} bytes, expected {EXPECTED}", "SIZE",
                   data.size(), "EXPECTED", expectedSize);
        return {};
    }

    auto read = [&](uint32_t uniqueId) -> std::optional<uint64_t> {
        for (size_t element = 0; element < uniqueIds.size(); element++)
        {
            if (uniqueIds[element] != uniqueId)
            {
                continue;
            }

            return readLe<uint64_t>(data.subspan(element * counterSize));
        }

        return std::nullopt;
    };

    std::vector<PortCounter> counters;
    counters.reserve(counterTable.size());

    for (const CounterInfo& counter : counterTable)
    {
        const std::optional<uint64_t> value = read(counter.uniqueId);
        if (value.has_value())
        {
            counters.emplace_back(counter.name, counter.unit, *value);
        }
    }

    return counters;
}

void update(NetworkAdapter& adapter, size_t instance,
            std::span<const uint32_t> uniqueIds, std::span<const uint8_t> data)
{
    const std::vector<PortCounter> counters = decode(uniqueIds, data);
    if (counters.empty())
    {
        return;
    }

    adapter.updatePcieCounters(instance, counters);
}

} // namespace ocp::bulk::nbio_counters
