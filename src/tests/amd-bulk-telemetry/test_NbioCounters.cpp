/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NbioCounters.hpp"
#include "NetworkAdapter.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

namespace ocp::bulk::nbio_counters
{
namespace
{

constexpr size_t counterSize = 8;

constexpr std::array<uint32_t, 28> firmwareUniqueIds{
    0x0005B003, 0x0005B018, 0x0005B00D, 0x0005B01C, 0x00057078, 0x00057077,
    0x00057076, 0x0005707B, 0x0005007B, 0x00050075, 0x0005307A, 0x0005337E,
    0x0005907C, 0x00059074, 0x00059073, 0x0005906A, 0x00059069, 0x00059078,
    0x00059077, 0x000590FF, 0x000510FE, 0x000510F3, 0x000510E2, 0x000510F8,
    0x000520CE, 0x000520CA, 0x000520B0, 0x000520C0};

constexpr size_t recoveriesElement = 1;
constexpr size_t naksReceivedElement = 6;
constexpr size_t crcErrorsElement = 7;
constexpr size_t unsupportedRequestsElement = 11;
constexpr size_t replaysElement = 12;
constexpr size_t rxErrorsElement = 4;

// Each counter's value comes from its element index, so reading the wrong
// offset gives an obviously wrong number.
constexpr uint64_t valueFor(size_t element)
{
    return 100 + element;
}

std::vector<uint32_t> uniqueIds()
{
    return {firmwareUniqueIds.begin(), firmwareUniqueIds.end()};
}

std::vector<uint8_t> makeCounterPayload()
{
    std::vector<uint8_t> data(firmwareUniqueIds.size() * counterSize, 0);
    for (size_t element = 0; element < firmwareUniqueIds.size(); element++)
    {
        uint64_t value = valueFor(element);
        for (size_t byte = 0; byte < counterSize; byte++)
        {
            data[(element * counterSize) + byte] =
                static_cast<uint8_t>(value >> (byte * 8));
        }
    }
    return data;
}

const PortCounter* findCounter(const std::vector<PortCounter>& counters,
                               std::string_view name)
{
    for (const PortCounter& counter : counters)
    {
        if (counter.name == name)
        {
            return &counter;
        }
    }
    return nullptr;
}

TEST(NbioCounters, DecodesTheCountersMetricValueDefines)
{
    const std::vector<PortCounter> counters =
        decode(uniqueIds(), makeCounterPayload());

    ASSERT_EQ(counters.size(), 6U);

    const PortCounter* recoveries =
        findCounter(counters, "l0_to_recovery_count");
    ASSERT_NE(recoveries, nullptr);
    EXPECT_EQ(recoveries->value, valueFor(recoveriesElement));
    EXPECT_EQ(recoveries->unit, MetricInfo::Unit::Count);

    const PortCounter* rxErrors = findCounter(counters, "rx_errors");
    ASSERT_NE(rxErrors, nullptr);
    EXPECT_EQ(rxErrors->value, valueFor(rxErrorsElement));
    EXPECT_EQ(rxErrors->unit, MetricInfo::Unit::Count);

    const PortCounter* unsupported =
        findCounter(counters, "unsupported_request_count");
    ASSERT_NE(unsupported, nullptr);
    EXPECT_EQ(unsupported->value, valueFor(unsupportedRequestsElement));
    EXPECT_EQ(unsupported->unit, MetricInfo::Unit::Count);

    const PortCounter* naks = findCounter(counters, "nak_received_count");
    ASSERT_NE(naks, nullptr);
    EXPECT_EQ(naks->value, valueFor(naksReceivedElement));

    const PortCounter* crcErrors =
        findCounter(counters, "dllp_crc_error_count");
    ASSERT_NE(crcErrors, nullptr);
    EXPECT_EQ(crcErrors->value, valueFor(crcErrorsElement));

    const PortCounter* replays = findCounter(counters, "replay_count");
    ASSERT_NE(replays, nullptr);
    EXPECT_EQ(replays->value, valueFor(replaysElement));
}

TEST(NbioCounters, RejectsWrongSize)
{
    std::vector<uint8_t> data = makeCounterPayload();
    data.pop_back();

    EXPECT_TRUE(decode(uniqueIds(), data).empty());
}

} // namespace
} // namespace ocp::bulk::nbio_counters
