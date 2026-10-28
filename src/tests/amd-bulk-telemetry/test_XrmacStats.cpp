/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NetworkAdapter.hpp"
#include "XrmacStats.hpp"

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

namespace ocp::bulk::xrmac_stats
{
namespace
{

constexpr size_t elementCount = 161;
constexpr size_t counterSize = 8;

constexpr size_t txFramesElement = 0x00;
constexpr size_t txGoodFramesElement = 0x02;
constexpr size_t txBytesElement = 0x07;
constexpr size_t rxFramesElement = 0x15;
constexpr size_t rxGoodFramesElement = 0x17;
constexpr size_t rxBadFramesElement = 0x18;
constexpr size_t rxBytesElement = 0x1D;
constexpr size_t rxOverflowElement = 0x2B;
constexpr size_t rxMatchFaultElement = 0x36;
constexpr uint64_t txGoodFramesValue = 90;
constexpr uint64_t rxGoodFramesValue = 95;

// Each counter's value comes from its element index, so reading the wrong
// offset gives an obviously wrong number.
constexpr uint64_t valueFor(size_t element)
{
    return 100 + element;
}

std::vector<uint8_t> makeCounterPayload()
{
    std::vector<uint8_t> data(elementCount * counterSize, 0);
    for (size_t element = 0; element < elementCount; element++)
    {
        uint64_t value = valueFor(element);
        if (element == txGoodFramesElement)
        {
            value = txGoodFramesValue;
        }
        else if (element == rxGoodFramesElement)
        {
            value = rxGoodFramesValue;
        }
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

TEST(XrmacStats, DecodesTheCountersMetricValue)
{
    const std::vector<PortCounter> counters = decode(makeCounterPayload());

    ASSERT_EQ(counters.size(), 18U);

    const PortCounter* txFrames = findCounter(counters, "tx_frames");
    ASSERT_NE(txFrames, nullptr);
    EXPECT_EQ(txFrames->value, valueFor(txFramesElement));
    EXPECT_EQ(txFrames->unit, MetricInfo::Unit::Count);

    const PortCounter* txErrors = findCounter(counters, "tx_errors");
    ASSERT_NE(txErrors, nullptr);
    EXPECT_EQ(txErrors->value, valueFor(txFramesElement) - txGoodFramesValue);
    EXPECT_EQ(txErrors->unit, MetricInfo::Unit::Count);

    const PortCounter* rxFrames = findCounter(counters, "rx_frames");
    ASSERT_NE(rxFrames, nullptr);
    EXPECT_EQ(rxFrames->value, valueFor(rxFramesElement));
    EXPECT_EQ(rxFrames->unit, MetricInfo::Unit::Count);

    const PortCounter* rxErrors = findCounter(counters, "rx_errors");
    ASSERT_NE(rxErrors, nullptr);
    EXPECT_EQ(rxErrors->value, valueFor(rxFramesElement) - rxGoodFramesValue);
    EXPECT_EQ(rxErrors->unit, MetricInfo::Unit::Count);

    const PortCounter* txBytes = findCounter(counters, "tx_bytes");
    ASSERT_NE(txBytes, nullptr);
    EXPECT_EQ(txBytes->value, valueFor(txBytesElement));
    EXPECT_EQ(txBytes->unit, MetricInfo::Unit::Bytes);

    const PortCounter* rxBytes = findCounter(counters, "rx_bytes");
    ASSERT_NE(rxBytes, nullptr);
    EXPECT_EQ(rxBytes->value, valueFor(rxBytesElement));
    EXPECT_EQ(rxBytes->unit, MetricInfo::Unit::Bytes);

    const PortCounter* rxOversize = findCounter(counters, "rx_oversize_frames");
    ASSERT_NE(rxOversize, nullptr);
    EXPECT_EQ(rxOversize->value, valueFor(rxOverflowElement));
    EXPECT_EQ(rxOversize->unit, MetricInfo::Unit::Count);

    const PortCounter* rxDiscards = findCounter(counters, "rx_discards");
    ASSERT_NE(rxDiscards, nullptr);
    EXPECT_EQ(rxDiscards->value, valueFor(rxBadFramesElement));
    EXPECT_EQ(rxDiscards->unit, MetricInfo::Unit::Count);

    const PortCounter* rxFalseCarrier =
        findCounter(counters, "rx_false_carrier_errors");
    ASSERT_NE(rxFalseCarrier, nullptr);
    EXPECT_EQ(rxFalseCarrier->value, valueFor(rxMatchFaultElement));
    EXPECT_EQ(rxFalseCarrier->unit, MetricInfo::Unit::Count);
}

TEST(XrmacStats, ReportsNoErrorsWhenGoodFramesExceedTotal)
{
    std::vector<uint8_t> data = makeCounterPayload();
    data[txGoodFramesElement * counterSize] =
        static_cast<uint8_t>(valueFor(txFramesElement) + 1);

    const std::vector<PortCounter> counters = decode(data);
    const PortCounter* txErrors = findCounter(counters, "tx_errors");
    ASSERT_NE(txErrors, nullptr);
    EXPECT_EQ(txErrors->value, 0U);
}

TEST(XrmacStats, RejectsWrongSize)
{
    std::vector<uint8_t> data = makeCounterPayload();
    data.pop_back();

    EXPECT_TRUE(decode(data).empty());
}

} // namespace
} // namespace ocp::bulk::xrmac_stats
