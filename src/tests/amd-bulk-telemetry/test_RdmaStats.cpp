/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NetworkAdapter.hpp"
#include "RdmaStats.hpp"

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

namespace ocp::bulk::rdma_stats
{
namespace
{

constexpr size_t elementCount = 61;
constexpr size_t counterSize = 8;

constexpr size_t txReadRequestsElement = 0x08;
constexpr size_t txSendRequestsElement = 0x09;
constexpr size_t txWriteRequestsElement = 0x0A;

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

TEST(RdmaStats, DecodesTheCountersMetricValue)
{
    const std::vector<PortCounter> counters = decode(makeCounterPayload());

    ASSERT_EQ(counters.size(), 3U);

    const PortCounter* readRequests =
        findCounter(counters, "rdma_tx_read_requests");
    ASSERT_NE(readRequests, nullptr);
    EXPECT_EQ(readRequests->value, valueFor(txReadRequestsElement));
    EXPECT_EQ(readRequests->unit, MetricInfo::Unit::Count);

    const PortCounter* sendRequests =
        findCounter(counters, "rdma_tx_send_requests");
    ASSERT_NE(sendRequests, nullptr);
    EXPECT_EQ(sendRequests->value, valueFor(txSendRequestsElement));
    EXPECT_EQ(sendRequests->unit, MetricInfo::Unit::Count);

    const PortCounter* writeRequests =
        findCounter(counters, "rdma_tx_write_requests");
    ASSERT_NE(writeRequests, nullptr);
    EXPECT_EQ(writeRequests->value, valueFor(txWriteRequestsElement));
    EXPECT_EQ(writeRequests->unit, MetricInfo::Unit::Count);
}

TEST(RdmaStats, RejectsWrongSize)
{
    std::vector<uint8_t> data = makeCounterPayload();
    data.pop_back();

    EXPECT_TRUE(decode(data).empty());
}

} // namespace
} // namespace ocp::bulk::rdma_stats
