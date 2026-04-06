/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"
#include "NvidiaEventReporting.hpp"
#include "NvidiaGpuXid.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace
{

void appendLE(std::vector<uint8_t>& buf, uint64_t value, size_t bytes)
{
    for (size_t i = 0; i < bytes; ++i)
    {
        buf.push_back(static_cast<uint8_t>((value >> (8 * i)) & 0xFF));
    }
}

// Build XID event data:
//   flags(1) + reserved(3) + reason(4 LE) + sequence(4 LE) + timestamp(8 LE)
//   + messageText
std::vector<uint8_t> buildXidEventData(uint8_t flags, uint32_t reason,
                                       uint32_t sequence, uint64_t timestamp,
                                       const std::string& text)
{
    std::vector<uint8_t> buf;
    buf.push_back(flags);
    appendLE(buf, 0, 3); // reserved
    appendLE(buf, reason, 4);
    appendLE(buf, sequence, 4);
    appendLE(buf, timestamp, 8);
    buf.insert(buf.end(), text.begin(), text.end());
    return buf;
}

class NvidiaGpuXidTestBase : public DbusMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaXidEventHandler> createHandler(
        const std::string& name = "GPU_XID")
    {
        return std::make_shared<NvidiaXidEventHandler>(name, conn);
    }
};

TEST_F(NvidiaGpuXidTestBase, HandleXidEventGoodDataNoCrash)
{
    auto xh = createHandler("xid_good");
    const auto data = buildXidEventData(
        0x01, 79, 1234, 1'700'000'000'000'000'000ULL, "XID error message");
    EXPECT_NO_THROW(xh->handleXidEvent(EventInfo{}, data));
    EXPECT_NO_THROW(drainPendingAsync());
}

TEST_F(NvidiaGpuXidTestBase, HandleXidEventEmptyDataNoCrash)
{
    auto xh = createHandler("xid_empty");
    const std::vector<uint8_t> data;
    EXPECT_NO_THROW(xh->handleXidEvent(EventInfo{}, data));
}

TEST_F(NvidiaGpuXidTestBase, HandleXidEventTinyDataNoCrash)
{
    auto xh = createHandler("xid_tiny");
    const std::vector<uint8_t> data(10, 0); // fewer than 20 bytes
    EXPECT_NO_THROW(xh->handleXidEvent(EventInfo{}, data));
}

TEST_F(NvidiaGpuXidTestBase, HandleXidEventExactly20BytesNoText)
{
    auto xh = createHandler("xid_exact");
    const auto data = buildXidEventData(0x00, 0, 0, 0, "");
    ASSERT_EQ(data.size(), 20U);
    EXPECT_NO_THROW(xh->handleXidEvent(EventInfo{}, data));
    EXPECT_NO_THROW(drainPendingAsync());
}

} // namespace
