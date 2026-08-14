/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "LoggingStub.hpp"
#include "MctpMockTestBase.hpp"
#include "NvidiaEventReporting.hpp"
#include "NvidiaGpuXid.hpp"

#include <chrono>
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

class NvidiaGpuXidTest : public MctpMockTestBase
{
  protected:
    void SetUp() override
    {
        MctpMockTestBase::SetUp();
        if (testing::Test::IsSkipped())
        {
            return;
        }
        // The stub stands in for phosphor-logging on the private test bus so
        // the Create() call the handler makes can be inspected. It is shared
        // with every other fixture in this binary that needs one.
        ASSERT_TRUE(logging_stub::ensure(bus(), objects()));
        logging_stub::setActiveCall(&createCall);
    }

    void TearDown() override
    {
        logging_stub::setActiveCall(nullptr);
        MctpMockTestBase::TearDown();
    }

    static std::shared_ptr<NvidiaXidEventHandler> createHandler(
        const std::string& name = "GPU_XID")
    {
        return std::make_shared<NvidiaXidEventHandler>(name, bus());
    }

    // Arguments of the Logging.Create calls made while this test runs.
    logging_stub::CreateCall createCall;
};

TEST_F(NvidiaGpuXidTest, HandleXidEventGoodDataNoCrash)
{
    auto xh = createHandler("xid_good");
    const auto data = buildXidEventData(
        0x01, 79, 1234, 1'700'000'000'000'000'000ULL, "XID error message");
    EXPECT_NO_THROW(xh->handleXidEvent(EventInfo{}, data));
    EXPECT_NO_THROW(drainPendingAsync());
}

TEST_F(NvidiaGpuXidTest, HandleXidEventEmptyDataNoCrash)
{
    auto xh = createHandler("xid_empty");
    const std::vector<uint8_t> data;
    EXPECT_NO_THROW(xh->handleXidEvent(EventInfo{}, data));
}

TEST_F(NvidiaGpuXidTest, HandleXidEventTinyDataNoCrash)
{
    auto xh = createHandler("xid_tiny");
    const std::vector<uint8_t> data(10, 0); // fewer than 20 bytes
    EXPECT_NO_THROW(xh->handleXidEvent(EventInfo{}, data));
}

TEST_F(NvidiaGpuXidTest, HandleXidEventExactly20BytesNoText)
{
    auto xh = createHandler("xid_exact");
    const auto data = buildXidEventData(0x00, 0, 0, 0, "");
    ASSERT_EQ(data.size(), 20U);
    EXPECT_NO_THROW(xh->handleXidEvent(EventInfo{}, data));
    EXPECT_NO_THROW(drainPendingAsync());
}

TEST_F(NvidiaGpuXidTest, HandleXidEventLogsFormattedMessage)
{
    auto xh = createHandler("xid_msg");
    const auto data = buildXidEventData(
        0x2A, 79, 1234, 1'700'000'000'000'000'000ULL, "XID error message");
    xh->handleXidEvent(EventInfo{}, data);

    // An earlier test's Create() can still be in flight, so wait for the one
    // naming this device rather than for just any logged message.
    ASSERT_TRUE(pumpIoUntil(
        [this] {
            return createCall.message.find("xid_msg") != std::string::npos;
        },
        std::chrono::seconds(5)));
    EXPECT_EQ(
        createCall.message,
        "The resource property xid_msg Driver Event Message has detected "
        "errors of type [Tue Nov 14 22:13:20.000000000 UTC 2023][1234][2a] "
        "XID 79 XID error message");
}

} // namespace
