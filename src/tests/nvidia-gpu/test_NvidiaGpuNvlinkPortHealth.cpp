/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "DbusMockTestBase.hpp"
#include "LoggingStub.hpp"
#include "NvidiaEventReporting.hpp"
#include "NvidiaGpuNvlinkPortHealth.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <span>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr const char* criticalLevel =
    "xyz.openbmc_project.Logging.Entry.Level.Critical";

constexpr std::chrono::seconds pumpTimeout{5};
constexpr std::chrono::seconds quietWindow{1};

constexpr uint32_t portRcvErrorsBit = 1U << 0;
constexpr uint32_t symbolBerBit = 1U << 2;

constexpr uint32_t mixedKnownAndReservedMask = symbolBerBit | (1U << 7);

std::array<uint8_t, 8> makeEvent(uint8_t portNumber, uint32_t thresholdMask)
{
    return {portNumber,
            0,
            0,
            0,
            static_cast<uint8_t>(thresholdMask),
            static_cast<uint8_t>(thresholdMask >> 8),
            static_cast<uint8_t>(thresholdMask >> 16),
            static_cast<uint8_t>(thresholdMask >> 24)};
}

} // namespace

class NvidiaGpuNvlinkPortHealthTest : public DbusMockTestBase
{
  protected:
    void SetUp() override
    {
        DbusMockTestBase::SetUp();
        if (testing::Test::IsSkipped())
        {
            return;
        }
        ASSERT_TRUE(logging_stub::ensure(bus(), objects()));
        logging_stub::setActiveCall(&createCall);
    }

    void TearDown() override
    {
        logging_stub::setActiveCall(nullptr);
        DbusMockTestBase::TearDown();
    }

    logging_stub::CreateCall createCall;
};

TEST_F(NvidiaGpuNvlinkPortHealthTest, PortHealthEventCreatesCriticalLog)
{
    auto handler =
        std::make_shared<NvidiaNvlinkPortHealthEventHandler>("GPU_0", bus());

    const EventInfo eventInfo{};
    const auto event = makeEvent(3, portRcvErrorsBit);
    handler->handleNvlinkPortHealthEvent(eventInfo, event);

    ASSERT_TRUE(
        pumpIoUntil([this] { return createCall.count > 0; }, pumpTimeout));

    EXPECT_EQ(createCall.count, 1);
    EXPECT_THAT(createCall.message, testing::HasSubstr("GPU_0"));
    EXPECT_THAT(createCall.message, testing::HasSubstr("NVLink Port 3"));
    EXPECT_THAT(createCall.message, testing::HasSubstr("port_rcv_errors"));
    EXPECT_EQ(createCall.severity, criticalLevel);
    EXPECT_TRUE(createCall.additionalData.empty());
}

TEST_F(NvidiaGpuNvlinkPortHealthTest, ReservedThresholdBitReachesTheLog)
{
    auto handler =
        std::make_shared<NvidiaNvlinkPortHealthEventHandler>("GPU_1", bus());

    const EventInfo eventInfo{};
    const auto event = makeEvent(1, mixedKnownAndReservedMask);
    handler->handleNvlinkPortHealthEvent(eventInfo, event);

    ASSERT_TRUE(
        pumpIoUntil([this] { return createCall.count > 0; }, pumpTimeout));

    EXPECT_EQ(createCall.count, 1);
    EXPECT_THAT(createCall.message, testing::HasSubstr("symbol_ber"));
    EXPECT_THAT(createCall.message, testing::HasSubstr("0x00000084"));
    EXPECT_EQ(createCall.severity, criticalLevel);
}

TEST_F(NvidiaGpuNvlinkPortHealthTest, TruncatedEventIsNotLogged)
{
    auto handler =
        std::make_shared<NvidiaNvlinkPortHealthEventHandler>("GPU_2", bus());

    const EventInfo eventInfo{};
    const std::array<uint8_t, 4> truncated{2, 0, 0, 0};
    handler->handleNvlinkPortHealthEvent(eventInfo, truncated);

    EXPECT_FALSE(
        pumpIoUntil([this] { return createCall.count > 0; }, quietWindow));
    EXPECT_EQ(createCall.count, 0);
}

TEST_F(NvidiaGpuNvlinkPortHealthTest, HandlerDestroyedBeforeReplyIsSafe)
{
    auto handler =
        std::make_shared<NvidiaNvlinkPortHealthEventHandler>("GPU_3", bus());

    const EventInfo eventInfo{};
    const auto event = makeEvent(7, portRcvErrorsBit);
    handler->handleNvlinkPortHealthEvent(eventInfo, event);

    handler.reset();

    ASSERT_TRUE(
        pumpIoUntil([this] { return createCall.count > 0; }, pumpTimeout));
    EXPECT_EQ(createCall.count, 1);

    drainPendingAsync();
    pumpIoUntil([] { return false; }, quietWindow);
}
