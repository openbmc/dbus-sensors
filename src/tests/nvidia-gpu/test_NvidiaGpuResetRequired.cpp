/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "DbusMockTestBase.hpp"
#include "LoggingStub.hpp"
#include "NvidiaEventReporting.hpp"
#include "NvidiaGpuResetRequired.hpp"

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

// Long enough to cover the self-directed D-Bus round trip the fixture pumps.
constexpr std::chrono::seconds pumpTimeout{5};

} // namespace

class NvidiaGpuResetRequiredTest : public DbusMockTestBase
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

// The Reset Required event carries no payload; the handler must create a
// critical phosphor-logging entry that names the resource requiring the reset.
TEST_F(NvidiaGpuResetRequiredTest, ResetRequiredEventCreatesCriticalLog)
{
    auto handler =
        std::make_shared<NvidiaResetRequiredEventHandler>("GPU_0", bus());

    const EventInfo eventInfo{};
    handler->handleResetRequiredEvent(eventInfo, std::span<const uint8_t>{});

    ASSERT_TRUE(
        pumpIoUntil([this] { return createCall.count > 0; }, pumpTimeout));

    EXPECT_EQ(createCall.count, 1);
    EXPECT_THAT(createCall.message, testing::HasSubstr("GPU_0"));
    EXPECT_THAT(createCall.message, testing::HasSubstr("requires a reset"));
    EXPECT_EQ(createCall.severity, criticalLevel);
    EXPECT_TRUE(createCall.additionalData.empty());
}

// Dropping the handler before the async Create reply is processed must not
// crash: the completion callback holds only a weak_ptr and has to no-op once
// the handler is gone.
TEST_F(NvidiaGpuResetRequiredTest, HandlerDestroyedBeforeReplyIsSafe)
{
    auto handler =
        std::make_shared<NvidiaResetRequiredEventHandler>("GPU_3", bus());

    const EventInfo eventInfo{};
    handler->handleResetRequiredEvent(eventInfo, std::span<const uint8_t>{});

    handler.reset();

    // The request is already dispatched, so the stub still records it while the
    // now-expired completion callback drains harmlessly.
    ASSERT_TRUE(
        pumpIoUntil([this] { return createCall.count > 0; }, pumpTimeout));
    EXPECT_EQ(createCall.count, 1);
}
