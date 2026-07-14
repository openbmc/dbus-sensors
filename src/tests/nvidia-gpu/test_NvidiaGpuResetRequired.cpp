/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "DbusMockTestBase.hpp"
#include "NvidiaEventReporting.hpp"
#include "NvidiaGpuResetRequired.hpp"

#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr const char* loggingService = "xyz.openbmc_project.Logging";
constexpr const char* loggingPath = "/xyz/openbmc_project/logging";
constexpr const char* loggingCreateIface = "xyz.openbmc_project.Logging.Create";
constexpr const char* criticalLevel =
    "xyz.openbmc_project.Logging.Entry.Level.Critical";

// Long enough to cover the self-directed D-Bus round trip the fixture pumps.
constexpr std::chrono::seconds pumpTimeout{5};

// Arguments captured from a single Logging.Create call.
struct CreateCall
{
    int count{0};
    std::string message;
    std::string severity;
    std::map<std::string, std::string> additionalData;
};

// The stub Create method is registered once for the whole binary, so it records
// into whichever test's capture is currently active (mirrors the
// mock_mctp::setActiveMock link-seam pattern).
CreateCall* activeCreateCall = nullptr;
std::shared_ptr<sdbusplus::asio::dbus_interface> fakeLoggingIface;

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
        ensureFakeLoggingService();
        activeCreateCall = &createCall;
    }

    void TearDown() override
    {
        activeCreateCall = nullptr;
        DbusMockTestBase::TearDown();
    }

    // Stand up a stub xyz.openbmc_project.Logging.Create once and own the
    // well-known name so the handler's async_method_call routes back to this
    // connection. The stub records the Create arguments into the active
    // capture.
    static void ensureFakeLoggingService()
    {
        if (fakeLoggingIface)
        {
            return;
        }
        bus()->request_name(loggingService);
        fakeLoggingIface =
            objects().add_interface(loggingPath, loggingCreateIface);
        fakeLoggingIface->register_method(
            "Create", [](std::string message, std::string severity,
                         std::map<std::string, std::string> additionalData) {
                if (activeCreateCall != nullptr)
                {
                    activeCreateCall->count++;
                    activeCreateCall->message = std::move(message);
                    activeCreateCall->severity = std::move(severity);
                    activeCreateCall->additionalData =
                        std::move(additionalData);
                }
            });
        ASSERT_TRUE(fakeLoggingIface->initialize());
    }

    CreateCall createCall;
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
