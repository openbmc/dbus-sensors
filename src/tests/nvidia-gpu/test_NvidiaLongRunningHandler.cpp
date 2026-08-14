/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaLongRunningHandler.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/system/error_code.hpp>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include <gtest/gtest.h>

namespace
{

constexpr uint8_t msgTypePlatformEnv =
    static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);
constexpr uint8_t commandCode = static_cast<uint8_t>(
    gpu::PlatformEnvironmentalCommands::GET_CURRENT_UTILIZATION);

class NvidiaLongRunningHandlerTest : public ::testing::Test
{
  protected:
    // Shared across the fixture's tests so the binary creates a single
    // io_uring instance. A per-test io_context exhausts the io_uring / locked
    // memory budget in constrained CI containers (io_uring_queue_init ENOMEM).
    inline static boost::asio::io_context io;
    std::shared_ptr<NvidiaLongRunningResponseHandler> handler =
        std::make_shared<NvidiaLongRunningResponseHandler>(io);
};

TEST_F(NvidiaLongRunningHandlerTest, RegisterReturnsZeroFirstTime)
{
    const int rc = handler->registerResponseHandler(
        gpu::MessageType::PLATFORM_ENVIRONMENTAL, commandCode, 5,
        [](boost::system::error_code,
           ocp::accelerator_management::CompletionCode, uint16_t,
           std::span<const uint8_t>) {});
    EXPECT_EQ(rc, 0);
}

TEST_F(NvidiaLongRunningHandlerTest, RegisterDuplicateReturnsEEXIST)
{
    EXPECT_EQ(handler->registerResponseHandler(
                  gpu::MessageType::PLATFORM_ENVIRONMENTAL, commandCode, 5,
                  [](boost::system::error_code,
                     ocp::accelerator_management::CompletionCode, uint16_t,
                     std::span<const uint8_t>) {}),
              0);
    EXPECT_EQ(handler->registerResponseHandler(
                  gpu::MessageType::PLATFORM_ENVIRONMENTAL, commandCode, 5,
                  [](boost::system::error_code,
                     ocp::accelerator_management::CompletionCode, uint16_t,
                     std::span<const uint8_t>) {}),
              EEXIST);
}

TEST_F(NvidiaLongRunningHandlerTest, HandlerWrongEventClassIgnored)
{
    bool called = false;
    handler->registerResponseHandler(
        gpu::MessageType::PLATFORM_ENVIRONMENTAL, commandCode, 5,
        [&called](boost::system::error_code,
                  ocp::accelerator_management::CompletionCode, uint16_t,
                  std::span<const uint8_t>) { called = true; });

    const auto data =
        test_utils::buildLongRunningEventData(5, 0, 0, {0x01, 0x02});
    handler->handler(
        test_utils::makeEventInfo(5, msgTypePlatformEnv, commandCode), data);

    EXPECT_FALSE(called);
}

TEST_F(NvidiaLongRunningHandlerTest, HandlerDispatchesToRegisteredHandler)
{
    bool called = false;
    ocp::accelerator_management::CompletionCode capturedCc{
        ocp::accelerator_management::CompletionCode::ERROR};
    std::vector<uint8_t> capturedData;

    handler->registerResponseHandler(
        gpu::MessageType::PLATFORM_ENVIRONMENTAL, commandCode, 5,
        [&](boost::system::error_code ec,
            ocp::accelerator_management::CompletionCode cc, uint16_t,
            std::span<const uint8_t> responseData) {
            called = true;
            EXPECT_FALSE(ec);
            capturedCc = cc;
            capturedData.assign(responseData.begin(), responseData.end());
        });

    const std::vector<uint8_t> payload{0xAA, 0xBB, 0xCC};
    const auto data = test_utils::buildLongRunningEventData(
        5,
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::SUCCESS),
        0, payload);
    handler->handler(
        test_utils::makeEventInfo(test_utils::longRunningResponseEventClass,
                                  msgTypePlatformEnv, commandCode),
        data);

    EXPECT_TRUE(called);
    EXPECT_EQ(capturedCc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(capturedData, payload);
}

TEST_F(NvidiaLongRunningHandlerTest, HandlerNoMatchingKeyIgnored)
{
    bool called = false;
    handler->registerResponseHandler(
        gpu::MessageType::PLATFORM_ENVIRONMENTAL, commandCode, 5,
        [&called](boost::system::error_code,
                  ocp::accelerator_management::CompletionCode, uint16_t,
                  std::span<const uint8_t>) { called = true; });

    // Event for a different command code -> no matching handler.
    const uint8_t otherCommand = commandCode + 1;
    const auto data = test_utils::buildLongRunningEventData(5, 0, 0, {0x01});
    handler->handler(
        test_utils::makeEventInfo(test_utils::longRunningResponseEventClass,
                                  msgTypePlatformEnv, otherCommand),
        data);

    EXPECT_FALSE(called);
}

TEST_F(NvidiaLongRunningHandlerTest, HandlerDecodeErrorTooShort)
{
    bool called = false;
    handler->registerResponseHandler(
        gpu::MessageType::PLATFORM_ENVIRONMENTAL, commandCode, 5,
        [&called](boost::system::error_code,
                  ocp::accelerator_management::CompletionCode, uint16_t,
                  std::span<const uint8_t>) { called = true; });

    // Fewer than longRunningResponseEventSize (4) bytes -> decode fails.
    const std::vector<uint8_t> data{0x05, 0x00};
    handler->handler(
        test_utils::makeEventInfo(test_utils::longRunningResponseEventClass,
                                  msgTypePlatformEnv, commandCode),
        data);

    EXPECT_FALSE(called);
}

TEST_F(NvidiaLongRunningHandlerTest, HandlerConsumesHandlerOnce)
{
    int callCount = 0;
    handler->registerResponseHandler(
        gpu::MessageType::PLATFORM_ENVIRONMENTAL, commandCode, 5,
        [&callCount](boost::system::error_code,
                     ocp::accelerator_management::CompletionCode, uint16_t,
                     std::span<const uint8_t>) { callCount++; });

    const auto data = test_utils::buildLongRunningEventData(5, 0, 0, {0x01});
    const auto info =
        test_utils::makeEventInfo(test_utils::longRunningResponseEventClass,
                                  msgTypePlatformEnv, commandCode);

    handler->handler(info, data);
    handler->handler(info, data); // entry already erased

    EXPECT_EQ(callCount, 1);
}

TEST_F(NvidiaLongRunningHandlerTest, HandlerTimesOutWhenNoEventArrives)
{
    int callCount = 0;
    boost::system::error_code capturedEc;
    ocp::accelerator_management::CompletionCode capturedCc{
        ocp::accelerator_management::CompletionCode::SUCCESS};

    handler->registerResponseHandler(
        gpu::MessageType::PLATFORM_ENVIRONMENTAL, commandCode, 5,
        [&](boost::system::error_code ec,
            ocp::accelerator_management::CompletionCode cc, uint16_t,
            std::span<const uint8_t> responseData) {
            callCount++;
            capturedEc = ec;
            capturedCc = cc;
            EXPECT_TRUE(responseData.empty());
        });

    // No response event is injected, so the registration's own timer must
    // fire. The wait is bounded well above the handler's 2s timeout;
    // run_for() returns as soon as the timer completes and no work is left.
    io.restart();
    io.run_for(std::chrono::seconds(10));

    ASSERT_EQ(callCount, 1);
    EXPECT_EQ(capturedEc, boost::system::errc::timed_out);
    EXPECT_EQ(capturedCc, ocp::accelerator_management::CompletionCode::ERROR);

    // Timing out erased the entry, so a late event finds no handler.
    handler->handler(
        test_utils::makeEventInfo(test_utils::longRunningResponseEventClass,
                                  msgTypePlatformEnv, commandCode),
        test_utils::buildLongRunningEventData(5, 0, 0, {0x01}));

    EXPECT_EQ(callCount, 1);
}

} // namespace
