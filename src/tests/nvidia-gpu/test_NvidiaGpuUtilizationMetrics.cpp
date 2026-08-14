/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaGpuUtilizationMetrics.hpp"
#include "NvidiaLongRunningHandler.hpp"
#include "OcpMctpVdm.hpp"
#include "SerialQueue.hpp"
#include "TestUtils.hpp"

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr const char* metricIface = "xyz.openbmc_project.Metric.Value";

// Build a GET_CURRENT_UTILIZATION response with the given completion code:
//   common response header + 2 x uint32_t (gpuUtil, memUtil)
std::vector<uint8_t> buildUtilizationResponse(
    ocp::accelerator_management::CompletionCode cc, uint32_t gpuUtil,
    uint32_t memUtil)
{
    const uint16_t dataSize = sizeof(uint32_t) * 2;
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + dataSize, 0);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_UTILIZATION));
    pack.pack(static_cast<uint8_t>(cc));
    pack.pack(static_cast<uint16_t>(0)); // reasonCode
    pack.pack(dataSize);
    pack.pack(gpuUtil);
    pack.pack(memUtil);
    EXPECT_EQ(pack.getError(), 0);
    return buf;
}

// The long-running response event carries the command payload on its own:
// 2 x uint32_t (gpuUtil, memUtil), with no response header.
std::vector<uint8_t> buildUtilizationPayload(uint32_t gpuUtil, uint32_t memUtil)
{
    std::vector<uint8_t> buf(sizeof(uint32_t) * 2, 0);
    PackBuffer pack(buf);
    pack.pack(gpuUtil);
    pack.pack(memUtil);
    EXPECT_EQ(pack.getError(), 0);
    return buf;
}

std::vector<uint8_t> buildErrorResponse()
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_UTILIZATION,
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::ERROR),
        0x1234);
}

class NvidiaGpuUtilizationMetricsTest : public MctpMockTestBase
{
  protected:
    std::shared_ptr<NvidiaGpuUtilizationMetrics> createUtil(
        const std::string& name = "GPU_UTIL",
        uint8_t eid = test_utils::defaultEid)
    {
        auto queue = std::make_shared<SerialQueue>(ioContext());
        return std::make_shared<NvidiaGpuUtilizationMetrics>(
            requester(), objects(), name, eid, std::move(queue), lrHandler);
    }

    static std::string processorMetricDbusPath(const std::string& name)
    {
        return "/xyz/openbmc_project/metric/gpu_" + name +
               "/processor_bandwidth";
    }

    static std::string memoryMetricDbusPath(const std::string& name)
    {
        return "/xyz/openbmc_project/metric/gpu_" + name + "/memory_bandwidth";
    }

    // Held by the fixture so a test can feed the deferred long-running
    // response event that follows an ACCEPTED reply.
    std::shared_ptr<NvidiaLongRunningResponseHandler> lrHandler =
        std::make_shared<NvidiaLongRunningResponseHandler>(ioContext());
};

TEST_F(NvidiaGpuUtilizationMetricsTest, ConstructorDoesNotCrash)
{
    auto util = createUtil("util_ctor");
    ASSERT_NE(util, nullptr);
}

TEST_F(NvidiaGpuUtilizationMetricsTest, UpdateImmediateSuccessSetsValue)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {},
            buildUtilizationResponse(
                ocp::accelerator_management::CompletionCode::SUCCESS, 85, 40)));

    auto util = createUtil("util_succ");
    util->update();

    EXPECT_DOUBLE_EQ(getProperty<double>(processorMetricDbusPath("util_succ"),
                                         metricIface, "Value"),
                     85.0);
    EXPECT_DOUBLE_EQ(getProperty<double>(memoryMetricDbusPath("util_succ"),
                                         metricIface, "Value"),
                     40.0);
}

TEST_F(NvidiaGpuUtilizationMetricsTest, UpdateSendsRequest)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith({}, {}));

    auto util = createUtil("util_sends");
    util->update();
}

TEST_F(NvidiaGpuUtilizationMetricsTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, {}));

    auto util = createUtil("util_eid", testEid);
    util->update();
}

TEST_F(NvidiaGpuUtilizationMetricsTest, UpdateVerifiesRequestEncoding)
{
    // Copy the request bytes before completing the call: the reqMsg span is
    // a view into caller-owned memory, valid only during the call.
    std::vector<uint8_t> lastRequest;
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce([&](uint8_t /*eid*/, std::span<const uint8_t> reqMsg,
                      auto callback) {
            lastRequest.assign(reqMsg.begin(), reqMsg.end());
            callback(std::error_code{}, std::span<const uint8_t>{});
        });

    auto util = createUtil("util_enc");
    util->update();

    ASSERT_FALSE(lastRequest.empty());

    UnpackBuffer unpack(lastRequest);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  unpack, gpu::nvidiaPciVendorId, ocpMsgType, instanceId,
                  msgType),
              0);
    EXPECT_EQ(msgType,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    uint8_t command = 0;
    unpack.unpack(command);
    EXPECT_EQ(command,
              static_cast<uint8_t>(
                  gpu::PlatformEnvironmentalCommands::GET_CURRENT_UTILIZATION));
    EXPECT_EQ(unpack.getError(), 0);
}

TEST_F(NvidiaGpuUtilizationMetricsTest,
       UpdateAcceptedThenLongRunningEventSetsValues)
{
    // WillOnce cardinality asserts the request was actually sent.
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {},
            buildUtilizationResponse(
                ocp::accelerator_management::CompletionCode::ACCEPTED, 0, 0)));

    auto util = createUtil("util_accepted");
    ASSERT_NO_THROW(util->update());

    // ACCEPTED only registers the response handler; the values stay at their
    // defaults until the long-running response event arrives.
    EXPECT_DOUBLE_EQ(
        getProperty<double>(processorMetricDbusPath("util_accepted"),
                            metricIface, "Value"),
        0.0);

    // The event payload carries the command data only, with no response
    // header, and the ACCEPTED reply above was built with instance id 0.
    lrHandler->handler(
        test_utils::makeEventInfo(
            test_utils::longRunningResponseEventClass,
            static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL),
            static_cast<uint8_t>(
                gpu::PlatformEnvironmentalCommands::GET_CURRENT_UTILIZATION)),
        test_utils::buildLongRunningEventData(
            0,
            static_cast<uint8_t>(
                ocp::accelerator_management::CompletionCode::SUCCESS),
            0, buildUtilizationPayload(85, 40)));

    EXPECT_DOUBLE_EQ(
        getProperty<double>(processorMetricDbusPath("util_accepted"),
                            metricIface, "Value"),
        85.0);
    EXPECT_DOUBLE_EQ(getProperty<double>(memoryMetricDbusPath("util_accepted"),
                                         metricIface, "Value"),
                     40.0);
}

TEST_F(NvidiaGpuUtilizationMetricsTest, UpdateMctpTransportErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    auto util = createUtil("util_mctp_err");
    EXPECT_NO_THROW(util->update());
}

TEST_F(NvidiaGpuUtilizationMetricsTest, UpdateDecodeErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildErrorResponse()));

    auto util = createUtil("util_dec_err");
    EXPECT_NO_THROW(util->update());
}

TEST_F(NvidiaGpuUtilizationMetricsTest, UpdateEmptyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    auto util = createUtil("util_empty");
    EXPECT_NO_THROW(util->update());
}

TEST_F(NvidiaGpuUtilizationMetricsTest, UpdateTinyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, {0x00, 0x01}));

    auto util = createUtil("util_tiny");
    EXPECT_NO_THROW(util->update());
}

TEST_F(NvidiaGpuUtilizationMetricsTest,
       UpdateSuccessThenErrorKeepsPreviousValue)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {},
            buildUtilizationResponse(
                ocp::accelerator_management::CompletionCode::SUCCESS, 85, 40)))
        .WillOnce(mock_mctp::respondWith({}, buildErrorResponse()));

    auto util = createUtil("util_keep");

    util->update();
    EXPECT_DOUBLE_EQ(getProperty<double>(processorMetricDbusPath("util_keep"),
                                         metricIface, "Value"),
                     85.0);

    // A subsequent error response must not overwrite the last good values.
    util->update();
    EXPECT_DOUBLE_EQ(getProperty<double>(processorMetricDbusPath("util_keep"),
                                         metricIface, "Value"),
                     85.0);
    EXPECT_DOUBLE_EQ(getProperty<double>(memoryMetricDbusPath("util_keep"),
                                         metricIface, "Value"),
                     40.0);
}

} // namespace
