/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuClockFrequencyMetric.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <sdbusplus/exception.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr double mhzToHz = 1'000'000.0;
constexpr const char* metricIface = "xyz.openbmc_project.Metric.Value";

std::vector<uint8_t> buildClockFreqResponse(uint32_t mhz)
{
    return test_utils::buildPlatformEnvSuccessResponse(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_CLOCK_FREQUENCY, mhz);
}

std::vector<uint8_t> buildErrorResponse()
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_CLOCK_FREQUENCY,
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::ERROR),
        0x1234);
}

class NvidiaGpuClockFrequencyMetricTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaGpuClockFrequencyMetric> createMetric(
        const std::string& name = "GPU_FREQ",
        uint8_t eid = test_utils::defaultEid)
    {
        return std::make_shared<NvidiaGpuClockFrequencyMetric>(
            requester(), name, eid, objects());
    }

    static std::string metricDbusPath(const std::string& name)
    {
        return "/xyz/openbmc_project/metric/" + name + "/OperatingFrequency";
    }
};

TEST_F(NvidiaGpuClockFrequencyMetricTest, ConstructorDoesNotCrash)
{
    auto metric = createMetric("freq_ctor");
    ASSERT_NE(metric, nullptr);
}

TEST_F(NvidiaGpuClockFrequencyMetricTest, UpdateSuccessSetsValue)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildClockFreqResponse(1410)));

    auto metric = createMetric("freq_succ");
    metric->update();

    EXPECT_DOUBLE_EQ(
        getProperty<double>(metricDbusPath("freq_succ"), metricIface, "Value"),
        1410.0 * mhzToHz);
}

TEST_F(NvidiaGpuClockFrequencyMetricTest, UpdateSendsRequest)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith({}, {}));

    auto metric = createMetric("freq_sends");
    metric->update();
}

TEST_F(NvidiaGpuClockFrequencyMetricTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, {}));

    auto metric = createMetric("freq_eid", testEid);
    metric->update();
}

TEST_F(NvidiaGpuClockFrequencyMetricTest, UpdateVerifiesRequestEncoding)
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

    auto metric = createMetric("freq_enc");
    metric->update();

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
    uint8_t dataSize = 0;
    uint8_t clockType = 0;
    unpack.unpack(command);
    unpack.unpack(dataSize);
    unpack.unpack(clockType);
    EXPECT_EQ(
        command,
        static_cast<uint8_t>(
            gpu::PlatformEnvironmentalCommands::GET_CURRENT_CLOCK_FREQUENCY));
    EXPECT_EQ(clockType, static_cast<uint8_t>(gpu::ClockType::GRAPHICS_CLOCK));
    EXPECT_EQ(unpack.getError(), 0);
}

TEST_F(NvidiaGpuClockFrequencyMetricTest, UpdateDecodeErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildErrorResponse()));

    auto metric = createMetric("freq_dec_err");
    EXPECT_NO_THROW(metric->update());
}

TEST_F(NvidiaGpuClockFrequencyMetricTest, UpdateEmptyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    auto metric = createMetric("freq_empty");
    EXPECT_NO_THROW(metric->update());
}

TEST_F(NvidiaGpuClockFrequencyMetricTest, UpdateTinyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, {0x00, 0x01}));

    auto metric = createMetric("freq_tiny");
    EXPECT_NO_THROW(metric->update());
}

TEST_F(NvidiaGpuClockFrequencyMetricTest, UpdateMctpTransportError)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    auto metric = createMetric("freq_mctp_err_val");
    metric->update();

    // A transport error must leave the value at its default (0).
    EXPECT_DOUBLE_EQ(getProperty<double>(metricDbusPath("freq_mctp_err_val"),
                                         metricIface, "Value"),
                     0.0);
}

TEST_F(NvidiaGpuClockFrequencyMetricTest,
       UpdateSuccessThenErrorKeepsPreviousValue)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildClockFreqResponse(1410)))
        .WillOnce(mock_mctp::respondWith({}, buildErrorResponse()));

    auto metric = createMetric("freq_keep");

    metric->update();
    EXPECT_DOUBLE_EQ(
        getProperty<double>(metricDbusPath("freq_keep"), metricIface, "Value"),
        1410.0 * mhzToHz);

    // A subsequent error response must not overwrite the last good value.
    metric->update();
    EXPECT_DOUBLE_EQ(
        getProperty<double>(metricDbusPath("freq_keep"), metricIface, "Value"),
        1410.0 * mhzToHz);
}

TEST_F(NvidiaGpuClockFrequencyMetricTest, DestructorRemovesInterface)
{
    const std::string name = "freq_dtor";
    {
        auto metric = createMetric(name);
        ASSERT_NE(metric, nullptr);
        EXPECT_NO_THROW(
            getProperty<double>(metricDbusPath(name), metricIface, "Value"));
    }
    drainPendingAsync();
    EXPECT_THROW(
        getProperty<double>(metricDbusPath(name), metricIface, "Value"),
        sdbusplus::exception_t);
}

} // namespace
