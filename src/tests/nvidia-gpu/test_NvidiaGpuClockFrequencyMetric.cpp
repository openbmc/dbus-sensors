/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuClockFrequencyMetric.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <MessagePackUnpackUtils.hpp>
#include <sdbusplus/exception.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

namespace
{

constexpr double mhzToHz = 1'000'000.0;
const std::string metricIface = "xyz.openbmc_project.Metric.Value";

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

class NvidiaGpuClockFrequencyMetricTestBase : public DbusMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaGpuClockFrequencyMetric> createMetric(
        const std::string& name = "GPU_FREQ",
        uint8_t eid = test_utils::defaultEid)
    {
        const std::string invPath = "/xyz/openbmc_project/inventory/" + name;
        return std::make_shared<NvidiaGpuClockFrequencyMetric>(
            *mctpRequester, name, eid, *objectServer, invPath);
    }

    static std::string metricDbusPath(const std::string& name)
    {
        return "/xyz/openbmc_project/metric/" + name + "/OperatingFrequency";
    }
};

TEST_F(NvidiaGpuClockFrequencyMetricTestBase, ConstructorDoesNotCrash)
{
    auto metric = createMetric("freq_ctor");
    ASSERT_NE(metric, nullptr);
}

TEST_F(NvidiaGpuClockFrequencyMetricTestBase, UpdateSuccessSetsValue)
{
    mock_mctp::setNextResponse({}, buildClockFreqResponse(1410));
    auto metric = createMetric("freq_succ");
    metric->update();

    EXPECT_DOUBLE_EQ(
        getProperty<double>(metricDbusPath("freq_succ"), metricIface, "Value"),
        1410.0 * mhzToHz);
}

TEST_F(NvidiaGpuClockFrequencyMetricTestBase, UpdateSendsRequest)
{
    mock_mctp::setNextResponse({}, {});
    auto metric = createMetric("freq_sends");
    metric->update();
    EXPECT_GE(mock_mctp::getRequestCount(), 1U);
}

TEST_F(NvidiaGpuClockFrequencyMetricTestBase, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    mock_mctp::setNextResponse({}, {});
    auto metric = createMetric("freq_eid", testEid);
    metric->update();
    EXPECT_EQ(mock_mctp::getLastEid(), testEid);
}

TEST_F(NvidiaGpuClockFrequencyMetricTestBase, UpdateVerifiesRequestEncoding)
{
    mock_mctp::setNextResponse({}, {});
    auto metric = createMetric("freq_enc");
    metric->update();

    const std::span<const uint8_t> req = mock_mctp::getLastRequest();
    ASSERT_FALSE(req.empty());

    UnpackBuffer unpack(req);
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

TEST_F(NvidiaGpuClockFrequencyMetricTestBase, UpdateMctpTransportErrorNoCrash)
{
    mock_mctp::setNextResponse(std::make_error_code(std::errc::timed_out), {});
    auto metric = createMetric("freq_mctp_err");
    EXPECT_NO_THROW(metric->update());
}

TEST_F(NvidiaGpuClockFrequencyMetricTestBase, UpdateDecodeErrorNoCrash)
{
    mock_mctp::setNextResponse({}, buildErrorResponse());
    auto metric = createMetric("freq_dec_err");
    EXPECT_NO_THROW(metric->update());
}

TEST_F(NvidiaGpuClockFrequencyMetricTestBase, UpdateEmptyBufferNoCrash)
{
    mock_mctp::setNextResponse({}, {});
    auto metric = createMetric("freq_empty");
    EXPECT_NO_THROW(metric->update());
}

TEST_F(NvidiaGpuClockFrequencyMetricTestBase, UpdateTinyBufferNoCrash)
{
    mock_mctp::setNextResponse({}, {0x00, 0x01});
    auto metric = createMetric("freq_tiny");
    EXPECT_NO_THROW(metric->update());
}

TEST_F(NvidiaGpuClockFrequencyMetricTestBase, DestructorRemovesInterface)
{
    const std::string name = "freq_dtor";
    {
        auto metric = createMetric(name);
        ASSERT_NE(metric, nullptr);
        EXPECT_NO_THROW(
            getProperty<double>(metricDbusPath(name), metricIface, "Value"));
    }
    EXPECT_THROW(
        getProperty<double>(metricDbusPath(name), metricIface, "Value"),
        sdbusplus::exception_t);
}

} // namespace
