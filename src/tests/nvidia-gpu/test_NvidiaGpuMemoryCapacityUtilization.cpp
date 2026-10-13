/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaGpuMemoryCapacityUtilization.hpp"
#include "NvidiaLongRunningHandler.hpp"
#include "OcpMctpVdm.hpp"
#include "SerialQueue.hpp"
#include "TestUtils.hpp"
#include "Utils.hpp"

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <tuple>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr const char* metricIface = "xyz.openbmc_project.Metric.Value";

// Build a GET_MEMORY_CAPACITY_UTILIZATION response with the given completion
// code: common response header + 2 x uint32_t (reservedMemory, usedMemory)
std::vector<uint8_t> buildCapacityResponse(
    ocp::accelerator_management::CompletionCode cc, uint32_t reservedMemory,
    uint32_t usedMemory)
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
        gpu::PlatformEnvironmentalCommands::GET_MEMORY_CAPACITY_UTILIZATION));
    pack.pack(static_cast<uint8_t>(cc));
    pack.pack(static_cast<uint16_t>(0)); // reasonCode
    pack.pack(dataSize);
    pack.pack(reservedMemory);
    pack.pack(usedMemory);
    EXPECT_EQ(pack.getError(), 0);
    return buf;
}

std::vector<uint8_t> buildErrorResponse()
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::GET_MEMORY_CAPACITY_UTILIZATION,
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::ERROR),
        0x1234);
}

class NvidiaGpuMemoryCapacityUtilizationTest : public MctpMockTestBase
{
  protected:
    // The Inventory supplies the memory size the raw counters are turned into
    // a percentage against. Passing nullptr keeps these tests focused on the
    // objects the constructor publishes and on the transport paths, which do
    // not need it.
    std::shared_ptr<NvidiaGpuMemoryCapacityUtilization> createMetric(
        const std::string& name = "GPU_MEMCAP",
        uint8_t eid = test_utils::defaultEid)
    {
        auto queue = std::make_shared<SerialQueue>(ioContext());
        return std::make_shared<NvidiaGpuMemoryCapacityUtilization>(
            requester(), objects(), name, eid, std::move(queue), lrHandler,
            nullptr);
    }

    static std::string processorMetricDbusPath(const std::string& name)
    {
        return "/xyz/openbmc_project/metric/gpu_" + name +
               "/memory_capacity_utilization";
    }

    static std::string processorInventoryPath(const std::string& name)
    {
        return "/xyz/openbmc_project/inventory/" + name;
    }

    std::shared_ptr<NvidiaLongRunningResponseHandler> lrHandler =
        std::make_shared<NvidiaLongRunningResponseHandler>(ioContext());
};

TEST_F(NvidiaGpuMemoryCapacityUtilizationTest, ConstructorDoesNotCrash)
{
    auto metric = createMetric("memcap_ctor");
    ASSERT_NE(metric, nullptr);
}

TEST_F(NvidiaGpuMemoryCapacityUtilizationTest, ProcessorMetricIsPublished)
{
    auto metric = createMetric("memcap_pub");

    EXPECT_EQ(getProperty<std::string>(processorMetricDbusPath("memcap_pub"),
                                       metricIface, "Unit"),
              "xyz.openbmc_project.Metric.Value.Unit.Percent");
    EXPECT_DOUBLE_EQ(getProperty<double>(processorMetricDbusPath("memcap_pub"),
                                         metricIface, "Value"),
                     0.0);
}

// The metric is only reachable from Redfish by walking measured_by from the
// inventory object, so the association target has to be the object the rest
// of the daemon publishes for this GPU.
TEST_F(NvidiaGpuMemoryCapacityUtilizationTest, ProcessorMetricMeasuresTheGpu)
{
    auto metric = createMetric("memcap_assoc");

    const auto associations = getProperty<std::vector<Association>>(
        processorMetricDbusPath("memcap_assoc"), association::interface,
        "Associations");

    ASSERT_EQ(associations.size(), 1U);
    EXPECT_EQ(std::get<0>(associations[0]), "measuring");
    EXPECT_EQ(std::get<1>(associations[0]), "measured_by");
    EXPECT_EQ(std::get<2>(associations[0]),
              processorInventoryPath("memcap_assoc"));
}

TEST_F(NvidiaGpuMemoryCapacityUtilizationTest, UpdateSendsRequest)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith({}, {}));

    auto metric = createMetric("memcap_sends");
    metric->update();
}

TEST_F(NvidiaGpuMemoryCapacityUtilizationTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, {}));

    auto metric = createMetric("memcap_eid", testEid);
    metric->update();
}

TEST_F(NvidiaGpuMemoryCapacityUtilizationTest, UpdateVerifiesRequestEncoding)
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

    auto metric = createMetric("memcap_enc");
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
    unpack.unpack(command);
    EXPECT_EQ(command,
              static_cast<uint8_t>(gpu::PlatformEnvironmentalCommands::
                                       GET_MEMORY_CAPACITY_UTILIZATION));
    EXPECT_EQ(unpack.getError(), 0);
}

// Without an Inventory there is no memory size to turn the counters into a
// percentage, so a good response must leave the value alone rather than
// publish a meaningless number.
TEST_F(NvidiaGpuMemoryCapacityUtilizationTest, UpdateWithoutInventoryKeepsZero)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildCapacityResponse(
                    ocp::accelerator_management::CompletionCode::SUCCESS, 1024,
                    2048)));

    auto metric = createMetric("memcap_noinv");
    EXPECT_NO_THROW(metric->update());

    EXPECT_DOUBLE_EQ(
        getProperty<double>(processorMetricDbusPath("memcap_noinv"),
                            metricIface, "Value"),
        0.0);
}

TEST_F(NvidiaGpuMemoryCapacityUtilizationTest, UpdateMctpTransportErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    auto metric = createMetric("memcap_mctp_err");
    EXPECT_NO_THROW(metric->update());
}

TEST_F(NvidiaGpuMemoryCapacityUtilizationTest, UpdateDecodeErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildErrorResponse()));

    auto metric = createMetric("memcap_dec_err");
    EXPECT_NO_THROW(metric->update());
}

TEST_F(NvidiaGpuMemoryCapacityUtilizationTest, UpdateEmptyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    auto metric = createMetric("memcap_empty");
    EXPECT_NO_THROW(metric->update());
}

TEST_F(NvidiaGpuMemoryCapacityUtilizationTest, UpdateTinyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, {0x00, 0x01}));

    auto metric = createMetric("memcap_tiny");
    EXPECT_NO_THROW(metric->update());
}

} // namespace
