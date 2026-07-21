/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaPciePortMetrics.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

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

constexpr uint8_t defaultPortNumber = 0;
constexpr uint8_t defaultUpstreamPortNumber = 0;

// QueryScalarGroupTelemetryV2 response: CommonResponse(11) + N*uint32_t
// For PCIe port errors (scalar group 2): indices [0..3] map to
// non_fatal_error_count, fatal_error_count, unsupported_request_count,
// correctable_error_count
std::vector<uint8_t> buildMetricsTelemetryResponse(
    const std::vector<uint32_t>& values)
{
    return test_utils::buildPcieScalarTelemetryResponse(values);
}

class NvidiaPciePortMetricsTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaPciePortMetrics> createPciePortMetrics(
        const std::string& name = "Port_0",
        const std::string& pcieDeviceName = "PCIE_DEV_METRICS",
        uint8_t eid = test_utils::defaultEid)
    {
        const std::string path = "/test/pcie/" + pcieDeviceName;
        return makeNvidiaPciePortErrors(
            bus(), requester(), name, pcieDeviceName, path, eid,
            gpu::PciePortType::DOWNSTREAM, defaultUpstreamPortNumber,
            defaultPortNumber, objects(),
            gpu::DeviceIdentification::DEVICE_PCIE);
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaPciePortMetricsTest, ConstructorCreatesMetricInterfaces)
{
    const std::shared_ptr<NvidiaPciePortMetrics> metrics =
        createPciePortMetrics("Port_0", "pcie_metrics_ctor");
    ASSERT_NE(metrics, nullptr);
}

// Update — successful response updates D-Bus metric values

TEST_F(NvidiaPciePortMetricsTest, UpdateSuccessUpdatesMetricValues)
{
    // non_fatal=10, fatal=2, unsupported=5, correctable=100
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildMetricsTelemetryResponse({10, 2, 5, 100})));

    const std::string pcieDeviceName = "pcie_metrics_upd";
    const std::shared_ptr<NvidiaPciePortMetrics> metrics =
        createPciePortMetrics("Port_0", pcieDeviceName);
    metrics->update();

    const std::string metricsBase =
        "/xyz/openbmc_project/metric/port_" + pcieDeviceName + "_Port_0";
    EXPECT_EQ(getProperty<double>(metricsBase + "/pcie/non_fatal_error_count",
                                  "xyz.openbmc_project.Metric.Value", "Value"),
              10.0);
    EXPECT_EQ(getProperty<double>(metricsBase + "/pcie/fatal_error_count",
                                  "xyz.openbmc_project.Metric.Value", "Value"),
              2.0);
    EXPECT_EQ(
        getProperty<double>(metricsBase + "/pcie/unsupported_request_count",
                            "xyz.openbmc_project.Metric.Value", "Value"),
        5.0);
    EXPECT_EQ(getProperty<double>(metricsBase + "/pcie/correctable_error_count",
                                  "xyz.openbmc_project.Metric.Value", "Value"),
              100.0);
}

// Update — request encoding verification

TEST_F(NvidiaPciePortMetricsTest, UpdateVerifiesRequestEncoding)
{
    // Copy the request bytes before completing the call: the reqMsg span is
    // a view into caller-owned memory, valid only during the call.
    std::vector<uint8_t> lastRequest;
    const std::vector<uint8_t> response;
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce([&](uint8_t /*eid*/, std::span<const uint8_t> reqMsg,
                      auto callback) {
            lastRequest.assign(reqMsg.begin(), reqMsg.end());
            callback(std::error_code{}, response);
        });

    const std::shared_ptr<NvidiaPciePortMetrics> metrics =
        createPciePortMetrics("Port_0", "pcie_metrics_req_enc");
    metrics->update();

    ASSERT_FALSE(lastRequest.empty());

    UnpackBuffer unpack(lastRequest);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    const int rc = ocp::accelerator_management::unpackHeader(
        unpack, gpu::nvidiaPciVendorId, ocpMsgType, instanceId, msgType);
    EXPECT_EQ(rc, 0);
    EXPECT_EQ(ocpMsgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(msgType, static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));

    uint8_t command = 0;
    unpack.unpack(command);
    EXPECT_EQ(command, static_cast<uint8_t>(
                           gpu::PcieLinkCommands::QueryScalarGroupTelemetryV2));

    EXPECT_EQ(unpack.getError(), 0);
}

// Update — sends request

TEST_F(NvidiaPciePortMetricsTest, UpdateSendsRequest)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaPciePortMetrics> metrics =
        createPciePortMetrics("Port_0", "pcie_metrics_sends");
    metrics->update();
}

TEST_F(NvidiaPciePortMetricsTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaPciePortMetrics> metrics =
        createPciePortMetrics("Port_0", "pcie_metrics_eid", testEid);
    metrics->update();
}

// Error handling

TEST_F(NvidiaPciePortMetricsTest, UpdateMctpTransportErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<NvidiaPciePortMetrics> metrics =
        createPciePortMetrics("Port_0", "pcie_metrics_mctp_err");
    EXPECT_NO_THROW(metrics->update());
}

TEST_F(NvidiaPciePortMetricsTest, UpdateEmptyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaPciePortMetrics> metrics =
        createPciePortMetrics("Port_0", "pcie_metrics_empty");
    EXPECT_NO_THROW(metrics->update());
}

} // namespace
