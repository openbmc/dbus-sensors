/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaPciePortMetrics.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <MessagePackUnpackUtils.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

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

class NvidiaPciePortMetricsTestBase : public DbusMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaPciePortMetrics> createPciePortMetrics(
        const std::string& name = "Port_0",
        const std::string& pcieDeviceName = "PCIE_DEV_METRICS",
        uint8_t eid = test_utils::defaultEid)
    {
        const std::string path = "/test/pcie/" + pcieDeviceName;
        return makeNvidiaPciePortErrors(
            conn, *mctpRequester, name, pcieDeviceName, path, eid,
            gpu::PciePortType::DOWNSTREAM, defaultUpstreamPortNumber,
            defaultPortNumber, *objectServer,
            gpu::DeviceIdentification::DEVICE_PCIE);
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaPciePortMetricsTestBase, ConstructorCreatesMetricInterfaces)
{
    const std::shared_ptr<NvidiaPciePortMetrics> metrics =
        createPciePortMetrics("Port_0", "pcie_metrics_ctor");
    ASSERT_NE(metrics, nullptr);
}

// Update — successful response updates D-Bus metric values

TEST_F(NvidiaPciePortMetricsTestBase, UpdateSuccessUpdatesMetricValues)
{
    // non_fatal=10, fatal=2, unsupported=5, correctable=100
    mock_mctp::setNextResponse({},
                               buildMetricsTelemetryResponse({10, 2, 5, 100}));
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

TEST_F(NvidiaPciePortMetricsTestBase, UpdateVerifiesRequestEncoding)
{
    mock_mctp::setNextResponse({}, {});
    const std::shared_ptr<NvidiaPciePortMetrics> metrics =
        createPciePortMetrics("Port_0", "pcie_metrics_req_enc");
    metrics->update();

    const std::span<const uint8_t> req = mock_mctp::getLastRequest();
    ASSERT_FALSE(req.empty());

    UnpackBuffer unpack(req);
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

TEST_F(NvidiaPciePortMetricsTestBase, UpdateSendsRequest)
{
    mock_mctp::setNextResponse({}, {});
    const std::shared_ptr<NvidiaPciePortMetrics> metrics =
        createPciePortMetrics("Port_0", "pcie_metrics_sends");
    metrics->update();
    EXPECT_GE(mock_mctp::getRequestCount(), 1U);
}

TEST_F(NvidiaPciePortMetricsTestBase, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    mock_mctp::setNextResponse({}, {});
    const std::shared_ptr<NvidiaPciePortMetrics> metrics =
        createPciePortMetrics("Port_0", "pcie_metrics_eid", testEid);
    metrics->update();
    EXPECT_EQ(mock_mctp::getLastEid(), testEid);
}

// Error handling

TEST_F(NvidiaPciePortMetricsTestBase, UpdateMctpTransportErrorNoCrash)
{
    mock_mctp::setNextResponse(std::make_error_code(std::errc::timed_out), {});
    const std::shared_ptr<NvidiaPciePortMetrics> metrics =
        createPciePortMetrics("Port_0", "pcie_metrics_mctp_err");
    EXPECT_NO_THROW(metrics->update());
}

TEST_F(NvidiaPciePortMetricsTestBase, UpdateEmptyBufferNoCrash)
{
    mock_mctp::setNextResponse({}, {});
    const std::shared_ptr<NvidiaPciePortMetrics> metrics =
        createPciePortMetrics("Port_0", "pcie_metrics_empty");
    EXPECT_NO_THROW(metrics->update());
}

} // namespace
