/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaNVLinkPortMetrics.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <tuple>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr uint8_t defaultPortIndex = 0;

constexpr const char* metricInterfaceName = "xyz.openbmc_project.Metric.Value";

// Counter indices of the published metrics, matching the port counter struct
// order used by Get Port Telemetry Counter.
constexpr uint8_t rxFramesIndex = 0;
constexpr uint8_t rxBytesIndex = 1;
constexpr uint8_t txBytesIndex = 9;

// Build a SUCCESS Get Port Telemetry Counter response: the supportedCounters
// bitmap followed by a dense array of 64-bit counters.
std::vector<uint8_t> buildPortTelemetryResponse(
    uint32_t supportedCounters, const std::vector<uint64_t>& counters)
{
    const size_t dataSize =
        sizeof(supportedCounters) + (counters.size() * sizeof(uint64_t));
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + dataSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));
    pack.pack(static_cast<uint8_t>(
        gpu::NetworkPortCommands::GetPortTelemetryCounter));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(static_cast<uint16_t>(dataSize));
    pack.pack(supportedCounters);
    for (const uint64_t counter : counters)
    {
        pack.pack(counter);
    }
    return buf;
}

// A full set of counters, with counter i carrying the value i * 10 so each
// metric can be told apart.
std::vector<uint64_t> countersWithDistinctValues()
{
    std::vector<uint64_t> counters(gpu::maxPortTelemetryCounters);
    for (size_t i = 0; i < counters.size(); ++i)
    {
        counters[i] = i * 10;
    }
    return counters;
}

class NvidiaNVLinkPortMetricsTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaNVLinkPortMetrics> createPortMetrics(
        const std::string& gpuName, uint8_t eid = test_utils::defaultEid,
        uint8_t portIndex = defaultPortIndex)
    {
        return makeNvidiaNVLinkPortMetrics(bus(), requester(), gpuName, eid,
                                           portIndex, objects());
    }

    static std::string metricPathFor(const std::string& gpuName,
                                     const std::string& metricName,
                                     uint8_t portIndex = defaultPortIndex)
    {
        return "/xyz/openbmc_project/metric/port_" + gpuName + "_NVLink_" +
               std::to_string(portIndex) + metricName;
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaNVLinkPortMetricsTest, ConstructorCreatesMetricInterfaces)
{
    const std::string gpuName = "nvlink_metrics_ctor";
    const std::shared_ptr<NvidiaNVLinkPortMetrics> metrics =
        createPortMetrics(gpuName);
    ASSERT_NE(metrics, nullptr);

    EXPECT_EQ(getProperty<double>(metricPathFor(gpuName, "/nvlink/rx_bytes"),
                                  metricInterfaceName, "Value"),
              0.0);
    EXPECT_EQ(getProperty<double>(metricPathFor(gpuName, "/nvlink/tx_discards"),
                                  metricInterfaceName, "Value"),
              0.0);
}

TEST_F(NvidiaNVLinkPortMetricsTest, ConstructorSetsUnitPerCounter)
{
    const std::string gpuName = "nvlink_metrics_units";
    const std::shared_ptr<NvidiaNVLinkPortMetrics> metrics =
        createPortMetrics(gpuName);

    // Byte counters are reported in bytes, packet counters as a count.
    EXPECT_EQ(
        getProperty<std::string>(metricPathFor(gpuName, "/nvlink/rx_bytes"),
                                 metricInterfaceName, "Unit"),
        "xyz.openbmc_project.Metric.Value.Unit.Bytes");
    EXPECT_EQ(
        getProperty<std::string>(metricPathFor(gpuName, "/nvlink/rx_frames"),
                                 metricInterfaceName, "Unit"),
        "xyz.openbmc_project.Metric.Value.Unit.Count");
}

TEST_F(NvidiaNVLinkPortMetricsTest, ConstructorAssociatesMetricWithPort)
{
    const std::string gpuName = "nvlink_metrics_assoc";
    const std::shared_ptr<NvidiaNVLinkPortMetrics> metrics =
        createPortMetrics(gpuName);

    using AssociationList =
        std::vector<std::tuple<std::string, std::string, std::string>>;
    const AssociationList associations = getProperty<AssociationList>(
        metricPathFor(gpuName, "/nvlink/rx_bytes"),
        "xyz.openbmc_project.Association.Definitions", "Associations");

    ASSERT_EQ(associations.size(), 1U);
    EXPECT_EQ(std::get<0>(associations[0]), "measuring");
    EXPECT_EQ(std::get<1>(associations[0]), "measured_by");
    EXPECT_EQ(std::get<2>(associations[0]),
              "/xyz/openbmc_project/inventory/" + gpuName + "/NVLink_0");
}

// Update — successful response updates D-Bus metric values

TEST_F(NvidiaNVLinkPortMetricsTest, UpdateSuccessUpdatesMetricValues)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildPortTelemetryResponse(0xFFFFFFFF,
                                           countersWithDistinctValues())));

    const std::string gpuName = "nvlink_metrics_upd";
    const std::shared_ptr<NvidiaNVLinkPortMetrics> metrics =
        createPortMetrics(gpuName);
    metrics->update();

    EXPECT_EQ(getProperty<double>(metricPathFor(gpuName, "/nvlink/rx_frames"),
                                  metricInterfaceName, "Value"),
              static_cast<double>(rxFramesIndex * 10));
    EXPECT_EQ(getProperty<double>(metricPathFor(gpuName, "/nvlink/rx_bytes"),
                                  metricInterfaceName, "Value"),
              static_cast<double>(rxBytesIndex * 10));
    EXPECT_EQ(getProperty<double>(metricPathFor(gpuName, "/nvlink/tx_bytes"),
                                  metricInterfaceName, "Value"),
              static_cast<double>(txBytesIndex * 10));
}

TEST_F(NvidiaNVLinkPortMetricsTest, UpdateSkipsCountersTheDeviceDoesNotSupport)
{
    // Everything except tx_bytes is supported.
    constexpr uint32_t supportedCounters = ~(1U << txBytesIndex);
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildPortTelemetryResponse(supportedCounters,
                                           countersWithDistinctValues())));

    const std::string gpuName = "nvlink_metrics_unsupported";
    const std::shared_ptr<NvidiaNVLinkPortMetrics> metrics =
        createPortMetrics(gpuName);
    metrics->update();

    // The supported counter is refreshed ...
    EXPECT_EQ(getProperty<double>(metricPathFor(gpuName, "/nvlink/rx_bytes"),
                                  metricInterfaceName, "Value"),
              static_cast<double>(rxBytesIndex * 10));
    // ... while the unsupported one keeps its initial value.
    EXPECT_EQ(getProperty<double>(metricPathFor(gpuName, "/nvlink/tx_bytes"),
                                  metricInterfaceName, "Value"),
              0.0);
}

// Update — request encoding verification

TEST_F(NvidiaNVLinkPortMetricsTest, UpdateVerifiesRequestEncoding)
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

    const std::shared_ptr<NvidiaNVLinkPortMetrics> metrics =
        createPortMetrics("nvlink_metrics_req_enc");
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
    EXPECT_EQ(msgType, static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));

    uint8_t command = 0;
    unpack.unpack(command);
    EXPECT_EQ(command, static_cast<uint8_t>(
                           gpu::NetworkPortCommands::GetPortTelemetryCounter));

    uint8_t dataSize = 0;
    unpack.unpack(dataSize);
    EXPECT_EQ(dataSize, sizeof(uint8_t));

    // Port numbers are 1-based on the wire.
    uint8_t portNumber = 0;
    unpack.unpack(portNumber);
    EXPECT_EQ(portNumber, defaultPortIndex + 1);

    EXPECT_EQ(unpack.getError(), 0);
}

// Update — sends request

TEST_F(NvidiaNVLinkPortMetricsTest, UpdateSendsRequest)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaNVLinkPortMetrics> metrics =
        createPortMetrics("nvlink_metrics_sends");
    metrics->update();
}

TEST_F(NvidiaNVLinkPortMetricsTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaNVLinkPortMetrics> metrics =
        createPortMetrics("nvlink_metrics_eid", testEid);
    metrics->update();
}

// Error handling

TEST_F(NvidiaNVLinkPortMetricsTest, UpdateMctpTransportErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<NvidiaNVLinkPortMetrics> metrics =
        createPortMetrics("nvlink_metrics_mctp_err");
    EXPECT_NO_THROW(metrics->update());
}

TEST_F(NvidiaNVLinkPortMetricsTest, UpdateEmptyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaNVLinkPortMetrics> metrics =
        createPortMetrics("nvlink_metrics_empty");
    EXPECT_NO_THROW(metrics->update());
}

} // namespace
