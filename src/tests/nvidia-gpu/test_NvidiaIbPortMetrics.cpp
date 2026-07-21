/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaIbPortMetrics.hpp"
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

// Ports are numbered from one, as GetPortTelemetryCounters requires.
constexpr uint16_t defaultPortNumber = 1;

std::string metricPathFor(const std::string& deviceName,
                          const std::string& name, const std::string& metric)
{
    return "/xyz/openbmc_project/metric/port_" + deviceName + "_" + name +
           "/nic/" + metric;
}

// Build a GetPortTelemetryCounters success response. The payload is
// dataSize(2) + supportedCounters(4) + one 64-bit value per counter slot,
// with bit N of supportedCounters marking slot N as carrying a value.
std::vector<uint8_t> buildIbTelemetryResponse(
    uint32_t supportedCounters, const std::vector<uint64_t>& counters)
{
    const uint16_t dataSize = static_cast<uint16_t>(
        sizeof(supportedCounters) + (counters.size() * sizeof(uint64_t)));
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + dataSize, 0);

    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));
    pack.pack(static_cast<uint8_t>(
        gpu::NetworkPortCommands::GetPortTelemetryCounters));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(dataSize);
    pack.pack(supportedCounters);
    for (const uint64_t value : counters)
    {
        pack.pack(value);
    }

    return buf;
}

class NvidiaIbPortMetricsTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaIbPortMetrics> createIbPortMetrics(
        const std::string& deviceName, const std::string& name = "Port_1",
        uint8_t eid = test_utils::defaultEid,
        uint16_t portNumber = defaultPortNumber)
    {
        return std::make_shared<NvidiaIbPortMetrics>(
            requester(), name, deviceName, eid, portNumber, objects());
    }
};

TEST_F(NvidiaIbPortMetricsTest, UpdateScalesDataCountersToBytes)
{
    const std::string deviceName = "ib_scale";
    // Counters 0 (port_rcv_pkts), 1 (port_rcv_data), 7 (port_xmit_pkts) and
    // 9 (port_xmit_data) are supported.
    constexpr uint32_t supported =
        (1U << 0) | (1U << 1) | (1U << 7) | (1U << 9);
    const std::vector<uint64_t> counters{7, 1000, 0, 0, 0, 0, 0, 9, 0, 2000};

    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildIbTelemetryResponse(supported, counters)));

    const std::shared_ptr<NvidiaIbPortMetrics> metrics =
        createIbPortMetrics(deviceName);
    metrics->update();

    // The data counters carry octets divided by four, so they are published
    // as four times the reported value; the packet counters are not scaled.
    EXPECT_EQ(
        getProperty<double>(metricPathFor(deviceName, "Port_1", "rx_bytes"),
                            "xyz.openbmc_project.Metric.Value", "Value"),
        4000.0);
    EXPECT_EQ(
        getProperty<double>(metricPathFor(deviceName, "Port_1", "tx_bytes"),
                            "xyz.openbmc_project.Metric.Value", "Value"),
        8000.0);
    EXPECT_EQ(
        getProperty<double>(metricPathFor(deviceName, "Port_1", "rx_frames"),
                            "xyz.openbmc_project.Metric.Value", "Value"),
        7.0);
    EXPECT_EQ(
        getProperty<double>(metricPathFor(deviceName, "Port_1", "tx_frames"),
                            "xyz.openbmc_project.Metric.Value", "Value"),
        9.0);
}

TEST_F(NvidiaIbPortMetricsTest, UpdateLeavesUnsupportedCounterUntouched)
{
    const std::string deviceName = "ib_unsupported";
    constexpr uint32_t supported = (1U << 0);
    const std::vector<uint64_t> counters{5, 1000};

    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildIbTelemetryResponse(supported, counters)));

    const std::shared_ptr<NvidiaIbPortMetrics> metrics =
        createIbPortMetrics(deviceName);
    metrics->update();

    EXPECT_EQ(
        getProperty<double>(metricPathFor(deviceName, "Port_1", "rx_frames"),
                            "xyz.openbmc_project.Metric.Value", "Value"),
        5.0);
    EXPECT_EQ(
        getProperty<double>(metricPathFor(deviceName, "Port_1", "rx_bytes"),
                            "xyz.openbmc_project.Metric.Value", "Value"),
        0.0);
}

TEST_F(NvidiaIbPortMetricsTest, MetricUnits)
{
    const std::string deviceName = "ib_units";
    const std::shared_ptr<NvidiaIbPortMetrics> metrics =
        createIbPortMetrics(deviceName);
    ASSERT_NE(metrics, nullptr);

    EXPECT_EQ(getProperty<std::string>(
                  metricPathFor(deviceName, "Port_1", "rx_bytes"),
                  "xyz.openbmc_project.Metric.Value", "Unit"),
              "xyz.openbmc_project.Metric.Value.Unit.Bytes");
    EXPECT_EQ(getProperty<std::string>(
                  metricPathFor(deviceName, "Port_1", "rx_frames"),
                  "xyz.openbmc_project.Metric.Value", "Unit"),
              "xyz.openbmc_project.Metric.Value.Unit.Count");
}

TEST_F(NvidiaIbPortMetricsTest, UpdateVerifiesRequestEncoding)
{
    constexpr uint16_t requestedPort = 2;
    // Copy the request bytes before completing the call: the reqMsg span is
    // a view into caller-owned memory, valid only during the call.
    std::vector<uint8_t> lastRequest;
    const std::vector<uint8_t> response{};
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce([&](uint8_t /*eid*/, std::span<const uint8_t> reqMsg,
                      auto callback) {
            lastRequest.assign(reqMsg.begin(), reqMsg.end());
            callback(std::error_code{}, response);
        });

    const std::shared_ptr<NvidiaIbPortMetrics> metrics = createIbPortMetrics(
        "ib_req_enc", "Port_2", test_utils::defaultEid, requestedPort);
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
                           gpu::NetworkPortCommands::GetPortTelemetryCounters));

    // The port number is a single byte here, unlike the Ethernet command.
    uint8_t dataSize = 0;
    unpack.unpack(dataSize);
    EXPECT_EQ(dataSize, 1);

    uint8_t portNumber = 0;
    unpack.unpack(portNumber);
    EXPECT_EQ(portNumber, requestedPort);

    EXPECT_EQ(unpack.getError(), 0);
}

TEST_F(NvidiaIbPortMetricsTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaIbPortMetrics> metrics =
        createIbPortMetrics("ib_eid", "Port_1", testEid);
    metrics->update();
}

TEST_F(NvidiaIbPortMetricsTest, UpdateMctpTransportErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<NvidiaIbPortMetrics> metrics =
        createIbPortMetrics("ib_mctp_err");
    EXPECT_NO_THROW(metrics->update());
}

TEST_F(NvidiaIbPortMetricsTest, UpdateEmptyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaIbPortMetrics> metrics =
        createIbPortMetrics("ib_empty");
    EXPECT_NO_THROW(metrics->update());
}

TEST_F(NvidiaIbPortMetricsTest, DestructorRemovesInterfaces)
{
    const std::string deviceName = "ib_metrics_dtor";
    const std::string metric = metricPathFor(deviceName, "Port_1", "rx_bytes");
    {
        const std::shared_ptr<NvidiaIbPortMetrics> metrics =
            createIbPortMetrics(deviceName);
        ASSERT_NE(metrics, nullptr);
        EXPECT_NO_THROW(getProperty<double>(
            metric, "xyz.openbmc_project.Metric.Value", "Value"));
    }
    drainPendingAsync();

    EXPECT_THROW(getProperty<double>(metric, "xyz.openbmc_project.Metric.Value",
                                     "Value"),
                 sdbusplus::exception_t);
}

} // namespace
