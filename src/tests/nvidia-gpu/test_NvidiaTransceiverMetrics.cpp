/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaTransceiverMetrics.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <sdbusplus/exception.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <system_error>
#include <tuple>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

using Associations =
    std::vector<std::tuple<std::string, std::string, std::string>>;

constexpr uint16_t defaultPortNumber = 1;

const std::string metricInterface = "xyz.openbmc_project.Metric.Value";

std::string portPathFor(const std::string& deviceName)
{
    return "/xyz/openbmc_project/inventory/" + deviceName + "/Port_1";
}

std::string transceiverPathFor(const std::string& deviceName)
{
    return portPathFor(deviceName) + "/Transceiver";
}

std::string metricPathFor(const std::string& deviceName,
                          const std::string& metric)
{
    return "/xyz/openbmc_project/metric/transceiver_" + deviceName +
           "_Port_1/" + metric;
}

// A record carrying a valid two-byte lane reading.
std::vector<uint8_t> laneRecord(uint8_t tag, uint16_t value)
{
    return {tag, 0x03, static_cast<uint8_t>(value & 0xff),
            static_cast<uint8_t>(value >> 8)};
}

std::vector<uint8_t> invalidLaneRecord(uint8_t tag)
{
    return {tag, 0x02, 0xff, 0xff};
}

std::vector<uint8_t> scalingFactorRecord(uint8_t index)
{
    return {0x20, 0x01, index};
}

std::vector<uint8_t> tokenRecord(uint32_t token)
{
    return {0xfd,
            0x05,
            static_cast<uint8_t>(token & 0xff),
            static_cast<uint8_t>((token >> 8) & 0xff),
            static_cast<uint8_t>((token >> 16) & 0xff),
            static_cast<uint8_t>(token >> 24)};
}

// A Query Port Telemetry v2 page holding the given records.
std::vector<uint8_t> buildPage(const std::vector<std::vector<uint8_t>>& records)
{
    // Header(5) + command(1) + cc(1) + telemetryCount(2)
    std::vector<uint8_t> buf(
        ocp::accelerator_management::messageHeaderSize + 4);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));
    pack.pack(
        static_cast<uint8_t>(gpu::NetworkPortCommands::QueryPortTelemetryV2));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(records.size()));
    for (const std::vector<uint8_t>& record : records)
    {
        buf.insert(buf.end(), record.begin(), record.end());
    }
    return buf;
}

std::vector<uint8_t> buildErrorResponse(
    ocp::accelerator_management::CompletionCode cc)
{
    std::vector<uint8_t> buf(test_utils::errorResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));
    pack.pack(
        static_cast<uint8_t>(gpu::NetworkPortCommands::QueryPortTelemetryV2));
    pack.pack(static_cast<uint8_t>(cc));
    pack.pack(static_cast<uint16_t>(0)); // reasonCode
    return buf;
}

// Captured from a ConnectX-9 port with a 4-lane optical module: TX and RX
// power and bias current for lanes 0-7, SNR for lanes 0-7, a 2x bias scaling
// factor and the last-page token.
const std::vector<uint8_t> capturedPage = {
    0x10, 0xde, 0x00, 0x89, 0x01, 0x14, 0x00, 0x22, 0x00, 0x00, 0x03, 0x3c,
    0x06, 0x01, 0x03, 0x3e, 0x06, 0x02, 0x03, 0x3c, 0x06, 0x03, 0x03, 0x1e,
    0x06, 0x04, 0x03, 0x00, 0x00, 0x05, 0x03, 0x00, 0x00, 0x06, 0x03, 0x00,
    0x00, 0x07, 0x03, 0x00, 0x00, 0x08, 0x03, 0x3e, 0x07, 0x09, 0x03, 0xa3,
    0x06, 0x0a, 0x03, 0xf8, 0x06, 0x0b, 0x03, 0xa2, 0x06, 0x0c, 0x03, 0x00,
    0x00, 0x0d, 0x03, 0x00, 0x00, 0x0e, 0x03, 0x00, 0x00, 0x0f, 0x03, 0x00,
    0x00, 0x10, 0x03, 0xd5, 0x24, 0x11, 0x03, 0x31, 0x24, 0x12, 0x03, 0x95,
    0x23, 0x13, 0x03, 0xba, 0x23, 0x14, 0x03, 0x00, 0x00, 0x15, 0x03, 0x00,
    0x00, 0x16, 0x03, 0x00, 0x00, 0x17, 0x03, 0x00, 0x00, 0x18, 0x05, 0xc9,
    0x16, 0x00, 0x00, 0x19, 0x05, 0x6a, 0x15, 0x00, 0x00, 0x1a, 0x05, 0xc8,
    0x15, 0x00, 0x00, 0x1b, 0x05, 0x79, 0x16, 0x00, 0x00, 0x1c, 0x05, 0x00,
    0x00, 0x00, 0x00, 0x1d, 0x05, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x05, 0x00,
    0x00, 0x00, 0x00, 0x1f, 0x05, 0x00, 0x00, 0x00, 0x00, 0x20, 0x01, 0x01,
    0xfd, 0x05, 0x00, 0x00, 0x00, 0x00};

using mock_mctp::PendingRequest;

class NvidiaTransceiverMetricsTest : public MctpMockTestBase
{
  protected:
    void SetUp() override
    {
        MctpMockTestBase::SetUp();
        if (testing::Test::IsSkipped())
        {
            return;
        }

        ON_CALL(mctpMock, sendRecvMsg)
            .WillByDefault(mock_mctp::deferInto(pending));
    }

    static std::shared_ptr<NvidiaTransceiverMetrics> createTransceiver(
        const std::string& deviceName)
    {
        return std::make_shared<NvidiaTransceiverMetrics>(
            objects(), requester(), deviceName, "Port_1",
            sdbusplus::object_path(transceiverPathFor(deviceName)),
            test_utils::defaultEid, defaultPortNumber);
    }

    // Answers the oldest request still waiting for its response.
    void respond(const std::vector<uint8_t>& response,
                 const std::error_code& ec = {})
    {
        ASSERT_FALSE(pending.empty());
        PendingRequest next = std::move(pending.front());
        pending.pop_front();
        next.callback(ec, response);
    }

    static double metricValue(const std::string& deviceName,
                              const std::string& metric)
    {
        return getProperty<double>(metricPathFor(deviceName, metric),
                                   metricInterface, "Value");
    }

    std::deque<PendingRequest> pending;
};

// Constructor

TEST_F(NvidiaTransceiverMetricsTest, ConstructorPublishesLaneMetrics)
{
    const std::string deviceName = "xcvr_metrics";
    const std::shared_ptr<NvidiaTransceiverMetrics> transceiver =
        createTransceiver(deviceName);

    for (size_t lane = 0; lane < gpu::opticalModuleLaneCount; ++lane)
    {
        const std::string suffix = std::to_string(lane);
        for (const auto& [metric, unit] :
             {std::pair{"tx_output_power_lane_", "Watts"},
              std::pair{"rx_input_power_lane_", "Watts"},
              std::pair{"tx_bias_current_lane_", "Amperes"}})
        {
            const std::string path = metricPathFor(deviceName, metric + suffix);

            EXPECT_EQ(getProperty<std::string>(path, metricInterface, "Unit"),
                      std::string("xyz.openbmc_project.Metric.Value.Unit.") +
                          unit);
            EXPECT_TRUE(std::isnan(
                getProperty<double>(path, metricInterface, "Value")));

            const Associations associations = getProperty<Associations>(
                path, "xyz.openbmc_project.Association.Definitions",
                "Associations");
            ASSERT_EQ(associations.size(), 1U);
            EXPECT_EQ(associations[0],
                      std::make_tuple(std::string("measuring"),
                                      std::string("measured_by"),
                                      transceiverPathFor(deviceName)));
        }
    }
}

// Update

TEST_F(NvidiaTransceiverMetricsTest, UpdateRequestsOpticalModuleGroup)
{
    const std::shared_ptr<NvidiaTransceiverMetrics> transceiver =
        createTransceiver("xcvr_request");
    transceiver->update();

    ASSERT_EQ(pending.size(), 1U);
    const std::vector<uint8_t> expected = {
        0x10, 0xde, 0x80, 0x89, 0x01, 0x14, 0x0c, 0x00, 0x00, 0x00,
        0x00, 0x01, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00};
    EXPECT_EQ(pending.front().request, expected);
}

TEST_F(NvidiaTransceiverMetricsTest, UpdatePublishesCapturedReadings)
{
    const std::string deviceName = "xcvr_captured";
    const std::shared_ptr<NvidiaTransceiverMetrics> transceiver =
        createTransceiver(deviceName);
    transceiver->update();
    respond(capturedPage);

    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_output_power_lane_0"),
                     1596 / 1e6);
    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_output_power_lane_3"),
                     1566 / 1e6);
    EXPECT_DOUBLE_EQ(metricValue(deviceName, "rx_input_power_lane_0"),
                     1854 / 1e6);
    EXPECT_DOUBLE_EQ(metricValue(deviceName, "rx_input_power_lane_3"),
                     1698 / 1e6);
    // 2 uA steps, doubled by the reported scaling factor
    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_bias_current_lane_0"),
                     9429 * 4 / 1e6);
    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_bias_current_lane_3"),
                     9146 * 4 / 1e6);
    // The device reports the lanes the module does not have as zero
    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_output_power_lane_7"), 0.0);
    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_bias_current_lane_7"), 0.0);
}

TEST_F(NvidiaTransceiverMetricsTest, UpdateReadsBiasAtOneTimesWithoutFactor)
{
    const std::string deviceName = "xcvr_no_factor";
    const std::shared_ptr<NvidiaTransceiverMetrics> transceiver =
        createTransceiver(deviceName);
    transceiver->update();
    respond(buildPage({laneRecord(0x10, 5000), tokenRecord(0)}));

    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_bias_current_lane_0"),
                     5000 * 2 / 1e6);
}

TEST_F(NvidiaTransceiverMetricsTest, UpdateAppliesFourTimesFactor)
{
    const std::string deviceName = "xcvr_four_times";
    const std::shared_ptr<NvidiaTransceiverMetrics> transceiver =
        createTransceiver(deviceName);
    transceiver->update();
    respond(buildPage(
        {laneRecord(0x10, 5000), scalingFactorRecord(2), tokenRecord(0)}));

    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_bias_current_lane_0"),
                     5000 * 8 / 1e6);
}

TEST_F(NvidiaTransceiverMetricsTest, UpdateRescalesBiasWhenOnlyFactorChanges)
{
    const std::string deviceName = "xcvr_factor_change";
    const std::shared_ptr<NvidiaTransceiverMetrics> transceiver =
        createTransceiver(deviceName);
    transceiver->update();
    respond(buildPage({laneRecord(0x10, 5000), tokenRecord(0)}));
    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_bias_current_lane_0"),
                     5000 * 2 / 1e6);

    transceiver->update();
    respond(buildPage(
        {laneRecord(0x10, 5000), scalingFactorRecord(1), tokenRecord(0)}));

    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_bias_current_lane_0"),
                     5000 * 4 / 1e6);
}

TEST_F(NvidiaTransceiverMetricsTest, UpdateLeavesBiasUnavailableForBadFactor)
{
    const std::string deviceName = "xcvr_bad_factor";
    const std::shared_ptr<NvidiaTransceiverMetrics> transceiver =
        createTransceiver(deviceName);
    transceiver->update();
    respond(buildPage({laneRecord(0x00, 1500), laneRecord(0x10, 5000),
                       scalingFactorRecord(3), tokenRecord(0)}));

    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_output_power_lane_0"),
                     1500 / 1e6);
    EXPECT_TRUE(std::isnan(metricValue(deviceName, "tx_bias_current_lane_0")));
}

TEST_F(NvidiaTransceiverMetricsTest, UpdateLeavesUnreadLanesUnavailable)
{
    const std::string deviceName = "xcvr_unread";
    const std::shared_ptr<NvidiaTransceiverMetrics> transceiver =
        createTransceiver(deviceName);
    transceiver->update();
    respond(buildPage({laneRecord(0x00, 1500), invalidLaneRecord(0x01),
                       laneRecord(0x08, 1700), tokenRecord(0)}));

    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_output_power_lane_0"),
                     1500 / 1e6);
    EXPECT_TRUE(std::isnan(metricValue(deviceName, "tx_output_power_lane_1")));
    EXPECT_DOUBLE_EQ(metricValue(deviceName, "rx_input_power_lane_0"),
                     1700 / 1e6);
    EXPECT_TRUE(std::isnan(metricValue(deviceName, "rx_input_power_lane_1")));
    EXPECT_TRUE(std::isnan(metricValue(deviceName, "tx_bias_current_lane_0")));
}

TEST_F(NvidiaTransceiverMetricsTest, UpdateFollowsSequenceToken)
{
    const std::string deviceName = "xcvr_pages";
    const std::shared_ptr<NvidiaTransceiverMetrics> transceiver =
        createTransceiver(deviceName);
    transceiver->update();
    respond(buildPage({laneRecord(0x00, 1500), tokenRecord(0x11223344)}));

    ASSERT_EQ(pending.size(), 1U);
    const std::vector<uint8_t> expected = {
        0x10, 0xde, 0x80, 0x89, 0x01, 0x14, 0x0c, 0x44, 0x33, 0x22,
        0x11, 0x01, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00};
    EXPECT_EQ(pending.front().request, expected);

    // Nothing is published until the last page arrives
    EXPECT_TRUE(std::isnan(metricValue(deviceName, "tx_output_power_lane_0")));

    respond(buildPage({laneRecord(0x08, 1700), laneRecord(0x10, 5000),
                       scalingFactorRecord(1), tokenRecord(0)}));

    EXPECT_TRUE(pending.empty());
    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_output_power_lane_0"),
                     1500 / 1e6);
    EXPECT_DOUBLE_EQ(metricValue(deviceName, "rx_input_power_lane_0"),
                     1700 / 1e6);
    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_bias_current_lane_0"),
                     5000 * 4 / 1e6);
}

TEST_F(NvidiaTransceiverMetricsTest, UpdateKeepsFactorFromEarlierPage)
{
    const std::string deviceName = "xcvr_factor_page";
    const std::shared_ptr<NvidiaTransceiverMetrics> transceiver =
        createTransceiver(deviceName);
    transceiver->update();
    respond(buildPage({scalingFactorRecord(1), tokenRecord(7)}));
    respond(buildPage({laneRecord(0x10, 5000), tokenRecord(0)}));

    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_bias_current_lane_0"),
                     5000 * 4 / 1e6);
}

TEST_F(NvidiaTransceiverMetricsTest, CompletionCodeMakesReadingsUnavailable)
{
    const std::string deviceName = "xcvr_cc";
    const std::shared_ptr<NvidiaTransceiverMetrics> transceiver =
        createTransceiver(deviceName);
    transceiver->update();
    respond(capturedPage);
    ASSERT_FALSE(std::isnan(metricValue(deviceName, "tx_output_power_lane_0")));

    transceiver->update();
    respond(buildErrorResponse(
        ocp::accelerator_management::CompletionCode::ERR_INVALID_DATA));

    for (size_t lane = 0; lane < gpu::opticalModuleLaneCount; ++lane)
    {
        const std::string suffix = std::to_string(lane);
        EXPECT_TRUE(std::isnan(
            metricValue(deviceName, "tx_output_power_lane_" + suffix)));
        EXPECT_TRUE(std::isnan(
            metricValue(deviceName, "rx_input_power_lane_" + suffix)));
        EXPECT_TRUE(std::isnan(
            metricValue(deviceName, "tx_bias_current_lane_" + suffix)));
    }

    transceiver->update();
    respond(capturedPage);
    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_output_power_lane_0"),
                     1596 / 1e6);
}

TEST_F(NvidiaTransceiverMetricsTest, CompletionCodeOnLaterPageDiscardsTheRead)
{
    const std::string deviceName = "xcvr_cc_page";
    const std::shared_ptr<NvidiaTransceiverMetrics> transceiver =
        createTransceiver(deviceName);
    transceiver->update();
    respond(capturedPage);

    transceiver->update();
    respond(buildPage(
        {laneRecord(0x00, 1500), laneRecord(0x10, 5000), tokenRecord(7)}));
    respond(
        buildErrorResponse(ocp::accelerator_management::CompletionCode::ERROR));

    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_output_power_lane_0"),
                     1596 / 1e6);
    EXPECT_DOUBLE_EQ(metricValue(deviceName, "rx_input_power_lane_0"),
                     1854 / 1e6);
    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_bias_current_lane_0"),
                     9429 * 4 / 1e6);

    // The failed read does not hold up the next one
    transceiver->update();
    EXPECT_EQ(pending.size(), 1U);
}

TEST_F(NvidiaTransceiverMetricsTest, MissingResponseKeepsReadings)
{
    const std::string deviceName = "xcvr_timeout";
    const std::shared_ptr<NvidiaTransceiverMetrics> transceiver =
        createTransceiver(deviceName);
    transceiver->update();
    respond(capturedPage);

    transceiver->update();
    respond({}, std::make_error_code(std::errc::timed_out));

    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_output_power_lane_0"),
                     1596 / 1e6);

    // The failed read does not hold up the next one
    transceiver->update();
    EXPECT_EQ(pending.size(), 1U);
}

TEST_F(NvidiaTransceiverMetricsTest, MissingLaterPageDiscardsTheRead)
{
    const std::string deviceName = "xcvr_timeout_page";
    const std::shared_ptr<NvidiaTransceiverMetrics> transceiver =
        createTransceiver(deviceName);
    transceiver->update();
    respond(capturedPage);

    transceiver->update();
    respond(buildPage({laneRecord(0x00, 1500), tokenRecord(7)}));
    respond({}, std::make_error_code(std::errc::timed_out));

    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_output_power_lane_0"),
                     1596 / 1e6);
    EXPECT_DOUBLE_EQ(metricValue(deviceName, "rx_input_power_lane_0"),
                     1854 / 1e6);
}

TEST_F(NvidiaTransceiverMetricsTest, MalformedResponseKeepsReadings)
{
    const std::string deviceName = "xcvr_malformed";
    const std::shared_ptr<NvidiaTransceiverMetrics> transceiver =
        createTransceiver(deviceName);
    transceiver->update();
    respond(capturedPage);

    transceiver->update();
    respond(buildPage({{0x00, 0x03, 0x3c}}));

    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_output_power_lane_0"),
                     1596 / 1e6);

    // The failed read does not hold up the next one
    transceiver->update();
    EXPECT_EQ(pending.size(), 1U);
}

TEST_F(NvidiaTransceiverMetricsTest, UpdateStopsFollowingEndlessTokens)
{
    const std::string deviceName = "xcvr_endless";
    const std::shared_ptr<NvidiaTransceiverMetrics> transceiver =
        createTransceiver(deviceName);
    transceiver->update();
    respond(capturedPage);

    transceiver->update();
    size_t pages = 0;
    while (!pending.empty() && pages < 100)
    {
        ++pages;
        respond(buildPage({laneRecord(0x00, 1500), tokenRecord(7)}));
    }

    EXPECT_EQ(pages, 34U);
    EXPECT_DOUBLE_EQ(metricValue(deviceName, "tx_output_power_lane_0"),
                     1596 / 1e6);

    // The next read gets the same number of pages
    transceiver->update();
    pages = 0;
    while (!pending.empty() && pages < 100)
    {
        ++pages;
        respond(buildPage({laneRecord(0x00, 1500), tokenRecord(7)}));
    }

    EXPECT_EQ(pages, 34U);
}

TEST_F(NvidiaTransceiverMetricsTest, UpdateWaitsForReadInProgress)
{
    const std::shared_ptr<NvidiaTransceiverMetrics> transceiver =
        createTransceiver("xcvr_inflight");
    transceiver->update();
    transceiver->update();
    EXPECT_EQ(pending.size(), 1U);

    respond(capturedPage);
    transceiver->update();
    EXPECT_EQ(pending.size(), 1U);
}

TEST_F(NvidiaTransceiverMetricsTest, ResponseAfterDestructionIsIgnored)
{
    std::shared_ptr<NvidiaTransceiverMetrics> transceiver =
        createTransceiver("xcvr_late");
    transceiver->update();
    transceiver.reset();

    respond(buildPage({laneRecord(0x00, 1500), tokenRecord(7)}));
    EXPECT_TRUE(pending.empty());
}

// Destructor

TEST_F(NvidiaTransceiverMetricsTest, DestructorRemovesInterfaces)
{
    const std::string deviceName = "xcvr_dtor";
    {
        const std::shared_ptr<NvidiaTransceiverMetrics> transceiver =
            createTransceiver(deviceName);
        EXPECT_NO_THROW(metricValue(deviceName, "tx_output_power_lane_0"));
    }
    drainPendingAsync();

    for (const char* metric :
         {"tx_output_power_lane_0", "rx_input_power_lane_0",
          "tx_bias_current_lane_7"})
    {
        EXPECT_THROW(getProperty<double>(metricPathFor(deviceName, metric),
                                         metricInterface, "Value"),
                     sdbusplus::exception_t);
        EXPECT_THROW(getProperty<Associations>(
                         metricPathFor(deviceName, metric),
                         "xyz.openbmc_project.Association.Definitions",
                         "Associations"),
                     sdbusplus::exception_t);
    }
}

} // namespace
