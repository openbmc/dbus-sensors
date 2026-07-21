/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaEthPort.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
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

constexpr uint16_t defaultPortNumber = 0;

// Build an aggregate TLV response for Ethernet port telemetry.
// CommonAggregateResponse header(9) = Header(5) + command(1) + cc(1) +
//   telemetryCount(2)
// Each TLV entry: tag(1) + tagInfo(1) + value(4 or 8 bytes)
std::vector<uint8_t> buildEthTelemetryResponse(
    const std::vector<std::pair<uint8_t, uint64_t>>& entries)
{
    // Compute total size: 9-byte aggregate header + TLV entries
    size_t dataSize = 0;
    for ([[maybe_unused]] const auto& [tag, value] : entries)
    {
        // Use 4-byte encoding if value fits in uint32, otherwise 8
        const size_t valueLen = (value <= 0xFFFFFFFF) ? 4 : 8;
        dataSize += 2 + valueLen; // tag + tagInfo + value
    }
    const size_t responseSize = 9 + dataSize;
    std::vector<uint8_t> buf(responseSize, 0);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));
    pack.pack(static_cast<uint8_t>(
        0x0F)); // command: GetEthernetPortTelemetryCounters
    pack.pack(static_cast<uint8_t>(0x00));            // cc: SUCCESS
    pack.pack(static_cast<uint16_t>(entries.size())); // telemetryCount

    // Pack TLV entries after the aggregate header
    size_t offset = 9;
    for (const auto& [tag, value] : entries)
    {
        const bool is32bit = (value <= 0xFFFFFFFF);
        // tagInfo: bit0=isValid(1), bits[3:1]=encodedLength, bit7=isByteLenEnc
        // For power-of-2 encoding: 4 bytes → encodedLength=2 (1<<2=4),
        //                          8 bytes → encodedLength=3 (1<<3=8)
        const uint8_t encodedLen = is32bit ? 2 : 3;
        const uint8_t tagInfo =
            static_cast<uint8_t>(0x01 | (encodedLen << 1)); // valid, pow2 enc
        buf[offset++] = tag;
        buf[offset++] = tagInfo;
        if (is32bit)
        {
            const uint32_t val32 = static_cast<uint32_t>(value);
            std::memcpy(buf.data() + offset, &val32, sizeof(val32));
            offset += sizeof(val32);
        }
        else
        {
            std::memcpy(buf.data() + offset, &value, sizeof(value));
            offset += sizeof(value);
        }
    }
    return buf;
}

class NvidiaEthPortTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaEthPortMetrics> createEthPort(
        const std::string& name = "Eth_0",
        const std::string& deviceName = "ETH_DEV",
        uint8_t eid = test_utils::defaultEid,
        uint16_t portNumber = defaultPortNumber,
        const std::vector<std::pair<uint8_t, uint64_t>>& addresses = {})
    {
        const std::string path = "/test/eth/" + deviceName;
        return std::make_shared<NvidiaEthPortMetrics>(
            bus(), requester(), name, deviceName, path, eid, portNumber,
            objects(), addresses);
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaEthPortTest, ConstructorCreatesPortInterface)
{
    const std::shared_ptr<NvidiaEthPortMetrics> ethPort =
        createEthPort("Eth_0", "eth_ctor");
    ASSERT_NE(ethPort, nullptr);
}

// Update — successful response updates D-Bus metric values

TEST_F(NvidiaEthPortTest, UpdateSuccessUpdatesMetricValues)
{
    const std::string deviceName = "eth_upd";
    // rx_bytes=1000(tag0), tx_bytes=2000(tag1)
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildEthTelemetryResponse({{0, 1000}, {1, 2000}})));

    const std::shared_ptr<NvidiaEthPortMetrics> ethPort =
        createEthPort("Eth_0", deviceName);
    ethPort->update();

    const std::string metricsBase =
        "/xyz/openbmc_project/metric/port_" + deviceName + "_Eth_0";
    EXPECT_EQ(getProperty<double>(metricsBase + "/nic/rx_bytes",
                                  "xyz.openbmc_project.Metric.Value", "Value"),
              1000.0);
    EXPECT_EQ(getProperty<double>(metricsBase + "/nic/tx_bytes",
                                  "xyz.openbmc_project.Metric.Value", "Value"),
              2000.0);
}

// Update — request encoding verification

TEST_F(NvidiaEthPortTest, UpdateVerifiesRequestEncoding)
{
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

    const std::shared_ptr<NvidiaEthPortMetrics> ethPort =
        createEthPort("Eth_0", "eth_req_enc");
    ethPort->update();

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
    EXPECT_EQ(command,
              static_cast<uint8_t>(
                  gpu::NetworkPortCommands::GetEthernetPortTelemetryCounters));

    EXPECT_EQ(unpack.getError(), 0);
}

// Update — sends request

TEST_F(NvidiaEthPortTest, UpdateSendsRequest)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaEthPortMetrics> ethPort =
        createEthPort("Eth_0", "eth_sends");
    ethPort->update();
}

TEST_F(NvidiaEthPortTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaEthPortMetrics> ethPort =
        createEthPort("Eth_0", "eth_eid", testEid);
    ethPort->update();
}

// Error handling

TEST_F(NvidiaEthPortTest, UpdateMctpTransportErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<NvidiaEthPortMetrics> ethPort =
        createEthPort("Eth_0", "eth_mctp_err");
    EXPECT_NO_THROW(ethPort->update());
}

TEST_F(NvidiaEthPortTest, UpdateEmptyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaEthPortMetrics> ethPort =
        createEthPort("Eth_0", "eth_empty");
    EXPECT_NO_THROW(ethPort->update());
}

} // namespace
