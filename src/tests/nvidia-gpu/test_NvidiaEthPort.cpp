/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaEthPort.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"

#include <MessagePackUnpackUtils.hpp>

#include <cstdint>
#include <cstring>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace
{

constexpr uint8_t defaultEid = 10;
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
    for (const auto& [tag, value] : entries)
    {
        (void)tag;
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

class NvidiaEthPortTestBase : public DbusMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaEthPortMetrics> createEthPort(
        const std::string& name = "Eth_0",
        const std::string& deviceName = "ETH_DEV", uint8_t eid = defaultEid,
        uint16_t portNumber = defaultPortNumber)
    {
        const std::string path = "/test/eth/" + deviceName;
        return std::make_shared<NvidiaEthPortMetrics>(
            conn, *mctpRequester, name, deviceName, path, eid, portNumber,
            *objectServer);
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaEthPortTestBase, ConstructorCreatesPortInterface)
{
    const std::shared_ptr<NvidiaEthPortMetrics> ethPort =
        createEthPort("Eth_0", "eth_ctor");
    ASSERT_NE(ethPort, nullptr);
}

// Update — successful response updates D-Bus metric values

TEST_F(NvidiaEthPortTestBase, UpdateSuccessUpdatesMetricValues)
{
    const std::string deviceName = "eth_upd";
    // rx_bytes=1000(tag0), tx_bytes=2000(tag1)
    mock_mctp::setNextResponse(
        {}, buildEthTelemetryResponse({{0, 1000}, {1, 2000}}));
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

TEST_F(NvidiaEthPortTestBase, UpdateVerifiesRequestEncoding)
{
    mock_mctp::setNextResponse({}, {});
    const std::shared_ptr<NvidiaEthPortMetrics> ethPort =
        createEthPort("Eth_0", "eth_req_enc");
    ethPort->update();

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
    EXPECT_EQ(msgType, static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));

    uint8_t command = 0;
    unpack.unpack(command);
    EXPECT_EQ(command,
              static_cast<uint8_t>(
                  gpu::NetworkPortCommands::GetEthernetPortTelemetryCounters));

    EXPECT_EQ(unpack.getError(), 0);
}

// Update — sends request

TEST_F(NvidiaEthPortTestBase, UpdateSendsRequest)
{
    mock_mctp::setNextResponse({}, {});
    const std::shared_ptr<NvidiaEthPortMetrics> ethPort =
        createEthPort("Eth_0", "eth_sends");
    ethPort->update();
    EXPECT_GE(mock_mctp::getRequestCount(), 1U);
}

TEST_F(NvidiaEthPortTestBase, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    mock_mctp::setNextResponse({}, {});
    const std::shared_ptr<NvidiaEthPortMetrics> ethPort =
        createEthPort("Eth_0", "eth_eid", testEid);
    ethPort->update();
    EXPECT_EQ(mock_mctp::getLastEid(), testEid);
}

// Error handling

TEST_F(NvidiaEthPortTestBase, UpdateMctpTransportErrorNoCrash)
{
    mock_mctp::setNextResponse(std::make_error_code(std::errc::timed_out), {});
    const std::shared_ptr<NvidiaEthPortMetrics> ethPort =
        createEthPort("Eth_0", "eth_mctp_err");
    EXPECT_NO_THROW(ethPort->update());
}

TEST_F(NvidiaEthPortTestBase, UpdateEmptyBufferNoCrash)
{
    mock_mctp::setNextResponse({}, {});
    const std::shared_ptr<NvidiaEthPortMetrics> ethPort =
        createEthPort("Eth_0", "eth_empty");
    EXPECT_NO_THROW(ethPort->update());
}

} // namespace
