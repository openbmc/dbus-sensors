/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaNVLinkPortCharacteristics.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <sdbusplus/asio/object_server.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr uint8_t defaultPortIndex = 0;

constexpr const char* portInterfaceName =
    "xyz.openbmc_project.Inventory.Connector.Port";

// Query Port Characteristics (Nvidia MCTP VDM 0x42): status, line rate, data
// rate and lane info, each a uint32_t.
std::vector<uint8_t> buildPortCharacteristicsResponse(
    uint32_t status, uint32_t lineRateMbps, uint32_t dataRateKbps,
    uint32_t laneInfo)
{
    std::vector<uint8_t> payload(4 * sizeof(uint32_t));
    PackBuffer payloadPack(payload);
    payloadPack.pack(status);
    payloadPack.pack(lineRateMbps);
    payloadPack.pack(dataRateKbps);
    payloadPack.pack(laneInfo);

    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + payload.size());
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));
    pack.pack(static_cast<uint8_t>(
        gpu::NetworkPortCommands::QueryPortCharacteristics));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(static_cast<uint16_t>(payload.size()));
    for (const uint8_t byte : payload)
    {
        pack.pack(byte);
    }
    return buf;
}

class NvidiaNVLinkPortCharacteristicsTest : public MctpMockTestBase
{
  protected:
    // The characteristics only set properties, so mirror GpuDevice and
    // publish the port object, seeded with its defaults, beforehand.
    static std::string portPath(const std::string& gpuName,
                                uint8_t portIndex = defaultPortIndex)
    {
        return "/xyz/openbmc_project/inventory/" + gpuName + "/NVLink_" +
               std::to_string(portIndex);
    }

    void makePortInterface(const std::string& gpuName,
                           uint8_t portIndex = defaultPortIndex)
    {
        portInterface = objects().add_interface(portPath(gpuName, portIndex),
                                                portInterfaceName);
        portInterface->register_property("Speed",
                                         std::numeric_limits<uint64_t>::max());
        portInterface->register_property("MaxSpeed",
                                         std::numeric_limits<uint64_t>::max());
        portInterface->register_property("Width",
                                         std::numeric_limits<size_t>::max());
        portInterface->initialize();
    }

    std::shared_ptr<NvidiaNVLinkPortCharacteristics> createCharacteristics(
        const std::string& gpuName, uint8_t eid = test_utils::defaultEid,
        uint8_t portIndex = defaultPortIndex)
    {
        makePortInterface(gpuName, portIndex);
        return std::make_shared<NvidiaNVLinkPortCharacteristics>(
            requester(), eid, portIndex, portInterface);
    }

    std::shared_ptr<sdbusplus::asio::dbus_interface> portInterface;
};

// Update — port characteristics map onto the speeds and the width

TEST_F(NvidiaNVLinkPortCharacteristicsTest, UpdateSuccessUpdatesSpeedsAndWidth)
{
    constexpr uint32_t lineRateMbps = 100000;
    constexpr uint32_t dataRateKbps = 50000000;
    constexpr uint32_t laneInfo = 0xF4; // low nibble is the width
    constexpr uint64_t expectedMaxSpeed = 100000ULL * 1000000ULL;
    constexpr uint64_t expectedSpeed = 50000000ULL * 1000ULL;

    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith(
            {}, buildPortCharacteristicsResponse(0, lineRateMbps, dataRateKbps,
                                                 laneInfo)));

    const std::string gpuName = "nvlink_char_speeds";
    const std::shared_ptr<NvidiaNVLinkPortCharacteristics> characteristics =
        createCharacteristics(gpuName);
    characteristics->update();
    const std::string path = portPath(gpuName);

    EXPECT_EQ(getProperty<uint64_t>(path, portInterfaceName, "MaxSpeed"),
              expectedMaxSpeed);
    EXPECT_EQ(getProperty<uint64_t>(path, portInterfaceName, "Speed"),
              expectedSpeed);
    EXPECT_EQ(getProperty<size_t>(path, portInterfaceName, "Width"),
              static_cast<size_t>(4));
}

// Update — request encoding verification

TEST_F(NvidiaNVLinkPortCharacteristicsTest, UpdateVerifiesRequestEncoding)
{
    // Copy the request bytes before completing the call: the reqMsg span is
    // a view into caller-owned memory, valid only during the call.
    std::vector<uint8_t> lastRequest;
    const std::vector<uint8_t> response;
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly([&](uint8_t /*eid*/, std::span<const uint8_t> reqMsg,
                            auto callback) {
            lastRequest.assign(reqMsg.begin(), reqMsg.end());
            callback(std::error_code{}, response);
        });

    const std::shared_ptr<NvidiaNVLinkPortCharacteristics> characteristics =
        createCharacteristics("nvlink_char_req_enc");
    characteristics->update();

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
                           gpu::NetworkPortCommands::QueryPortCharacteristics));

    uint8_t dataSize = 0;
    unpack.unpack(dataSize);
    EXPECT_EQ(dataSize, sizeof(uint8_t));

    // Port numbers are 1-based on the wire.
    uint8_t portNumber = 0;
    unpack.unpack(portNumber);
    EXPECT_EQ(portNumber, defaultPortIndex + 1);

    EXPECT_EQ(unpack.getError(), 0);
}

TEST_F(NvidiaNVLinkPortCharacteristicsTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaNVLinkPortCharacteristics> characteristics =
        createCharacteristics("nvlink_char_eid", testEid);
    characteristics->update();
}

// Error handling

TEST_F(NvidiaNVLinkPortCharacteristicsTest, UpdateMctpTransportErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<NvidiaNVLinkPortCharacteristics> characteristics =
        createCharacteristics("nvlink_char_mctp_err");
    EXPECT_NO_THROW(characteristics->update());
}

TEST_F(NvidiaNVLinkPortCharacteristicsTest, UpdateEmptyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaNVLinkPortCharacteristics> characteristics =
        createCharacteristics("nvlink_char_empty");
    EXPECT_NO_THROW(characteristics->update());
}

} // namespace
