/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaNVLinkPort.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <cstddef>
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

constexpr uint8_t defaultPortIndex = 0;

constexpr const char* portInterface =
    "xyz.openbmc_project.Inventory.Connector.Port";

// Port state values from Query Port Status (Nvidia MCTP VDM 0x43).
constexpr uint8_t portStateDown = 1;
constexpr uint8_t portStateUp = 2;
constexpr uint8_t portStateReserved = 3;
constexpr uint8_t portStatePolling = 6;
constexpr uint8_t portStateTraining = 7;

// Port status values from Query Port Status (Nvidia MCTP VDM 0x43).
constexpr uint8_t portStatusDisabled = 1;
constexpr uint8_t portStatusEnabled = 2;

// Build a SUCCESS NETWORK_PORT response with the supplied payload appended
// after the common response header.
std::vector<uint8_t> buildNetworkPortResponse(
    gpu::NetworkPortCommands command, const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + payload.size());
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));
    pack.pack(static_cast<uint8_t>(command));
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

// Query Port Status (Nvidia MCTP VDM 0x43): portState + portStatus.
std::vector<uint8_t> buildPortStatusResponse(uint8_t portState,
                                             uint8_t portStatus)
{
    return buildNetworkPortResponse(gpu::NetworkPortCommands::QueryPortStatus,
                                    {portState, portStatus});
}

// Query Port Characteristics (Nvidia MCTP VDM 0x42): status, line rate, data
// rate and lane info, each a uint32_t.
std::vector<uint8_t> buildPortCharacteristicsResponse(
    uint32_t status, uint32_t lineRateMbps, uint32_t dataRateKbps,
    uint32_t laneInfo)
{
    std::vector<uint8_t> payload(4 * sizeof(uint32_t));
    PackBuffer pack(payload);
    pack.pack(status);
    pack.pack(lineRateMbps);
    pack.pack(dataRateKbps);
    pack.pack(laneInfo);
    return buildNetworkPortResponse(
        gpu::NetworkPortCommands::QueryPortCharacteristics, payload);
}

// Return the command byte of an encoded request.
uint8_t requestCommand(std::span<const uint8_t> request)
{
    UnpackBuffer unpack(request);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    ocp::accelerator_management::unpackHeader(unpack, gpu::nvidiaPciVendorId,
                                              ocpMsgType, instanceId, msgType);
    uint8_t command = 0;
    unpack.unpack(command);
    return command;
}

class NvidiaNVLinkPortTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaNVLinkPort> createPort(
        const std::string& gpuName, uint8_t eid = test_utils::defaultEid,
        uint8_t portIndex = defaultPortIndex)
    {
        return std::make_shared<NvidiaNVLinkPort>(bus(), requester(), gpuName,
                                                  eid, portIndex, objects());
    }

    static std::string portPath(const std::string& gpuName,
                                uint8_t portIndex = defaultPortIndex)
    {
        return "/xyz/openbmc_project/inventory/" + gpuName + "/NVLink_" +
               std::to_string(portIndex);
    }

    // Answer whichever of the two polled commands the port asks for, so the
    // test does not depend on the order they are issued in.
    void expectPolledResponses(const std::vector<uint8_t>& statusResponse,
                               const std::vector<uint8_t>& characteristics)
    {
        EXPECT_CALL(mctpMock, sendRecvMsg)
            .Times(testing::AnyNumber())
            .WillRepeatedly([statusResponse,
                             characteristics](uint8_t /*eid*/,
                                              std::span<const uint8_t> reqMsg,
                                              auto callback) {
                const uint8_t command = requestCommand(reqMsg);
                if (command ==
                    static_cast<uint8_t>(
                        gpu::NetworkPortCommands::QueryPortCharacteristics))
                {
                    callback(std::error_code{}, characteristics);
                    return;
                }
                callback(std::error_code{}, statusResponse);
            });
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaNVLinkPortTest, ConstructorCreatesPortInterface)
{
    const std::string gpuName = "nvlink_port_ctor";
    const std::shared_ptr<NvidiaNVLinkPort> port = createPort(gpuName);
    const std::string path = portPath(gpuName);

    EXPECT_EQ(getProperty<std::string>(path, portInterface, "PortProtocol"),
              "xyz.openbmc_project.Inventory.Connector.Port.PortProtocol."
              "NVLink");
    EXPECT_EQ(getProperty<std::string>(path, portInterface, "PortType"),
              "xyz.openbmc_project.Inventory.Connector.Port.PortType."
              "Bidirectional");
}

TEST_F(NvidiaNVLinkPortTest, ConstructorSeedsLinkStateAndStatus)
{
    const std::string gpuName = "nvlink_port_defaults";
    const std::shared_ptr<NvidiaNVLinkPort> port = createPort(gpuName);
    const std::string path = portPath(gpuName);

    EXPECT_EQ(getProperty<std::string>(path, portInterface, "LinkStatus"),
              "xyz.openbmc_project.Inventory.Connector.Port.LinkStatus.NoLink");
    EXPECT_EQ(getProperty<std::string>(path, portInterface, "LinkState"),
              "xyz.openbmc_project.Inventory.Connector.Port.LinkState.Unknown");
}

// Update — port status maps onto LinkStatus / LinkState

TEST_F(NvidiaNVLinkPortTest, UpdateSuccessUpdatesLinkStatusAndState)
{
    expectPolledResponses(
        buildPortStatusResponse(portStateUp, portStatusEnabled), {});

    const std::string gpuName = "nvlink_port_up";
    const std::shared_ptr<NvidiaNVLinkPort> port = createPort(gpuName);
    port->update();
    const std::string path = portPath(gpuName);

    EXPECT_EQ(getProperty<std::string>(path, portInterface, "LinkStatus"),
              "xyz.openbmc_project.Inventory.Connector.Port.LinkStatus.LinkUp");
    EXPECT_EQ(getProperty<std::string>(path, portInterface, "LinkState"),
              "xyz.openbmc_project.Inventory.Connector.Port.LinkState.Enabled");
}

TEST_F(NvidiaNVLinkPortTest, UpdateMapsDownPortStateToLinkDown)
{
    expectPolledResponses(
        buildPortStatusResponse(portStateDown, portStatusDisabled), {});

    const std::string gpuName = "nvlink_port_down";
    const std::shared_ptr<NvidiaNVLinkPort> port = createPort(gpuName);
    port->update();
    const std::string path = portPath(gpuName);

    EXPECT_EQ(
        getProperty<std::string>(path, portInterface, "LinkStatus"),
        "xyz.openbmc_project.Inventory.Connector.Port.LinkStatus.LinkDown");
    EXPECT_EQ(
        getProperty<std::string>(path, portInterface, "LinkState"),
        "xyz.openbmc_project.Inventory.Connector.Port.LinkState.Disabled");
}

TEST_F(NvidiaNVLinkPortTest, UpdateMapsTrainingPortStateToTraining)
{
    expectPolledResponses(
        buildPortStatusResponse(portStateTraining, portStatusEnabled), {});

    const std::string gpuName = "nvlink_port_training";
    const std::shared_ptr<NvidiaNVLinkPort> port = createPort(gpuName);
    port->update();

    EXPECT_EQ(
        getProperty<std::string>(portPath(gpuName), portInterface,
                                 "LinkStatus"),
        "xyz.openbmc_project.Inventory.Connector.Port.LinkStatus.Training");
}

TEST_F(NvidiaNVLinkPortTest, UpdateMapsPollingPortStateToStarting)
{
    expectPolledResponses(
        buildPortStatusResponse(portStatePolling, portStatusEnabled), {});

    const std::string gpuName = "nvlink_port_polling";
    const std::shared_ptr<NvidiaNVLinkPort> port = createPort(gpuName);
    port->update();

    EXPECT_EQ(
        getProperty<std::string>(portPath(gpuName), portInterface,
                                 "LinkStatus"),
        "xyz.openbmc_project.Inventory.Connector.Port.LinkStatus.Starting");
}

TEST_F(NvidiaNVLinkPortTest, UpdateMapsUnknownPortStateToNoLink)
{
    expectPolledResponses(
        buildPortStatusResponse(portStateReserved, portStatusEnabled), {});

    const std::string gpuName = "nvlink_port_nolink";
    const std::shared_ptr<NvidiaNVLinkPort> port = createPort(gpuName);
    port->update();

    EXPECT_EQ(getProperty<std::string>(portPath(gpuName), portInterface,
                                       "LinkStatus"),
              "xyz.openbmc_project.Inventory.Connector.Port.LinkStatus.NoLink");
}

// Update — port characteristics map onto the speeds and the width

TEST_F(NvidiaNVLinkPortTest, UpdateSuccessUpdatesSpeedsAndWidth)
{
    constexpr uint32_t lineRateMbps = 100000;
    constexpr uint32_t dataRateKbps = 50000000;
    constexpr uint32_t laneInfo = 0xF4; // low nibble is the width
    constexpr uint64_t expectedMaxSpeed = 100000ULL * 1000000ULL;
    constexpr uint64_t expectedSpeed = 50000000ULL * 1000ULL;

    expectPolledResponses(
        buildPortStatusResponse(portStateUp, portStatusEnabled),
        buildPortCharacteristicsResponse(0, lineRateMbps, dataRateKbps,
                                         laneInfo));

    const std::string gpuName = "nvlink_port_speeds";
    const std::shared_ptr<NvidiaNVLinkPort> port = createPort(gpuName);
    port->update();
    const std::string path = portPath(gpuName);

    EXPECT_EQ(getProperty<uint64_t>(path, portInterface, "MaxSpeed"),
              expectedMaxSpeed);
    EXPECT_EQ(getProperty<uint64_t>(path, portInterface, "Speed"),
              expectedSpeed);
    EXPECT_EQ(getProperty<size_t>(path, portInterface, "Width"),
              static_cast<size_t>(4));
}

// Update — request encoding verification

TEST_F(NvidiaNVLinkPortTest, UpdateVerifiesRequestEncoding)
{
    // Copy the request bytes before completing the call: the reqMsg span is
    // a view into caller-owned memory, valid only during the call. An update
    // polls both the status and the characteristics, so keep the former.
    std::vector<uint8_t> lastRequest;
    const std::vector<uint8_t> response;
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly([&](uint8_t /*eid*/, std::span<const uint8_t> reqMsg,
                            auto callback) {
            if (requestCommand(reqMsg) ==
                static_cast<uint8_t>(gpu::NetworkPortCommands::QueryPortStatus))
            {
                lastRequest.assign(reqMsg.begin(), reqMsg.end());
            }
            callback(std::error_code{}, response);
        });

    const std::shared_ptr<NvidiaNVLinkPort> port =
        createPort("nvlink_port_req_enc");
    port->update();

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
              static_cast<uint8_t>(gpu::NetworkPortCommands::QueryPortStatus));

    uint8_t dataSize = 0;
    unpack.unpack(dataSize);
    EXPECT_EQ(dataSize, sizeof(uint8_t));

    // Port numbers are 1-based on the wire.
    uint8_t portNumber = 0;
    unpack.unpack(portNumber);
    EXPECT_EQ(portNumber, defaultPortIndex + 1);

    EXPECT_EQ(unpack.getError(), 0);
}

TEST_F(NvidiaNVLinkPortTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaNVLinkPort> port =
        createPort("nvlink_port_eid", testEid);
    port->update();
}

// Error handling

TEST_F(NvidiaNVLinkPortTest, UpdateMctpTransportErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<NvidiaNVLinkPort> port =
        createPort("nvlink_port_mctp_err");
    EXPECT_NO_THROW(port->update());
}

TEST_F(NvidiaNVLinkPortTest, UpdateEmptyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaNVLinkPort> port =
        createPort("nvlink_port_empty");
    EXPECT_NO_THROW(port->update());
}

} // namespace
