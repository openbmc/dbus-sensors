/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaNVLinkPortStatus.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <sdbusplus/asio/object_server.hpp>

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

constexpr const char* portInterfaceName =
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

// Query Port Status (Nvidia MCTP VDM 0x43): portState + portStatus.
std::vector<uint8_t> buildPortStatusResponse(uint8_t portState,
                                             uint8_t portStatus)
{
    const std::vector<uint8_t> payload{portState, portStatus};

    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + payload.size());
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));
    pack.pack(static_cast<uint8_t>(gpu::NetworkPortCommands::QueryPortStatus));
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

class NvidiaNVLinkPortStatusTest : public MctpMockTestBase
{
  protected:
    // The status only sets properties, so mirror GpuDevice and publish the
    // port object, seeded with its defaults, before creating the class.
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
        portInterface->register_property(
            "LinkStatus",
            std::string("xyz.openbmc_project.Inventory.Connector.Port."
                        "LinkStatus.NoLink"));
        portInterface->register_property(
            "LinkState",
            std::string("xyz.openbmc_project.Inventory.Connector.Port."
                        "LinkState.Unknown"));
        portInterface->initialize();
    }

    std::shared_ptr<NvidiaNVLinkPortStatus> createStatus(
        const std::string& gpuName, uint8_t eid = test_utils::defaultEid,
        uint8_t portIndex = defaultPortIndex)
    {
        makePortInterface(gpuName, portIndex);
        return std::make_shared<NvidiaNVLinkPortStatus>(
            requester(), eid, portIndex, portInterface);
    }

    std::shared_ptr<sdbusplus::asio::dbus_interface> portInterface;
};

// Update — port status maps onto LinkStatus / LinkState

TEST_F(NvidiaNVLinkPortStatusTest, UpdateSuccessUpdatesLinkStatusAndState)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith(
            {}, buildPortStatusResponse(portStateUp, portStatusEnabled)));

    const std::string gpuName = "nvlink_status_up";
    const std::shared_ptr<NvidiaNVLinkPortStatus> status =
        createStatus(gpuName);
    status->update();
    const std::string path = portPath(gpuName);

    EXPECT_EQ(getProperty<std::string>(path, portInterfaceName, "LinkStatus"),
              "xyz.openbmc_project.Inventory.Connector.Port.LinkStatus.LinkUp");
    EXPECT_EQ(getProperty<std::string>(path, portInterfaceName, "LinkState"),
              "xyz.openbmc_project.Inventory.Connector.Port.LinkState.Enabled");
}

TEST_F(NvidiaNVLinkPortStatusTest, UpdateMapsDownPortStateToLinkDown)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith(
            {}, buildPortStatusResponse(portStateDown, portStatusDisabled)));

    const std::string gpuName = "nvlink_status_down";
    const std::shared_ptr<NvidiaNVLinkPortStatus> status =
        createStatus(gpuName);
    status->update();
    const std::string path = portPath(gpuName);

    EXPECT_EQ(
        getProperty<std::string>(path, portInterfaceName, "LinkStatus"),
        "xyz.openbmc_project.Inventory.Connector.Port.LinkStatus.LinkDown");
    EXPECT_EQ(
        getProperty<std::string>(path, portInterfaceName, "LinkState"),
        "xyz.openbmc_project.Inventory.Connector.Port.LinkState.Disabled");
}

TEST_F(NvidiaNVLinkPortStatusTest, UpdateMapsTrainingPortStateToTraining)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith(
            {}, buildPortStatusResponse(portStateTraining, portStatusEnabled)));

    const std::string gpuName = "nvlink_status_training";
    const std::shared_ptr<NvidiaNVLinkPortStatus> status =
        createStatus(gpuName);
    status->update();

    EXPECT_EQ(
        getProperty<std::string>(portPath(gpuName), portInterfaceName,
                                 "LinkStatus"),
        "xyz.openbmc_project.Inventory.Connector.Port.LinkStatus.Training");
}

TEST_F(NvidiaNVLinkPortStatusTest, UpdateMapsPollingPortStateToStarting)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith(
            {}, buildPortStatusResponse(portStatePolling, portStatusEnabled)));

    const std::string gpuName = "nvlink_status_polling";
    const std::shared_ptr<NvidiaNVLinkPortStatus> status =
        createStatus(gpuName);
    status->update();

    EXPECT_EQ(
        getProperty<std::string>(portPath(gpuName), portInterfaceName,
                                 "LinkStatus"),
        "xyz.openbmc_project.Inventory.Connector.Port.LinkStatus.Starting");
}

TEST_F(NvidiaNVLinkPortStatusTest, UpdateMapsUnknownPortStateToNoLink)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith(
            {}, buildPortStatusResponse(portStateReserved, portStatusEnabled)));

    const std::string gpuName = "nvlink_status_nolink";
    const std::shared_ptr<NvidiaNVLinkPortStatus> status =
        createStatus(gpuName);
    status->update();

    EXPECT_EQ(getProperty<std::string>(portPath(gpuName), portInterfaceName,
                                       "LinkStatus"),
              "xyz.openbmc_project.Inventory.Connector.Port.LinkStatus.NoLink");
}

// Update — request encoding verification

TEST_F(NvidiaNVLinkPortStatusTest, UpdateVerifiesRequestEncoding)
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

    const std::shared_ptr<NvidiaNVLinkPortStatus> status =
        createStatus("nvlink_status_req_enc");
    status->update();

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

TEST_F(NvidiaNVLinkPortStatusTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaNVLinkPortStatus> status =
        createStatus("nvlink_status_eid", testEid);
    status->update();
}

// Error handling

TEST_F(NvidiaNVLinkPortStatusTest, UpdateMctpTransportErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<NvidiaNVLinkPortStatus> status =
        createStatus("nvlink_status_mctp_err");
    EXPECT_NO_THROW(status->update());
}

TEST_F(NvidiaNVLinkPortStatusTest, UpdateEmptyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaNVLinkPortStatus> status =
        createStatus("nvlink_status_empty");
    EXPECT_NO_THROW(status->update());
}

} // namespace
