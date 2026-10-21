/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaNVLinkPortControl.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <sdbusplus/exception.hpp>

#include <array>
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

constexpr uint8_t defaultPortCount = 2;

constexpr const char* linkStateInterfaceName =
    "xyz.openbmc_project.Control.Port.LinkState";

constexpr const char* linkStateUnknown =
    "xyz.openbmc_project.Inventory.Connector.Port.LinkState.Unknown";
constexpr const char* linkStateEnabled =
    "xyz.openbmc_project.Inventory.Connector.Port.LinkState.Enabled";
constexpr const char* linkStateDisabled =
    "xyz.openbmc_project.Inventory.Connector.Port.LinkState.Disabled";

using PortMask = std::array<uint8_t, gpu::portDisableFutureMaskSize>;

// Marks the port with the given (1-based) port number as disabled.
void disablePort(PortMask& mask, size_t portNumber)
{
    mask[portNumber / 8] |= static_cast<uint8_t>(1U << (portNumber % 8));
}

// Get Port Disable Future (Nvidia MCTP VDM 0x45): the 256-bit port mask.
std::vector<uint8_t> buildPortDisableFutureResponse(const PortMask& mask)
{
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + mask.size());
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));
    pack.pack(
        static_cast<uint8_t>(gpu::NetworkPortCommands::GetPortDisableFuture));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(static_cast<uint16_t>(mask.size()));
    for (const uint8_t byte : mask)
    {
        pack.pack(byte);
    }
    return buf;
}

std::vector<uint8_t> buildErrorResponse(
    ocp::accelerator_management::CompletionCode cc)
{
    std::vector<uint8_t> buf(ocp::accelerator_management::commonResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));
    pack.pack(
        static_cast<uint8_t>(gpu::NetworkPortCommands::GetPortDisableFuture));
    pack.pack(static_cast<uint8_t>(cc));
    pack.pack(static_cast<uint16_t>(0)); // reason code
    return buf;
}

class NvidiaNVLinkPortControlTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaNVLinkPortControl> createControl(
        const std::string& gpuName, uint8_t eid = test_utils::defaultEid,
        uint8_t numberOfPorts = defaultPortCount)
    {
        return std::make_shared<NvidiaNVLinkPortControl>(
            objects(), requester(), gpuName, eid, numberOfPorts);
    }

    static std::string controlPath(const std::string& gpuName,
                                   uint8_t portIndex)
    {
        return "/xyz/openbmc_project/control/port/" + gpuName + "/NVLink_" +
               std::to_string(portIndex);
    }

    static std::string requestedLinkState(const std::string& gpuName,
                                          uint8_t portIndex)
    {
        return getProperty<std::string>(controlPath(gpuName, portIndex),
                                        linkStateInterfaceName,
                                        "RequestedLinkState");
    }
};

// Constructor — D-Bus object creation

TEST_F(NvidiaNVLinkPortControlTest, ConstructorPublishesUnknownPerPort)
{
    const std::string gpuName = "nvlink_ctrl_ctor";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl(gpuName);

    for (uint8_t i = 0; i < defaultPortCount; ++i)
    {
        EXPECT_EQ(requestedLinkState(gpuName, i), linkStateUnknown);
    }
}

TEST_F(NvidiaNVLinkPortControlTest, ConstructorAssociatesControlWithPort)
{
    const std::string gpuName = "nvlink_ctrl_assoc";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl(gpuName);

    using AssociationList =
        std::vector<std::tuple<std::string, std::string, std::string>>;
    const AssociationList associations = getProperty<AssociationList>(
        controlPath(gpuName, 1), "xyz.openbmc_project.Association.Definitions",
        "Associations");

    ASSERT_EQ(associations.size(), 1U);
    EXPECT_EQ(std::get<0>(associations[0]), "controlling");
    EXPECT_EQ(std::get<1>(associations[0]), "controlled_by");
    EXPECT_EQ(std::get<2>(associations[0]),
              "/xyz/openbmc_project/inventory/" + gpuName + "/NVLink_1");
}

TEST_F(NvidiaNVLinkPortControlTest, DestructorRemovesInterfaces)
{
    const std::string gpuName = "nvlink_ctrl_dtor";
    {
        const std::shared_ptr<NvidiaNVLinkPortControl> control =
            createControl(gpuName);
        EXPECT_NO_THROW(requestedLinkState(gpuName, 0));
    }
    drainPendingAsync();
    EXPECT_THROW(requestedLinkState(gpuName, 0), sdbusplus::exception_t);
}

// Update — the port mask maps onto RequestedLinkState

TEST_F(NvidiaNVLinkPortControlTest, UpdateMapsMaskBitsToRequestedLinkState)
{
    // NVLink_1 is port number 2.
    PortMask mask{};
    disablePort(mask, 2);
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildPortDisableFutureResponse(mask)));

    const std::string gpuName = "nvlink_ctrl_bits";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl(gpuName);
    control->update();

    EXPECT_EQ(requestedLinkState(gpuName, 0), linkStateEnabled);
    EXPECT_EQ(requestedLinkState(gpuName, 1), linkStateDisabled);
}

TEST_F(NvidiaNVLinkPortControlTest, UpdateIgnoresMaskBitZero)
{
    // Port numbers start at 1, so bit 0 never belongs to a port.
    PortMask mask{};
    mask[0] = 0x01;
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildPortDisableFutureResponse(mask)));

    const std::string gpuName = "nvlink_ctrl_bit0";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl(gpuName);
    control->update();

    EXPECT_EQ(requestedLinkState(gpuName, 0), linkStateEnabled);
    EXPECT_EQ(requestedLinkState(gpuName, 1), linkStateEnabled);
}

TEST_F(NvidiaNVLinkPortControlTest, UpdateReadsPortsBeyondFirstMaskByte)
{
    // NVLink_8 is port number 9, the second bit of the second mask byte.
    constexpr uint8_t portCount = 10;
    PortMask mask{};
    disablePort(mask, 9);
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildPortDisableFutureResponse(mask)));

    const std::string gpuName = "nvlink_ctrl_byte1";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl(gpuName, test_utils::defaultEid, portCount);
    control->update();

    for (uint8_t i = 0; i < portCount; ++i)
    {
        EXPECT_EQ(requestedLinkState(gpuName, i),
                  i == 8 ? linkStateDisabled : linkStateEnabled);
    }
}

TEST_F(NvidiaNVLinkPortControlTest, UpdateVerifiesRequestEncoding)
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

    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl("nvlink_ctrl_req_enc");
    control->update();

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
                           gpu::NetworkPortCommands::GetPortDisableFuture));

    uint8_t dataSize = 0;
    unpack.unpack(dataSize);
    EXPECT_EQ(dataSize, 0);
    EXPECT_EQ(unpack.getError(), 0);
}

TEST_F(NvidiaNVLinkPortControlTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl("nvlink_ctrl_eid", testEid);
    control->update();
}

// Error handling — the requested state stays at its last known value

TEST_F(NvidiaNVLinkPortControlTest, UpdateErrorCompletionCodeKeepsUnknown)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {},
            buildErrorResponse(
                ocp::accelerator_management::CompletionCode::ERR_NOT_READY)));

    const std::string gpuName = "nvlink_ctrl_cc_err";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl(gpuName);
    control->update();

    EXPECT_EQ(requestedLinkState(gpuName, 0), linkStateUnknown);
}

TEST_F(NvidiaNVLinkPortControlTest, UpdateMctpTransportErrorKeepsUnknown)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::string gpuName = "nvlink_ctrl_mctp_err";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl(gpuName);
    EXPECT_NO_THROW(control->update());

    EXPECT_EQ(requestedLinkState(gpuName, 0), linkStateUnknown);
}

TEST_F(NvidiaNVLinkPortControlTest, UpdateEmptyBufferKeepsUnknown)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    const std::string gpuName = "nvlink_ctrl_empty";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl(gpuName);
    EXPECT_NO_THROW(control->update());

    EXPECT_EQ(requestedLinkState(gpuName, 0), linkStateUnknown);
}

} // namespace
