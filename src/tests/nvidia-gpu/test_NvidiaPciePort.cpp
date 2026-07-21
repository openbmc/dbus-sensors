/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaPciePort.hpp"
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

constexpr uint8_t defaultPortNumber = 0;
constexpr uint8_t defaultUpstreamPortNumber = 0;

// QueryScalarGroupTelemetryV2 response for PCIe port:
// CommonResponse(11) + N*uint32_t telemetry values
// Indices: [0]=GenInUse (maps to speed) [1]=LanesInUse (maps to width)
std::vector<uint8_t> buildPortTelemetryResponse(
    const std::vector<uint32_t>& values)
{
    return test_utils::buildPcieScalarTelemetryResponse(values);
}

class NvidiaPciePortTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaPciePortInfo> createPciePort(
        const std::string& name = "Port_0",
        const std::string& pcieDeviceName = "PCIE_DEV",
        uint8_t eid = test_utils::defaultEid,
        gpu::PciePortType portType = gpu::PciePortType::DOWNSTREAM,
        uint8_t upstreamPortNumber = defaultUpstreamPortNumber,
        uint8_t portNumber = defaultPortNumber)
    {
        const std::string path = "/test/pcie/" + pcieDeviceName;
        return std::make_shared<NvidiaPciePortInfo>(
            bus(), requester(), name, pcieDeviceName, path, eid, portType,
            upstreamPortNumber, portNumber, objects(),
            gpu::DeviceIdentification::DEVICE_PCIE);
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaPciePortTest, ConstructorCreatesPortInterface)
{
    const std::string pcieDeviceName = "pcie_port_ctor";
    const std::string portName = "Port_0";
    const std::shared_ptr<NvidiaPciePortInfo> port =
        createPciePort(portName, pcieDeviceName);
    const std::string path =
        "/xyz/openbmc_project/inventory/" + pcieDeviceName + "/" + portName;

    EXPECT_EQ(getProperty<std::string>(
                  path, "xyz.openbmc_project.Inventory.Connector.Port",
                  "PortProtocol"),
              "xyz.openbmc_project.Inventory.Connector.Port.PortProtocol.PCIe");
}

// Update — successful response updates D-Bus properties

TEST_F(NvidiaPciePortTest, UpdateSuccessUpdatesSpeedAndWidth)
{
    // Gen5 speed = 32 Gbps, link width value 5 → decoded 16 lanes
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildPortTelemetryResponse({5, 5})));

    const std::string pcieDeviceName = "pcie_port_upd";
    const std::string portName = "Port_0";
    const std::shared_ptr<NvidiaPciePortInfo> port =
        createPciePort(portName, pcieDeviceName);
    port->update();
    const std::string path =
        "/xyz/openbmc_project/inventory/" + pcieDeviceName + "/" + portName;

    constexpr uint64_t gen5Speed =
        static_cast<uint64_t>(32.0 * 1000000000); // 32 Gbps
    EXPECT_EQ(getProperty<uint64_t>(
                  path, "xyz.openbmc_project.Inventory.Connector.Port",
                  "Speed"),
              gen5Speed);
    EXPECT_EQ(getProperty<size_t>(
                  path, "xyz.openbmc_project.Inventory.Connector.Port",
                  "Width"),
              static_cast<size_t>(16));
}

// Update — request encoding verification

TEST_F(NvidiaPciePortTest, UpdateVerifiesRequestEncoding)
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

    const std::shared_ptr<NvidiaPciePortInfo> port =
        createPciePort("Port_0", "pcie_port_req_enc");
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
    EXPECT_EQ(msgType, static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));

    uint8_t command = 0;
    unpack.unpack(command);
    EXPECT_EQ(command, static_cast<uint8_t>(
                           gpu::PcieLinkCommands::QueryScalarGroupTelemetryV2));

    EXPECT_EQ(unpack.getError(), 0);
}

// Update — sends request

TEST_F(NvidiaPciePortTest, UpdateSendsRequest)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaPciePortInfo> port =
        createPciePort("Port_0", "pcie_port_sends");
    port->update();
}

TEST_F(NvidiaPciePortTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaPciePortInfo> port =
        createPciePort("Port_0", "pcie_port_eid", testEid);
    port->update();
}

// Error handling

TEST_F(NvidiaPciePortTest, UpdateMctpTransportErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<NvidiaPciePortInfo> port =
        createPciePort("Port_0", "pcie_port_mctp_err");
    EXPECT_NO_THROW(port->update());
}

TEST_F(NvidiaPciePortTest, UpdateEmptyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaPciePortInfo> port =
        createPciePort("Port_0", "pcie_port_empty");
    EXPECT_NO_THROW(port->update());
}

} // namespace
