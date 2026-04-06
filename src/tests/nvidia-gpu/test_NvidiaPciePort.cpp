/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaPciePort.hpp"
#include "OcpMctpVdm.hpp"

#include <MessagePackUnpackUtils.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

namespace
{

constexpr uint8_t defaultEid = 10;
constexpr uint8_t defaultPortNumber = 0;
constexpr uint8_t defaultUpstreamPortNumber = 0;

// QueryScalarGroupTelemetryV2 response for PCIe port:
// CommonResponse(11) + N*uint32_t telemetry values
// Indices: [0]=GenInUse (maps to speed) [1]=LanesInUse (maps to width)
std::vector<uint8_t> buildPortTelemetryResponse(
    const std::vector<uint32_t>& values)
{
    const size_t dataSize = values.size() * sizeof(uint32_t);
    const size_t responseSize = 11 + dataSize;
    std::vector<uint8_t> buf(responseSize, 0);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));
    pack.pack(static_cast<uint8_t>(0x24)); // QueryScalarGroupTelemetryV2
    pack.pack(static_cast<uint8_t>(0x00)); // cc: SUCCESS
    pack.pack(static_cast<uint16_t>(0));   // reserved
    pack.pack(static_cast<uint16_t>(dataSize));
    for (const uint32_t val : values)
    {
        pack.pack(val);
    }
    return buf;
}

class NvidiaPciePortTestBase : public DbusMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaPciePortInfo> createPciePort(
        const std::string& name = "Port_0",
        const std::string& pcieDeviceName = "PCIE_DEV",
        uint8_t eid = defaultEid,
        gpu::PciePortType portType = gpu::PciePortType::DOWNSTREAM,
        uint8_t upstreamPortNumber = defaultUpstreamPortNumber,
        uint8_t portNumber = defaultPortNumber)
    {
        const std::string path = "/test/pcie/" + pcieDeviceName;
        return std::make_shared<NvidiaPciePortInfo>(
            conn, *mctpRequester, name, pcieDeviceName, path, eid, portType,
            upstreamPortNumber, portNumber, *objectServer,
            gpu::DeviceIdentification::DEVICE_PCIE);
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaPciePortTestBase, ConstructorCreatesPortInterface)
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

TEST_F(NvidiaPciePortTestBase, UpdateSuccessUpdatesSpeedAndWidth)
{
    // Gen5 speed = 32 Gbps, link width value 5 → decoded 16 lanes
    mock_mctp::setNextResponse({}, buildPortTelemetryResponse({5, 5}));
    const std::string pcieDeviceName = "pcie_port_upd";
    const std::string portName = "Port_0";
    const std::shared_ptr<NvidiaPciePortInfo> port =
        createPciePort(portName, pcieDeviceName);
    port->update();
    const std::string path =
        "/xyz/openbmc_project/inventory/" + pcieDeviceName + "/" + portName;

    constexpr uint64_t gen5Speed =
        static_cast<uint64_t>(32.0 * (1 << 30)); // 32 Gbps
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

TEST_F(NvidiaPciePortTestBase, UpdateVerifiesRequestEncoding)
{
    mock_mctp::setNextResponse({}, {});
    const std::shared_ptr<NvidiaPciePortInfo> port =
        createPciePort("Port_0", "pcie_port_req_enc");
    port->update();

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
    EXPECT_EQ(msgType, static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));

    uint8_t command = 0;
    unpack.unpack(command);
    EXPECT_EQ(command, static_cast<uint8_t>(
                           gpu::PcieLinkCommands::QueryScalarGroupTelemetryV2));

    EXPECT_EQ(unpack.getError(), 0);
}

// Update — sends request

TEST_F(NvidiaPciePortTestBase, UpdateSendsRequest)
{
    mock_mctp::setNextResponse({}, {});
    const std::shared_ptr<NvidiaPciePortInfo> port =
        createPciePort("Port_0", "pcie_port_sends");
    port->update();
    EXPECT_GE(mock_mctp::getRequestCount(), 1U);
}

TEST_F(NvidiaPciePortTestBase, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    mock_mctp::setNextResponse({}, {});
    const std::shared_ptr<NvidiaPciePortInfo> port =
        createPciePort("Port_0", "pcie_port_eid", testEid);
    port->update();
    EXPECT_EQ(mock_mctp::getLastEid(), testEid);
}

// Error handling

TEST_F(NvidiaPciePortTestBase, UpdateMctpTransportErrorNoCrash)
{
    mock_mctp::setNextResponse(std::make_error_code(std::errc::timed_out), {});
    const std::shared_ptr<NvidiaPciePortInfo> port =
        createPciePort("Port_0", "pcie_port_mctp_err");
    EXPECT_NO_THROW(port->update());
}

TEST_F(NvidiaPciePortTestBase, UpdateEmptyBufferNoCrash)
{
    mock_mctp::setNextResponse({}, {});
    const std::shared_ptr<NvidiaPciePortInfo> port =
        createPciePort("Port_0", "pcie_port_empty");
    EXPECT_NO_THROW(port->update());
}

} // namespace
