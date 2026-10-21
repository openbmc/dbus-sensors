/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaPcieDevice.hpp"
#include "NvidiaSensorConfig.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <sdbusplus/exception.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

class NvidiaPcieDeviceTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<PcieDevice> createDevice(
        const std::string& name = "PCIE_DEV",
        uint8_t eid = test_utils::defaultEid, uint64_t pollRate = 1000,
        uint64_t networkPortCount = 0)
    {
        const std::string path = "/test/pcie/" + name;
        const NvidiaMctpVdmConfig config{
            .path = path, .name = name, .pollRate = pollRate};
        const PcieDeviceConfigs pcieConfig{
            .networkPortCount = networkPortCount};
        return std::make_shared<PcieDevice>(config, pcieConfig, bus(), eid,
                                            ioContext(), requester(),
                                            objects());
    }
};

// Constructor

TEST_F(NvidiaPcieDeviceTest, ConstructorDoesNotCrash)
{
    const std::shared_ptr<PcieDevice> device = createDevice("pciedev_ctor");
    EXPECT_NE(device, nullptr);
}

TEST_F(NvidiaPcieDeviceTest, GetPathReturnsConfiguredPath)
{
    const std::string name = "pciedev_path";
    const std::shared_ptr<PcieDevice> device = createDevice(name);
    EXPECT_EQ(device->getPath(), "/test/pcie/" + name);
}

// Init

TEST_F(NvidiaPcieDeviceTest, InitSendsAtLeastOneRequest)
{
    // Error response keeps init() from cascading into per-sensor requests.
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<PcieDevice> device = createDevice("pciedev_init");
    device->init();
}

// ReadLoop

TEST_F(NvidiaPcieDeviceTest, ReadLoopStopsOnTimerCancel)
{
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    {
        const std::shared_ptr<PcieDevice> device =
            createDevice("pciedev_readloop");
        device->init();
    }

    EXPECT_NO_THROW(drainPendingAsync());
}

// Destructor

TEST_F(NvidiaPcieDeviceTest, DestructorRemovesInterfaces)
{
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::string name = "pciedev_dtor";
    const std::string adapterPath =
        "/xyz/openbmc_project/inventory/" + name + "_NIC";
    {
        const std::shared_ptr<PcieDevice> device = createDevice(name);
        device->init();
        EXPECT_NO_THROW(getProperty<std::string>(
            adapterPath, "xyz.openbmc_project.Inventory.Decorator.LocationCode",
            "LocationCode"));
    }
    drainPendingAsync();

    EXPECT_THROW(getProperty<std::string>(
                     adapterPath,
                     "xyz.openbmc_project.Inventory.Decorator.LocationCode",
                     "LocationCode"),
                 sdbusplus::exception_t);
}

// Transceiver discovery

// Query Port Telemetry Capabilities answer with groups 1, 4, 5 and 6 in the
// first bitmask byte and groupsByte1 in the second, which holds group 9.
std::vector<uint8_t> buildCapabilitiesResponse(uint8_t maxGroupId,
                                               uint8_t groupsByte1)
{
    constexpr uint16_t dataSize =
        sizeof(uint8_t) + sizeof(uint8_t) + gpu::supportedListBitfieldSize +
        sizeof(uint16_t);
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + dataSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));
    pack.pack(static_cast<uint8_t>(
        gpu::NetworkPortCommands::QueryPortTelemetryCapabilities));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(dataSize);
    pack.pack(maxGroupId);
    pack.pack(static_cast<uint8_t>(0x42)); // max records per response
    pack.pack(static_cast<uint8_t>(0x72));
    pack.pack(groupsByte1);
    return buf;
}

// Get Port Network Addresses answer for a port of the given link type.
std::vector<uint8_t> buildPortResponse(gpu::NetworkPortLinkType linkType)
{
    // Header(5) + command(1) + cc(1) + telemetryCount(2) + a link type record
    // of tag(1), tagInfo(1) and value(1)
    std::vector<uint8_t> buf(
        ocp::accelerator_management::messageHeaderSize + 7);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));
    pack.pack(static_cast<uint8_t>(
        gpu::NetworkPortCommands::GetPortNetworkAddresses));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(1));   // telemetryCount
    pack.pack(static_cast<uint8_t>(0));    // tag: link type
    pack.pack(static_cast<uint8_t>(0x01)); // valid, one byte
    pack.pack(static_cast<uint8_t>(linkType));
    return buf;
}

// List PCIe Ports answer with no upstream ports.
std::vector<uint8_t> buildNoPciePortsResponse()
{
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + sizeof(uint16_t));
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));
    pack.pack(static_cast<uint8_t>(gpu::PcieLinkCommands::ListPCIePorts));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0));                // reserved
    pack.pack(static_cast<uint16_t>(sizeof(uint16_t))); // dataSize
    pack.pack(static_cast<uint16_t>(0));                // upstream ports
    return buf;
}

// Whether a request is the given command of the given message type.
bool isRequestFor(std::span<const uint8_t> request, gpu::MessageType type,
                  uint8_t command)
{
    UnpackBuffer unpack(request);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    uint8_t requestCommand = 0;
    if (ocp::accelerator_management::unpackHeader(
            unpack, gpu::nvidiaPciVendorId, ocpMsgType, instanceId, msgType) !=
            0 ||
        unpack.unpack(requestCommand) != 0)
    {
        return false;
    }
    return msgType == static_cast<uint8_t>(type) && requestCommand == command;
}

// The port number a request carries at the given offset.
uint16_t portNumberOf(std::span<const uint8_t> request, size_t offset)
{
    UnpackBuffer unpack(request.subspan(offset));
    uint16_t portNumber = 0;
    unpack.unpack(portNumber);
    return portNumber;
}

using mock_mctp::PendingRequest;

class NvidiaPcieDeviceTransceiverTest : public NvidiaPcieDeviceTest
{
  protected:
    void SetUp() override
    {
        NvidiaPcieDeviceTest::SetUp();
        if (testing::Test::IsSkipped())
        {
            return;
        }

        ON_CALL(mctpMock, sendRecvMsg)
            .WillByDefault(mock_mctp::deferInto(pending));
    }

    // Answers the first waiting request of the given message type and
    // command; false if there is none.
    bool respondTo(gpu::MessageType type, uint8_t command,
                   const std::vector<uint8_t>& response,
                   const std::error_code& ec = {})
    {
        const auto it =
            std::ranges::find_if(pending, [type, command](const auto& p) {
                return isRequestFor(p.request, type, command);
            });
        if (it == pending.end())
        {
            return false;
        }
        PendingRequest next = std::move(*it);
        pending.erase(it);
        next.callback(ec, response);
        return true;
    }

    bool isPending(gpu::MessageType type, uint8_t command) const
    {
        return std::ranges::any_of(pending, [type, command](const auto& p) {
            return isRequestFor(p.request, type, command);
        });
    }

    // Brings up a device with the given number of ports of the given link
    // type, answering the capabilities query with the given response, or with
    // a timeout if there is none.
    std::shared_ptr<PcieDevice> initWithCapabilities(
        const std::string& name,
        const std::optional<std::vector<uint8_t>>& capabilities,
        uint64_t portCount = 1,
        gpu::NetworkPortLinkType linkType = gpu::NetworkPortLinkType::ETHERNET)
    {
        std::shared_ptr<PcieDevice> device =
            createDevice(name, test_utils::defaultEid, 1000, portCount);
        device->init();

        // The ports are discovered once the capabilities are known
        EXPECT_FALSE(
            isPending(gpu::MessageType::NETWORK_PORT,
                      static_cast<uint8_t>(
                          gpu::NetworkPortCommands::GetPortNetworkAddresses)));

        EXPECT_TRUE(respondTo(
            gpu::MessageType::NETWORK_PORT,
            static_cast<uint8_t>(
                gpu::NetworkPortCommands::QueryPortTelemetryCapabilities),
            capabilities.value_or(std::vector<uint8_t>{}),
            capabilities ? std::error_code{}
                         : std::make_error_code(std::errc::timed_out)));
        for (uint64_t port = 0; port < portCount; ++port)
        {
            EXPECT_TRUE(respondTo(
                gpu::MessageType::NETWORK_PORT,
                static_cast<uint8_t>(
                    gpu::NetworkPortCommands::GetPortNetworkAddresses),
                buildPortResponse(linkType)));
        }

        // Each port is discovered once
        EXPECT_FALSE(
            isPending(gpu::MessageType::NETWORK_PORT,
                      static_cast<uint8_t>(
                          gpu::NetworkPortCommands::GetPortNetworkAddresses)));
        return device;
    }

    static std::string portPathFor(const std::string& name)
    {
        return "/xyz/openbmc_project/inventory/" + name + "_NIC/Port_1";
    }

    static bool hasPort(const std::string& name)
    {
        try
        {
            getProperty<std::string>(
                portPathFor(name),
                "xyz.openbmc_project.Inventory.Connector.Port", "PortProtocol");
            return true;
        }
        catch (const sdbusplus::exception_t&)
        {
            return false;
        }
    }

    static bool hasTransceiver(const std::string& name)
    {
        try
        {
            getProperty<std::string>(
                portPathFor(name) + "/Transceiver",
                "xyz.openbmc_project.Inventory.Item.Transceiver", "Type");
            return true;
        }
        catch (const sdbusplus::exception_t&)
        {
            return false;
        }
    }

    static bool hasTransceiverMetrics(const std::string& name)
    {
        try
        {
            getProperty<std::string>(
                "/xyz/openbmc_project/metric/transceiver_" + name +
                    "_NIC_Port_1/tx_output_power_lane_0",
                "xyz.openbmc_project.Metric.Value", "Unit");
            return true;
        }
        catch (const sdbusplus::exception_t&)
        {
            return false;
        }
    }

    std::deque<PendingRequest> pending;
};

TEST_F(NvidiaPcieDeviceTransceiverTest, OpticalModuleGroupAddsTransceiver)
{
    const std::string name = "pciedev_xcvr";
    const std::shared_ptr<PcieDevice> device =
        initWithCapabilities(name, buildCapabilitiesResponse(0x09, 0x02));

    EXPECT_TRUE(hasPort(name));
    EXPECT_TRUE(hasTransceiver(name));
    EXPECT_TRUE(hasTransceiverMetrics(name));
}

TEST_F(NvidiaPcieDeviceTransceiverTest, InfiniBandPortGetsTransceiver)
{
    const std::string name = "pciedev_xcvr_ib";
    const std::shared_ptr<PcieDevice> device =
        initWithCapabilities(name, buildCapabilitiesResponse(0x09, 0x02), 1,
                             gpu::NetworkPortLinkType::INFINIBAND);

    EXPECT_EQ(
        getProperty<std::string>(portPathFor(name),
                                 "xyz.openbmc_project.Inventory.Connector.Port",
                                 "PortProtocol"),
        "xyz.openbmc_project.Inventory.Connector.Port.PortProtocol."
        "InfiniBand");
    EXPECT_TRUE(hasTransceiver(name));
}

TEST_F(NvidiaPcieDeviceTransceiverTest, UnknownLinkTypeNoPortNoTransceiver)
{
    const std::string name = "pciedev_xcvr_unknown";
    const std::shared_ptr<PcieDevice> device =
        initWithCapabilities(name, buildCapabilitiesResponse(0x09, 0x02), 1,
                             gpu::NetworkPortLinkType::UNKNOWN);

    EXPECT_FALSE(hasPort(name));
    EXPECT_FALSE(hasTransceiver(name));
}

TEST_F(NvidiaPcieDeviceTransceiverTest, NoOpticalModuleGroupNoTransceiver)
{
    const std::string name = "pciedev_xcvr_nogroup";
    // Group 8 is listed, group 9 is not
    const std::shared_ptr<PcieDevice> device =
        initWithCapabilities(name, buildCapabilitiesResponse(0x09, 0x01));

    EXPECT_TRUE(hasPort(name));
    EXPECT_FALSE(hasTransceiver(name));
    EXPECT_FALSE(hasTransceiverMetrics(name));
}

TEST_F(NvidiaPcieDeviceTransceiverTest, GroupAboveMaxGroupIdIsIgnored)
{
    const std::string name = "pciedev_xcvr_maxgroup";
    const std::shared_ptr<PcieDevice> device =
        initWithCapabilities(name, buildCapabilitiesResponse(0x08, 0x02));

    EXPECT_TRUE(hasPort(name));
    EXPECT_FALSE(hasTransceiver(name));
}

TEST_F(NvidiaPcieDeviceTransceiverTest, UnreadableCapabilitiesNoTransceiver)
{
    const std::string name = "pciedev_xcvr_aggregate";
    // Captured from a ConnectX that answers with an aggregate response
    // instead of the capabilities
    const std::shared_ptr<PcieDevice> device = initWithCapabilities(
        name, std::vector<uint8_t>{0x10, 0xde, 0x00, 0x89, 0x01, 0x15, 0x00,
                                   0x05, 0x00, 0x00, 0x03, 0x04, 0x00, 0x01,
                                   0x03, 0x41, 0x00, 0x02, 0x03, 0x39, 0x01,
                                   0x03, 0x01, 0x0f, 0x04, 0x01, 0x07});

    EXPECT_TRUE(hasPort(name));
    EXPECT_FALSE(hasTransceiver(name));
}

TEST_F(NvidiaPcieDeviceTransceiverTest, TruncatedCapabilitiesNoTransceiver)
{
    const std::string name = "pciedev_xcvr_truncated";
    // The bitmask ends after the byte holding group 9
    std::vector<uint8_t> capabilities = buildCapabilitiesResponse(0x09, 0x02);
    capabilities.resize(15);
    const std::shared_ptr<PcieDevice> device =
        initWithCapabilities(name, capabilities);

    EXPECT_TRUE(hasPort(name));
    EXPECT_FALSE(hasTransceiver(name));
}

TEST_F(NvidiaPcieDeviceTransceiverTest, RefusedCapabilitiesNoTransceiver)
{
    const std::string name = "pciedev_xcvr_refused";
    // Captured from a ConnectX that does not support the command
    const std::shared_ptr<PcieDevice> device = initWithCapabilities(
        name, std::vector<uint8_t>{0x10, 0xde, 0x00, 0x89, 0x01, 0x15, 0x05,
                                   0x00, 0x00});

    EXPECT_TRUE(hasPort(name));
    EXPECT_FALSE(hasTransceiver(name));
}

TEST_F(NvidiaPcieDeviceTransceiverTest, CapabilitiesTimeoutStillAddsPorts)
{
    const std::string name = "pciedev_xcvr_timeout";
    const std::shared_ptr<PcieDevice> device =
        initWithCapabilities(name, std::nullopt);

    EXPECT_TRUE(hasPort(name));
    EXPECT_FALSE(hasTransceiver(name));
}

TEST_F(NvidiaPcieDeviceTransceiverTest, PortAddressesRequestedOncePerPort)
{
    const std::shared_ptr<PcieDevice> device =
        createDevice("pciedev_xcvr_once", test_utils::defaultEid, 1000, 2);
    device->init();
    ASSERT_TRUE(
        respondTo(gpu::MessageType::NETWORK_PORT,
                  static_cast<uint8_t>(
                      gpu::NetworkPortCommands::QueryPortTelemetryCapabilities),
                  buildCapabilitiesResponse(0x09, 0x02)));

    const auto isPortAddressRequest = [](const PendingRequest& p) {
        return isRequestFor(
            p.request, gpu::MessageType::NETWORK_PORT,
            static_cast<uint8_t>(
                gpu::NetworkPortCommands::GetPortNetworkAddresses));
    };

    // Every other request times out
    while (true)
    {
        const auto it = std::ranges::find_if_not(pending, isPortAddressRequest);
        if (it == pending.end())
        {
            break;
        }
        PendingRequest next = std::move(*it);
        pending.erase(it);
        next.callback(std::make_error_code(std::errc::timed_out), {});
    }

    std::vector<uint16_t> requestedPorts;
    requestedPorts.reserve(pending.size());
    for (const PendingRequest& p : pending)
    {
        requestedPorts.push_back(portNumberOf(
            p.request, ocp::accelerator_management::commonRequestSize));
    }
    EXPECT_EQ(requestedPorts, (std::vector<uint16_t>{1, 2}));
}

TEST_F(NvidiaPcieDeviceTransceiverTest, RoundRobinPollsTransceiverOfEachPort)
{
    const std::string name = "pciedev_xcvr_poll";
    const std::shared_ptr<PcieDevice> device =
        initWithCapabilities(name, buildCapabilitiesResponse(0x09, 0x02), 2);
    ASSERT_TRUE(hasTransceiver(name));

    ASSERT_TRUE(
        respondTo(gpu::MessageType::PCIE_LINK,
                  static_cast<uint8_t>(gpu::PcieLinkCommands::ListPCIePorts),
                  buildNoPciePortsResponse()));

    std::vector<uint16_t> polledPorts;
    for (const PendingRequest& p : pending)
    {
        if (isRequestFor(p.request, gpu::MessageType::NETWORK_PORT,
                         static_cast<uint8_t>(
                             gpu::NetworkPortCommands::QueryPortTelemetryV2)))
        {
            // The port follows the sequence token
            polledPorts.push_back(portNumberOf(
                p.request, ocp::accelerator_management::commonRequestSize +
                               sizeof(uint32_t)));
        }
    }
    EXPECT_EQ(polledPorts, (std::vector<uint16_t>{1, 2}));
}

TEST_F(NvidiaPcieDeviceTransceiverTest, CapabilitiesAfterDestructionIsIgnored)
{
    std::shared_ptr<PcieDevice> device =
        createDevice("pciedev_xcvr_late", test_utils::defaultEid, 1000, 1);
    device->init();
    device.reset();

    EXPECT_TRUE(
        respondTo(gpu::MessageType::NETWORK_PORT,
                  static_cast<uint8_t>(
                      gpu::NetworkPortCommands::QueryPortTelemetryCapabilities),
                  buildCapabilitiesResponse(0x09, 0x02)));
    EXPECT_FALSE(
        isPending(gpu::MessageType::NETWORK_PORT,
                  static_cast<uint8_t>(
                      gpu::NetworkPortCommands::GetPortNetworkAddresses)));
}

TEST_F(NvidiaPcieDeviceTransceiverTest, DestructorRemovesTransceiver)
{
    const std::string name = "pciedev_xcvr_dtor";
    {
        const std::shared_ptr<PcieDevice> device =
            initWithCapabilities(name, buildCapabilitiesResponse(0x09, 0x02));
        ASSERT_TRUE(hasTransceiver(name));
        ASSERT_TRUE(hasTransceiverMetrics(name));
    }
    drainPendingAsync();

    EXPECT_FALSE(hasTransceiver(name));
    EXPECT_FALSE(hasTransceiverMetrics(name));
}

} // namespace
