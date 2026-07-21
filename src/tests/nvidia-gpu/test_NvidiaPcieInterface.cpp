/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaPcieInterface.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

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

// QueryScalarGroupTelemetryV2 response: CommonResponse(11) + N*uint32_t
// Telemetry indices: [0]=GenInUse [1]=LanesInUse [2]=unused [3]=GenSupported
// [4]=MaxLanes
std::vector<uint8_t> buildScalarTelemetryResponse(
    const std::vector<uint32_t>& values)
{
    return test_utils::buildPcieScalarTelemetryResponse(values);
}

class NvidiaPcieInterfaceTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaPcieInterface> createPcieInterface(
        const std::string& name = "PCIE_IF",
        uint8_t eid = test_utils::defaultEid)
    {
        const std::string path = "/test/pcie/" + name;
        return std::make_shared<NvidiaPcieInterface>(
            bus(), requester(), name, path, eid, objects(),
            gpu::DeviceIdentification::DEVICE_PCIE);
    }

    static std::string pciePath(const std::string& name)
    {
        return "/xyz/openbmc_project/inventory/" + name;
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaPcieInterfaceTest, ConstructorCreatesPcieDeviceInterface)
{
    const std::string name = "pcie_ctor";
    const std::shared_ptr<NvidiaPcieInterface> pcieIf =
        createPcieInterface(name);
    const std::string path = pciePath(name);

    EXPECT_EQ(
        getProperty<std::string>(
            path, "xyz.openbmc_project.Inventory.Item.PCIeDevice",
            "GenerationInUse"),
        "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Unknown");
    EXPECT_EQ(getProperty<size_t>(
                  path, "xyz.openbmc_project.Inventory.Item.PCIeDevice",
                  "LanesInUse"),
              std::numeric_limits<size_t>::max());
    EXPECT_EQ(
        getProperty<std::string>(
            path, "xyz.openbmc_project.Inventory.Item.PCIeDevice",
            "GenerationSupported"),
        "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Unknown");
    EXPECT_EQ(getProperty<size_t>(
                  path, "xyz.openbmc_project.Inventory.Item.PCIeDevice",
                  "MaxLanes"),
              static_cast<size_t>(0));
}

// Update — successful response updates D-Bus properties

TEST_F(NvidiaPcieInterfaceTest, UpdateSuccessUpdatesGeneration)
{
    // GenInUse=5(Gen5), LanesInUse=5(x16), unused=0, GenSupported=5, MaxLanes=5
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildScalarTelemetryResponse({5, 5, 0, 5, 5})));

    const std::string name = "pcie_update_gen";
    const std::shared_ptr<NvidiaPcieInterface> pcieIf =
        createPcieInterface(name);
    pcieIf->update();
    const std::string path = pciePath(name);

    EXPECT_EQ(getProperty<std::string>(
                  path, "xyz.openbmc_project.Inventory.Item.PCIeDevice",
                  "GenerationInUse"),
              "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Gen5");
    EXPECT_EQ(getProperty<size_t>(
                  path, "xyz.openbmc_project.Inventory.Item.PCIeDevice",
                  "LanesInUse"),
              static_cast<size_t>(16));
    EXPECT_EQ(getProperty<std::string>(
                  path, "xyz.openbmc_project.Inventory.Item.PCIeDevice",
                  "GenerationSupported"),
              "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Gen5");
    EXPECT_EQ(getProperty<size_t>(
                  path, "xyz.openbmc_project.Inventory.Item.PCIeDevice",
                  "MaxLanes"),
              static_cast<size_t>(16));
}

// Update — request encoding verification

TEST_F(NvidiaPcieInterfaceTest, UpdateVerifiesRequestEncoding)
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

    const std::shared_ptr<NvidiaPcieInterface> pcieIf =
        createPcieInterface("pcie_req_enc");
    pcieIf->update();

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

TEST_F(NvidiaPcieInterfaceTest, UpdateSendsRequest)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaPcieInterface> pcieIf =
        createPcieInterface("pcie_sends");
    pcieIf->update();
}

TEST_F(NvidiaPcieInterfaceTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaPcieInterface> pcieIf =
        createPcieInterface("pcie_eid_test", testEid);
    pcieIf->update();
}

// Error handling

TEST_F(NvidiaPcieInterfaceTest, UpdateMctpTransportErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<NvidiaPcieInterface> pcieIf =
        createPcieInterface("pcie_mctp_err");
    EXPECT_NO_THROW(pcieIf->update());
}

TEST_F(NvidiaPcieInterfaceTest, UpdateEmptyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaPcieInterface> pcieIf =
        createPcieInterface("pcie_empty");
    EXPECT_NO_THROW(pcieIf->update());
}

} // namespace
