/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaPcieFunction.hpp"
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

const std::string functionIface =
    "xyz.openbmc_project.Inventory.Item.PCIeFunction";

// Build a SUCCESS QueryScalarGroupTelemetryV1 response on the PCIE_LINK
// message type with the supplied uint32_t telemetry values. test_utils has a
// V2 builder; the V1 command byte is needed for the GPU device path.
std::vector<uint8_t> buildPcieScalarTelemetryV1Response(
    const std::vector<uint32_t>& values)
{
    const size_t dataSize = values.size() * sizeof(uint32_t);
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + dataSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));
    pack.pack(static_cast<uint8_t>(
        gpu::PcieLinkCommands::QueryScalarGroupTelemetryV1));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(static_cast<uint16_t>(dataSize));
    for (const uint32_t val : values)
    {
        pack.pack(val);
    }
    EXPECT_EQ(pack.getError(), 0);
    return buf;
}

class NvidiaPcieFunctionTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaPcieFunction> createFunction(
        const std::string& pcieDeviceName, gpu::DeviceIdentification deviceType,
        uint8_t functionNumber = 0, uint8_t eid = test_utils::defaultEid)
    {
        const std::string invPath =
            "/xyz/openbmc_project/inventory/" + pcieDeviceName;
        return std::make_shared<NvidiaPcieFunction>(
            bus(), requester(), pcieDeviceName, invPath, eid, functionNumber,
            objects(), deviceType);
    }

    static std::string functionPath(const std::string& pcieDeviceName,
                                    uint8_t functionNumber = 0)
    {
        return "/xyz/openbmc_project/inventory/" + pcieDeviceName +
               "/Function" + std::to_string(functionNumber);
    }
};

TEST_F(NvidiaPcieFunctionTest, ConstructorGpuDoesNotCrash)
{
    auto func = createFunction("pciefn_gpu_ctor",
                               gpu::DeviceIdentification::DEVICE_GPU);
    ASSERT_NE(func, nullptr);
}

TEST_F(NvidiaPcieFunctionTest, ConstructorPcieDoesNotCrash)
{
    auto func = createFunction("pciefn_pcie_ctor",
                               gpu::DeviceIdentification::DEVICE_PCIE);
    ASSERT_NE(func, nullptr);
}

TEST_F(NvidiaPcieFunctionTest, UpdatePcieSuccessSetsIds)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, test_utils::buildPcieScalarTelemetryResponse(
                    {0x10DE, 0x2330, 0x10DE, 0x1234})));

    auto func =
        createFunction("pciefn_pcie", gpu::DeviceIdentification::DEVICE_PCIE);
    func->update();

    const std::string path = functionPath("pciefn_pcie");
    EXPECT_EQ(getProperty<uint16_t>(path, functionIface, "VendorId"), 0x10DE);
    EXPECT_EQ(getProperty<uint16_t>(path, functionIface, "DeviceId"), 0x2330);
    EXPECT_EQ(getProperty<uint16_t>(path, functionIface, "SubsystemVendorId"),
              0x10DE);
    EXPECT_EQ(getProperty<uint16_t>(path, functionIface, "SubsystemId"),
              0x1234);
}

TEST_F(NvidiaPcieFunctionTest, UpdateGpuSuccessSetsIds)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildPcieScalarTelemetryV1Response(
                                           {0x10DE, 0x2330, 0x10DE, 0x1234})));

    auto func =
        createFunction("pciefn_gpu", gpu::DeviceIdentification::DEVICE_GPU);
    func->update();

    const std::string path = functionPath("pciefn_gpu");
    EXPECT_EQ(getProperty<uint16_t>(path, functionIface, "VendorId"), 0x10DE);
    EXPECT_EQ(getProperty<uint16_t>(path, functionIface, "DeviceId"), 0x2330);
    EXPECT_EQ(getProperty<uint16_t>(path, functionIface, "SubsystemVendorId"),
              0x10DE);
    EXPECT_EQ(getProperty<uint16_t>(path, functionIface, "SubsystemId"),
              0x1234);
}

TEST_F(NvidiaPcieFunctionTest, UpdateInsufficientValuesNoSet)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, test_utils::buildPcieScalarTelemetryResponse({1, 2, 3})));

    auto func =
        createFunction("pciefn_insuff", gpu::DeviceIdentification::DEVICE_PCIE);
    func->update();

    EXPECT_EQ(getProperty<uint16_t>(functionPath("pciefn_insuff"),
                                    functionIface, "VendorId"),
              0);
}

TEST_F(NvidiaPcieFunctionTest, UpdateTruncatesTo16Bits)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, test_utils::buildPcieScalarTelemetryResponse(
                    {0x10001, 0x20002, 0x30003, 0x40004})));

    auto func =
        createFunction("pciefn_trunc", gpu::DeviceIdentification::DEVICE_PCIE);
    func->update();

    EXPECT_EQ(getProperty<uint16_t>(functionPath("pciefn_trunc"), functionIface,
                                    "VendorId"),
              1);
}

TEST_F(NvidiaPcieFunctionTest, UpdateGpuVerifiesRequestEncoding)
{
    // Copy the request bytes before completing the call: the reqMsg span is
    // a view into caller-owned memory, valid only during the call.
    std::vector<uint8_t> lastRequest;
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce([&](uint8_t /*eid*/, std::span<const uint8_t> reqMsg,
                      auto callback) {
            lastRequest.assign(reqMsg.begin(), reqMsg.end());
            callback(std::error_code{}, {});
        });

    auto func =
        createFunction("pciefn_gpu_enc", gpu::DeviceIdentification::DEVICE_GPU);
    func->update();

    ASSERT_FALSE(lastRequest.empty());

    UnpackBuffer unpack(lastRequest);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  unpack, gpu::nvidiaPciVendorId, ocpMsgType, instanceId,
                  msgType),
              0);
    EXPECT_EQ(msgType, static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));

    uint8_t command = 0;
    unpack.unpack(command);
    EXPECT_EQ(command, static_cast<uint8_t>(
                           gpu::PcieLinkCommands::QueryScalarGroupTelemetryV1));
    EXPECT_EQ(unpack.getError(), 0);
}

TEST_F(NvidiaPcieFunctionTest, UpdatePcieVerifiesRequestEncoding)
{
    std::vector<uint8_t> lastRequest;
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce([&](uint8_t /*eid*/, std::span<const uint8_t> reqMsg,
                      auto callback) {
            lastRequest.assign(reqMsg.begin(), reqMsg.end());
            callback(std::error_code{}, {});
        });

    auto func = createFunction("pciefn_pcie_enc",
                               gpu::DeviceIdentification::DEVICE_PCIE);
    func->update();

    ASSERT_FALSE(lastRequest.empty());

    UnpackBuffer unpack(lastRequest);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  unpack, gpu::nvidiaPciVendorId, ocpMsgType, instanceId,
                  msgType),
              0);
    EXPECT_EQ(msgType, static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));

    uint8_t command = 0;
    unpack.unpack(command);
    EXPECT_EQ(command, static_cast<uint8_t>(
                           gpu::PcieLinkCommands::QueryScalarGroupTelemetryV2));
    EXPECT_EQ(unpack.getError(), 0);
}

TEST_F(NvidiaPcieFunctionTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, {}));

    auto func = createFunction(
        "pciefn_eid", gpu::DeviceIdentification::DEVICE_PCIE, 0, testEid);
    func->update();
}

TEST_F(NvidiaPcieFunctionTest, UpdateWrongCommandByteDecodeError)
{
    // Feed a V2 response to a GPU object (which decodes V1) → decode fails.
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, test_utils::buildPcieScalarTelemetryResponse(
                    {0x10DE, 0x2330, 0x10DE, 0x1234})));

    auto func = createFunction("pciefn_wrongcmd",
                               gpu::DeviceIdentification::DEVICE_GPU);
    func->update();

    EXPECT_EQ(getProperty<uint16_t>(functionPath("pciefn_wrongcmd"),
                                    functionIface, "VendorId"),
              0);
}

TEST_F(NvidiaPcieFunctionTest, UpdateMctpTransportErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    auto func = createFunction("pciefn_mctp_err",
                               gpu::DeviceIdentification::DEVICE_PCIE);
    EXPECT_NO_THROW(func->update());
}

TEST_F(NvidiaPcieFunctionTest, UpdateEmptyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    auto func =
        createFunction("pciefn_empty", gpu::DeviceIdentification::DEVICE_PCIE);
    EXPECT_NO_THROW(func->update());
}

TEST_F(NvidiaPcieFunctionTest, UpdateTinyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, {0x00, 0x01}));

    auto func =
        createFunction("pciefn_tiny", gpu::DeviceIdentification::DEVICE_PCIE);
    EXPECT_NO_THROW(func->update());
}

} // namespace
