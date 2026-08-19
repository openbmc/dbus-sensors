/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuFirmwareInformation.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"
#include "Utils.hpp"

#include <sdbusplus/message/native_types.hpp>

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

// Build a SUCCESS GetInventoryInformation response carrying an ASCII firmware
// version. data_size must match the payload length: the decoder rejects a
// zero or oversized value before it reads the string.
std::vector<uint8_t> buildFirmwareVersionResponse(const std::string& version)
{
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + version.size());
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0));              // reserved
    pack.pack(static_cast<uint16_t>(version.size())); // data_size
    for (const char c : version)
    {
        pack.pack(static_cast<uint8_t>(c));
    }
    return buf;
}

std::vector<uint8_t> buildFirmwareVersionErrorResponse(uint8_t cc,
                                                       uint16_t reasonCode)
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION, cc,
        reasonCode);
}

class NvidiaGpuFirmwareInformationTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaGpuFirmwareInformation> createFirmwareInfo(
        const std::string& name, uint8_t eid = test_utils::defaultEid)
    {
        return std::make_shared<NvidiaGpuFirmwareInformation>(
            requester(), name, sdbusplus::object_path(inventoryPath(name)), eid,
            objects());
    }

    static std::string inventoryPath(const std::string& name)
    {
        return "/xyz/openbmc_project/inventory/" + name;
    }

    static std::string softwarePath(const std::string& name)
    {
        return "/xyz/openbmc_project/software/" + name + "_Firmware";
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaGpuFirmwareInformationTest, ConstructorCreatesVersionInterface)
{
    const std::string name = "fw_ctor";
    const std::shared_ptr<NvidiaGpuFirmwareInformation> firmwareInfo =
        createFirmwareInfo(name);

    EXPECT_EQ(getProperty<std::string>(softwarePath(name),
                                       "xyz.openbmc_project.Software.Version",
                                       "Version"),
              "");
}

// The firmware object is the only Software.Version object associated with the
// inventory item, so a Redfish processor resolves to exactly one of them.
TEST_F(NvidiaGpuFirmwareInformationTest, ConstructorAssociatesToInventoryItem)
{
    const std::string name = "fw_assoc";
    const std::shared_ptr<NvidiaGpuFirmwareInformation> firmwareInfo =
        createFirmwareInfo(name);

    const std::vector<Association> associations =
        getProperty<std::vector<Association>>(
            softwarePath(name), association::interface, "Associations");

    ASSERT_EQ(associations.size(), 1U);
    EXPECT_EQ(std::get<0>(associations.front()), "running");
    EXPECT_EQ(std::get<1>(associations.front()), "ran_on");
    EXPECT_EQ(std::get<2>(associations.front()), inventoryPath(name));
}

// Update — successful version update

TEST_F(NvidiaGpuFirmwareInformationTest, UpdateSuccessUpdatesVersion)
{
    const std::string name = "fw_update";
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildFirmwareVersionResponse("01.02.03.04.05")));

    const std::shared_ptr<NvidiaGpuFirmwareInformation> firmwareInfo =
        createFirmwareInfo(name);
    firmwareInfo->update();

    EXPECT_EQ(getProperty<std::string>(softwarePath(name),
                                       "xyz.openbmc_project.Software.Version",
                                       "Version"),
              "01.02.03.04.05");
}

TEST_F(NvidiaGpuFirmwareInformationTest, UpdateSuccessTwiceOverwritesVersion)
{
    const std::string name = "fw_update2";
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildFirmwareVersionResponse("01.00")))
        .WillOnce(
            mock_mctp::respondWith({}, buildFirmwareVersionResponse("02.00")));

    const std::shared_ptr<NvidiaGpuFirmwareInformation> firmwareInfo =
        createFirmwareInfo(name);
    firmwareInfo->update();

    EXPECT_EQ(getProperty<std::string>(softwarePath(name),
                                       "xyz.openbmc_project.Software.Version",
                                       "Version"),
              "01.00");

    firmwareInfo->update();

    EXPECT_EQ(getProperty<std::string>(softwarePath(name),
                                       "xyz.openbmc_project.Software.Version",
                                       "Version"),
              "02.00");
}

// Update — request encoding verification

TEST_F(NvidiaGpuFirmwareInformationTest, UpdateRequestsFirmwareVersionProperty)
{
    // Copy the request bytes before completing the call: the reqMsg span is
    // a view into caller-owned memory, valid only during the call.
    std::vector<uint8_t> lastRequest;
    const std::vector<uint8_t> response =
        buildFirmwareVersionResponse("01.02.03.04.05");
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce([&](uint8_t /*eid*/, std::span<const uint8_t> reqMsg,
                      auto callback) {
            lastRequest.assign(reqMsg.begin(), reqMsg.end());
            callback(std::error_code{}, response);
        });

    const std::shared_ptr<NvidiaGpuFirmwareInformation> firmwareInfo =
        createFirmwareInfo("fw_req_enc");
    firmwareInfo->update();

    ASSERT_EQ(lastRequest.size(), gpu::getInventoryInformationRequestSize);

    UnpackBuffer unpack(lastRequest);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    const int rc = ocp::accelerator_management::unpackHeader(
        unpack, gpu::nvidiaPciVendorId, ocpMsgType, instanceId, msgType);
    EXPECT_EQ(rc, 0);
    EXPECT_EQ(ocpMsgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(msgType,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    uint8_t command = 0;
    unpack.unpack(command);
    EXPECT_EQ(
        command,
        static_cast<uint8_t>(
            gpu::PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION));

    uint8_t dataSize = 0;
    unpack.unpack(dataSize);
    EXPECT_EQ(dataSize, 1);

    uint8_t propertyId = 0;
    unpack.unpack(propertyId);
    EXPECT_EQ(propertyId,
              static_cast<uint8_t>(gpu::InventoryPropertyId::FIRMWARE_VERSION));

    EXPECT_EQ(unpack.getError(), 0);
}

TEST_F(NvidiaGpuFirmwareInformationTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith(
            {}, buildFirmwareVersionResponse("01.02.03.04.05")));

    const std::shared_ptr<NvidiaGpuFirmwareInformation> firmwareInfo =
        createFirmwareInfo("fw_eid_test", testEid);
    firmwareInfo->update();
}

// Error handling — the version must keep its last good value

TEST_F(NvidiaGpuFirmwareInformationTest, UpdateMctpTransportErrorKeepsVersion)
{
    const std::string name = "fw_mctp_err";
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildFirmwareVersionResponse("01.00")))
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<NvidiaGpuFirmwareInformation> firmwareInfo =
        createFirmwareInfo(name);
    firmwareInfo->update();
    EXPECT_NO_THROW(firmwareInfo->update());

    EXPECT_EQ(getProperty<std::string>(softwarePath(name),
                                       "xyz.openbmc_project.Software.Version",
                                       "Version"),
              "01.00");
}

TEST_F(NvidiaGpuFirmwareInformationTest, UpdateBadCompletionCodeKeepsVersion)
{
    const std::string name = "fw_bad_cc";
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildFirmwareVersionResponse("01.00")))
        .WillOnce(mock_mctp::respondWith(
            {}, buildFirmwareVersionErrorResponse(
                    static_cast<uint8_t>(
                        ocp::accelerator_management::CompletionCode::ERROR),
                    0)));

    const std::shared_ptr<NvidiaGpuFirmwareInformation> firmwareInfo =
        createFirmwareInfo(name);
    firmwareInfo->update();
    EXPECT_NO_THROW(firmwareInfo->update());

    EXPECT_EQ(getProperty<std::string>(softwarePath(name),
                                       "xyz.openbmc_project.Software.Version",
                                       "Version"),
              "01.00");
}

TEST_F(NvidiaGpuFirmwareInformationTest, UpdateEmptyBufferKeepsVersion)
{
    const std::string name = "fw_empty";
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildFirmwareVersionResponse("01.00")))
        .WillOnce(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaGpuFirmwareInformation> firmwareInfo =
        createFirmwareInfo(name);
    firmwareInfo->update();
    EXPECT_NO_THROW(firmwareInfo->update());

    EXPECT_EQ(getProperty<std::string>(softwarePath(name),
                                       "xyz.openbmc_project.Software.Version",
                                       "Version"),
              "01.00");
}

} // namespace
