/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Inventory.hpp"
#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr const char* versionIfaceName = "xyz.openbmc_project.Software.Version";

// A payload of at least 16 bytes decodes cleanly for every queried property
// type, so one canned response can satisfy the whole property set.
constexpr const char* firmwareVersionText = "FW-VERSION-TEST-01";

std::vector<uint8_t> buildInventoryErrorResponse(uint8_t cc,
                                                 uint16_t reasonCode)
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION, cc,
        reasonCode);
}

// Recover the queried property ID from a GET_INVENTORY_INFORMATION request,
// or nullopt if the buffer is not one. Inventory queries several properties,
// so the tests need to pick the FIRMWARE_VERSION request out of the stream.
std::optional<uint8_t> decodeRequestedPropertyId(
    std::span<const uint8_t> request)
{
    if (request.size() != gpu::getInventoryInformationRequestSize)
    {
        return std::nullopt;
    }

    UnpackBuffer unpack(request);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    if (ocp::accelerator_management::unpackHeader(
            unpack, gpu::nvidiaPciVendorId, ocpMsgType, instanceId, msgType) !=
            0 ||
        ocpMsgType != ocp::accelerator_management::MessageType::REQUEST ||
        msgType !=
            static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL))
    {
        return std::nullopt;
    }

    uint8_t command = 0;
    uint8_t dataSize = 0;
    uint8_t propertyId = 0;
    unpack.unpack(command);
    unpack.unpack(dataSize);
    unpack.unpack(propertyId);
    if (unpack.getError() != 0 || dataSize != 1 ||
        command !=
            static_cast<uint8_t>(
                gpu::PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION))
    {
        return std::nullopt;
    }
    return propertyId;
}

class InventoryTest : public MctpMockTestBase
{
  protected:
    // GpuDevice creates and initializes the Software.Version interface and
    // hands it to Inventory to drive; mirror that split here. Tests that do
    // not call this pass a null interface, as a non-GPU device would.
    void makeFirmwareVersionInterface(const std::string& name)
    {
        firmwareVersionIface =
            objects().add_interface(softwarePath(name), versionIfaceName);
        firmwareVersionIface->register_property<std::string>("Version", "");
        firmwareVersionIface->register_property<std::string>(
            "Purpose",
            "xyz.openbmc_project.Software.Version.VersionPurpose.Other");
        ASSERT_TRUE(firmwareVersionIface->initialize());
    }

    std::shared_ptr<Inventory> createInventory(
        const std::string& name = "GPU_INV",
        gpu::DeviceIdentification deviceType =
            gpu::DeviceIdentification::DEVICE_GPU,
        uint8_t eid = test_utils::defaultEid)
    {
        return std::make_shared<Inventory>(
            bus(), objects(), name, requester(), deviceType, eid, ioContext(),
            nullptr, nullptr, firmwareVersionIface);
    }

    static std::string softwarePath(const std::string& name)
    {
        return "/xyz/openbmc_project/software/" + name + "_Firmware";
    }

    std::shared_ptr<sdbusplus::asio::dbus_interface> firmwareVersionIface;
};

// Constructor — D-Bus interface creation

TEST_F(InventoryTest, ConstructorCreatesAssetInterface)
{
    const std::string name = "inv_ctor";
    const std::shared_ptr<Inventory> inv = createInventory(name);
    const std::string path = "/xyz/openbmc_project/inventory/" + name;

    EXPECT_EQ(getProperty<std::string>(
                  path, "xyz.openbmc_project.Inventory.Decorator.Asset",
                  "Manufacturer"),
              "NVIDIA");
}

// Init — sends requests

TEST_F(InventoryTest, InitSendsAtLeastOneRequest)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<Inventory> inv = createInventory("inv_init");
    inv->init();
}

// Init — decoded values populate the Asset properties

TEST_F(InventoryTest, InitSuccessSetsAssetProperty)
{
    // Answer every property request with the same valid string response so the
    // whole property set resolves in one synchronous pass and the decode ->
    // set_property path is exercised for the Asset interface. A payload of at
    // least 16 bytes decodes cleanly for every queried property type: a string
    // for the Asset fields, a valid GUID for the UUID interface, and a uint32
    // for the clock properties.
    const std::string inventoryText = "NVIDIA-INV-TEST-01";
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(mock_mctp::respondWith(
            {},
            test_utils::buildPlatformEnvStringResponse(
                gpu::PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION,
                inventoryText)));

    const std::string name = "inv_success";
    const std::shared_ptr<Inventory> inv = createInventory(name);
    inv->init();

    const std::string path = "/xyz/openbmc_project/inventory/" + name;
    const std::string assetIface =
        "xyz.openbmc_project.Inventory.Decorator.Asset";
    EXPECT_EQ(getProperty<std::string>(path, assetIface, "SerialNumber"),
              inventoryText);
    EXPECT_EQ(getProperty<std::string>(path, assetIface, "PartNumber"),
              inventoryText);
    EXPECT_EQ(getProperty<std::string>(path, assetIface, "Model"),
              inventoryText);
}

// Init — the firmware version reaches the Software.Version object

TEST_F(InventoryTest, InitSuccessSetsFirmwareVersion)
{
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(mock_mctp::respondWith(
            {},
            test_utils::buildPlatformEnvStringResponse(
                gpu::PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION,
                firmwareVersionText)));

    const std::string name = "inv_fw";
    makeFirmwareVersionInterface(name);
    const std::shared_ptr<Inventory> inv = createInventory(name);
    inv->init();

    EXPECT_EQ(getProperty<std::string>(softwarePath(name), versionIfaceName,
                                       "Version"),
              firmwareVersionText);
}

TEST_F(InventoryTest, InitRequestsFirmwareVersionProperty)
{
    std::vector<uint8_t> requestedPropertyIds;
    const std::vector<uint8_t> response =
        test_utils::buildPlatformEnvStringResponse(
            gpu::PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION,
            firmwareVersionText);
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault([&](uint8_t /*eid*/, std::span<const uint8_t> reqMsg,
                           auto callback) {
            const std::optional<uint8_t> propertyId =
                decodeRequestedPropertyId(reqMsg);
            if (propertyId)
            {
                requestedPropertyIds.push_back(*propertyId);
            }
            callback(std::error_code{}, response);
        });

    const std::string name = "inv_fw_req";
    makeFirmwareVersionInterface(name);
    const std::shared_ptr<Inventory> inv = createInventory(name);
    inv->init();

    EXPECT_THAT(requestedPropertyIds,
                testing::Contains(static_cast<uint8_t>(
                    gpu::InventoryPropertyId::FIRMWARE_VERSION)));
}

// A rejected response must leave the property at its registered default
// rather than publishing a decoded-from-nothing value.
TEST_F(InventoryTest, InitBadCompletionCodeLeavesFirmwareVersionUnset)
{
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(mock_mctp::respondWith(
            {}, buildInventoryErrorResponse(
                    static_cast<uint8_t>(
                        ocp::accelerator_management::CompletionCode::ERROR),
                    0)));

    const std::string name = "inv_fw_bad_cc";
    makeFirmwareVersionInterface(name);
    const std::shared_ptr<Inventory> inv = createInventory(name);
    EXPECT_NO_THROW(inv->init());

    EXPECT_EQ(getProperty<std::string>(softwarePath(name), versionIfaceName,
                                       "Version"),
              "");
}

// Error handling — init() must not crash on failed responses

TEST_F(InventoryTest, InitMctpTransportErrorNoCrash)
{
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<Inventory> inv = createInventory("inv_mctp_err");
    EXPECT_NO_THROW(inv->init());
}

TEST_F(InventoryTest, InitBadCompletionCodeNoCrash)
{
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(mock_mctp::respondWith(
            {}, buildInventoryErrorResponse(
                    static_cast<uint8_t>(
                        ocp::accelerator_management::CompletionCode::ERROR),
                    0)));

    const std::shared_ptr<Inventory> inv = createInventory("inv_bad_cc");
    EXPECT_NO_THROW(inv->init());
}

TEST_F(InventoryTest, InitMalformedResponseNoCrash)
{
    // A short all-zero buffer fails common-header validation before any
    // completion code is read.
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(mock_mctp::respondWith({}, std::vector<uint8_t>(5, 0)));

    const std::shared_ptr<Inventory> inv = createInventory("inv_malformed");
    EXPECT_NO_THROW(inv->init());
}

TEST_F(InventoryTest, InitEmptyBufferNoCrash)
{
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<Inventory> inv = createInventory("inv_empty");
    EXPECT_NO_THROW(inv->init());
}

} // namespace
