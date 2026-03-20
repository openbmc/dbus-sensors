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

#include <cstdint>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

// Build a SUCCESS GET_INVENTORY_INFORMATION response carrying an ASCII string
// payload. A payload of at least 16 bytes decodes cleanly for every queried
// property type: a string for the Asset fields, a valid GUID for the UUID
// interface, and a uint32 for the clock properties.
std::vector<uint8_t> buildInventoryStringResponse(const std::string& value)
{
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + value.size());
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0));            // reserved
    pack.pack(static_cast<uint16_t>(value.size())); // data_size
    for (const char c : value)
    {
        pack.pack(static_cast<uint8_t>(c));
    }
    return buf;
}

std::vector<uint8_t> buildInventoryErrorResponse(uint8_t cc,
                                                 uint16_t reasonCode)
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION, cc,
        reasonCode);
}

class InventoryTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<Inventory> createInventory(
        const std::string& name = "GPU_INV",
        gpu::DeviceIdentification deviceType =
            gpu::DeviceIdentification::DEVICE_GPU,
        uint8_t eid = test_utils::defaultEid)
    {
        return std::make_shared<Inventory>(bus(), objects(), name, requester(),
                                           deviceType, eid, ioContext(),
                                           nullptr, nullptr, nullptr, nullptr);
    }
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
    // set_property path is exercised for the Asset interface.
    const std::string inventoryText = "NVIDIA-INV-TEST-01";
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(mock_mctp::respondWith(
            {}, buildInventoryStringResponse(inventoryText)));

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
