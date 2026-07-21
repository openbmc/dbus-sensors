/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Inventory.hpp"
#include "MctpMockTestBase.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
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
                                           nullptr, nullptr);
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

// Error handling

TEST_F(InventoryTest, UpdateMctpTransportErrorNoCrash)
{
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<Inventory> inv = createInventory("inv_mctp_err");
    EXPECT_NO_THROW(inv->init());
}

TEST_F(InventoryTest, UpdateBadCompletionCodeNoCrash)
{
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(mock_mctp::respondWith({}, std::vector<uint8_t>(5, 0)));

    const std::shared_ptr<Inventory> inv = createInventory("inv_bad_cc");
    EXPECT_NO_THROW(inv->init());
}

TEST_F(InventoryTest, UpdateEmptyBufferNoCrash)
{
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<Inventory> inv = createInventory("inv_empty");
    EXPECT_NO_THROW(inv->init());
}

} // namespace
