/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Inventory.hpp"
#include "MockDbusHelper.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

namespace
{

constexpr uint8_t defaultEid = 10;

class InventoryTestBase : public DbusMockTestBase
{
  protected:
    static std::shared_ptr<Inventory> createInventory(
        const std::string& name = "GPU_INV",
        gpu::DeviceIdentification deviceType =
            gpu::DeviceIdentification::DEVICE_GPU,
        uint8_t eid = defaultEid)
    {
        return std::make_shared<Inventory>(conn, *objectServer, name,
                                           *mctpRequester, deviceType, eid, io,
                                           nullptr);
    }
};

// Constructor — D-Bus interface creation

TEST_F(InventoryTestBase, ConstructorCreatesAssetInterface)
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

TEST_F(InventoryTestBase, InitSendsAtLeastOneRequest)
{
    const std::shared_ptr<Inventory> inv = createInventory("inv_init");
    inv->init();
    EXPECT_GE(mock_mctp::getRequestCount(), 1U);
}

// Error handling

TEST_F(InventoryTestBase, UpdateMctpTransportErrorNoCrash)
{
    mock_mctp::setNextResponse(std::make_error_code(std::errc::timed_out), {});
    const std::shared_ptr<Inventory> inv = createInventory("inv_mctp_err");
    EXPECT_NO_THROW(inv->init());
}

TEST_F(InventoryTestBase, UpdateBadCompletionCodeNoCrash)
{
    mock_mctp::setNextResponse({}, std::vector<uint8_t>(5, 0));
    const std::shared_ptr<Inventory> inv = createInventory("inv_bad_cc");
    EXPECT_NO_THROW(inv->init());
}

TEST_F(InventoryTestBase, UpdateEmptyBufferNoCrash)
{
    mock_mctp::setNextResponse({}, {});
    const std::shared_ptr<Inventory> inv = createInventory("inv_empty");
    EXPECT_NO_THROW(inv->init());
}

} // namespace
