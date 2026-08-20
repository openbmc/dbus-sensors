/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaPcieDevice.hpp"
#include "NvidiaSensorConfig.hpp"
#include "TestUtils.hpp"

#include <sdbusplus/exception.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <system_error>

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
        const SensorConfigs configs{.name = name,
                                    .pollRate = pollRate,
                                    .networkPortCount = networkPortCount};
        return std::make_shared<PcieDevice>(configs, name, path, bus(), eid,
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

} // namespace
