/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaDeviceDiscovery.hpp"
#include "NvidiaPcieDevice.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <system_error>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr uint8_t defaultEid = 20;

class NvidiaPcieDeviceTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<PcieDevice> createDevice(
        const std::string& name = "PCIE_DEV", uint8_t eid = defaultEid,
        uint64_t pollRate = 1000, uint64_t nicNetworkPortCount = 0)
    {
        const std::string path = "/test/pcie/" + name;
        const SensorConfigs configs{.name = name,
                                    .pollRate = pollRate,
                                    .nicNetworkPortCount = nicNetworkPortCount};
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

} // namespace
