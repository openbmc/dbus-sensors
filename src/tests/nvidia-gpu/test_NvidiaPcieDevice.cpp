/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaDeviceDiscovery.hpp"
#include "NvidiaPcieDevice.hpp"

#include <cstdint>
#include <memory>
#include <string>

#include <gtest/gtest.h>

namespace
{

constexpr uint8_t defaultEid = 20;

class NvidiaPcieDeviceTestBase : public DbusMockTestBase
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
        return std::make_shared<PcieDevice>(configs, name, path, conn, eid, io,
                                            *mctpRequester, *objectServer);
    }
};

// Constructor

TEST_F(NvidiaPcieDeviceTestBase, ConstructorDoesNotCrash)
{
    const std::shared_ptr<PcieDevice> device = createDevice("pciedev_ctor");
    EXPECT_NE(device, nullptr);
}

TEST_F(NvidiaPcieDeviceTestBase, GetPathReturnsConfiguredPath)
{
    const std::string name = "pciedev_path";
    const std::shared_ptr<PcieDevice> device = createDevice(name);
    EXPECT_EQ(device->getPath(), "/test/pcie/" + name);
}

// Init

TEST_F(NvidiaPcieDeviceTestBase, InitSendsAtLeastOneRequest)
{
    const std::shared_ptr<PcieDevice> device = createDevice("pciedev_init");
    device->init();
    EXPECT_GE(mock_mctp::getRequestCount(), 1U);
}

// ReadLoop

TEST_F(NvidiaPcieDeviceTestBase, ReadLoopStopsOnTimerCancel)
{
    {
        const std::shared_ptr<PcieDevice> device =
            createDevice("pciedev_readloop");
        device->init();
    }

    EXPECT_NO_THROW(drainPendingAsync());
}

} // namespace
