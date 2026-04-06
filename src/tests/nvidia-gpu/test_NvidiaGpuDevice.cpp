/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaDeviceDiscovery.hpp"
#include "NvidiaGpuDevice.hpp"

#include <cstdint>
#include <memory>
#include <string>

#include <gtest/gtest.h>

namespace
{

constexpr uint8_t defaultEid = 20;

class NvidiaGpuDeviceTestBase : public DbusMockTestBase
{
  protected:
    static std::shared_ptr<GpuDevice> createDevice(
        const std::string& name = "GPU_DEV", uint8_t eid = defaultEid,
        uint64_t pollRate = 1000)
    {
        const std::string path = "/test/gpu/" + name;
        const SensorConfigs configs{.name = name, .pollRate = pollRate};
        return std::make_shared<GpuDevice>(configs, name, path, conn, eid, io,
                                           *mctpRequester, *objectServer);
    }
};

// Constructor

TEST_F(NvidiaGpuDeviceTestBase, ConstructorDoesNotCrash)
{
    const std::shared_ptr<GpuDevice> device = createDevice("gpudev_ctor");
    EXPECT_NE(device, nullptr);
}

TEST_F(NvidiaGpuDeviceTestBase, GetPathReturnsConfiguredPath)
{
    const std::string name = "gpudev_path";
    const std::shared_ptr<GpuDevice> device = createDevice(name);
    EXPECT_EQ(device->getPath(), "/test/gpu/" + name);
}

// Init

TEST_F(NvidiaGpuDeviceTestBase, InitSendsAtLeastOneRequest)
{
    const std::shared_ptr<GpuDevice> device = createDevice("gpudev_init");
    device->init();
    EXPECT_GE(mock_mctp::getRequestCount(), 1U);
}

// ReadLoop

TEST_F(NvidiaGpuDeviceTestBase, ReadLoopStopsOnTimerCancel)
{
    {
        const std::shared_ptr<GpuDevice> device =
            createDevice("gpudev_readloop");
        device->init();
    }

    EXPECT_NO_THROW(drainPendingAsync());
}

} // namespace
