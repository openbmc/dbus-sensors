/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuDevice.hpp"
#include "NvidiaSensorConfig.hpp"

#include <cstdint>
#include <memory>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr uint8_t defaultEid = 20;

class NvidiaGpuDeviceTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<GpuDevice> createDevice(
        const std::string& name = "GPU_DEV", uint8_t eid = defaultEid,
        uint64_t pollRate = 1000)
    {
        const std::string path = "/test/gpu/" + name;
        const SensorConfigs configs{.name = name, .pollRate = pollRate};
        return std::make_shared<GpuDevice>(configs, name, path, bus(), eid,
                                           ioContext(), requester(), objects());
    }
};

// Constructor

TEST_F(NvidiaGpuDeviceTest, ConstructorDoesNotCrash)
{
    const std::shared_ptr<GpuDevice> device = createDevice("gpudev_ctor");
    EXPECT_NE(device, nullptr);
}

TEST_F(NvidiaGpuDeviceTest, GetPathReturnsConfiguredPath)
{
    const std::string name = "gpudev_path";
    const std::shared_ptr<GpuDevice> device = createDevice(name);
    EXPECT_EQ(device->getPath(), "/test/gpu/" + name);
}

// Init

TEST_F(NvidiaGpuDeviceTest, InitSendsAtLeastOneRequest)
{
    // init() sends one request per sensor/metric; complete each with an
    // empty response so any chained requests still make progress.
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<GpuDevice> device = createDevice("gpudev_init");
    device->init();
}

// ReadLoop

TEST_F(NvidiaGpuDeviceTest, ReadLoopStopsOnTimerCancel)
{
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(mock_mctp::respondWith({}, {}));

    {
        const std::shared_ptr<GpuDevice> device =
            createDevice("gpudev_readloop");
        device->init();
    }

    EXPECT_NO_THROW(drainPendingAsync());
}

} // namespace
