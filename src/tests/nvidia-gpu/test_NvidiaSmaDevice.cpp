/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaSensorConfig.hpp"
#include "NvidiaSmaDevice.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr uint8_t defaultEid = 20;

// Short enough that a second poll round lands well inside pollTimeout.
constexpr uint64_t fastPollMs = 10;

// Upper bound on how long the read loop may take to poll again.
constexpr std::chrono::seconds pollTimeout{5};

// Several fastPollMs intervals, so a loop that kept running would be caught.
constexpr std::chrono::seconds quietWindow{1};

class NvidiaSmaDeviceTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<SmaDevice> createDevice(
        const std::string& name = "SMA", uint8_t eid = defaultEid,
        uint64_t pollRate = sensorPollRateMs)
    {
        const std::string path = "/test/chassis/" + name;
        const SensorConfigs configs{.name = name, .pollRate = pollRate};
        return std::make_shared<SmaDevice>(configs, name, path, bus(), eid,
                                           ioContext(), requester(), objects(),
                                           gpu::DeviceCapabilities{});
    }
};

TEST_F(NvidiaSmaDeviceTest, ConstructorSetsPath)
{
    const std::string name = "sma_path";
    const std::shared_ptr<SmaDevice> device = createDevice(name);
    ASSERT_NE(device, nullptr);
    EXPECT_EQ(device->getPath(), "/test/chassis/" + name);
}

TEST_F(NvidiaSmaDeviceTest, InitSendsAtLeastOneRequest)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<SmaDevice> device = createDevice();
    device->init();
}

TEST_F(NvidiaSmaDeviceTest, ReadLoopStopsAfterDeviceIsDestroyed)
{
    int requests = 0;
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(
            [&requests](uint8_t /*eid*/, std::span<const uint8_t> /*reqMsg*/,
                        auto callback) {
                ++requests;
                callback(std::error_code{}, std::span<const uint8_t>{});
            });

    {
        const std::shared_ptr<SmaDevice> device =
            createDevice("sma_readloop", defaultEid, fastPollMs);
        device->init();

        // While the device is alive the poll timer keeps re-arming the loop,
        // so a later round has to produce more requests.
        const int afterInit = requests;
        ASSERT_TRUE(
            pumpIoUntil([&] { return requests > afterInit; }, pollTimeout));
    }

    // Once the device is gone the loop must stop: no further request may be
    // issued even after several more poll intervals have elapsed.
    const int afterDestroy = requests;
    EXPECT_FALSE(
        pumpIoUntil([&] { return requests > afterDestroy; }, quietWindow));
}

} // namespace
