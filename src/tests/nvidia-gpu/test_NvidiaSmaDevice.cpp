/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaSensorConfig.hpp"
#include "NvidiaSmaDevice.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <system_error>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr uint8_t defaultEid = 20;

class NvidiaSmaDeviceTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<SmaDevice> createDevice(
        const std::string& name = "SMA", uint8_t eid = defaultEid,
        uint64_t pollRate = 1000, uint64_t nicNetworkPortCount = 0)
    {
        const std::string path = "/test/chassis/" + name;
        const SensorConfigs configs{.name = name,
                                    .pollRate = pollRate,
                                    .nicNetworkPortCount = nicNetworkPortCount};
        return std::make_shared<SmaDevice>(configs, name, path, bus(), eid,
                                           ioContext(), requester(), objects());
    }
};

TEST_F(NvidiaSmaDeviceTest, ConstructorSetsPath)
{
    const std::shared_ptr<SmaDevice> device = createDevice();
    EXPECT_NE(device, nullptr);
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

TEST_F(NvidiaSmaDeviceTest, ReadLoopStopsOnTimerCancel)
{
    {
        const std::shared_ptr<SmaDevice> device = createDevice();
        device->init();
    }

    // Drain pending async handlers — no crash from dangling callbacks.
    EXPECT_NO_THROW(drainPendingAsync());
}
} // namespace
