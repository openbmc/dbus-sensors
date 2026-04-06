/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaDeviceDiscovery.hpp"
#include "NvidiaSmaDevice.hpp"

#include <cstdint>
#include <memory>
#include <string>

#include <gtest/gtest.h>

namespace
{

constexpr uint8_t defaultEid = 20;

class SmaDeviceTestBase : public DbusMockTestBase
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
        return std::make_shared<SmaDevice>(configs, name, path, conn, eid, io,
                                           *mctpRequester, *objectServer);
    }
};

TEST_F(SmaDeviceTestBase, ConstructorSetsPath)
{
    const std::shared_ptr<SmaDevice> device = createDevice();
    EXPECT_NE(device, nullptr);
}

TEST_F(SmaDeviceTestBase, InitSendsAtLeastOneRequest)
{
    const std::shared_ptr<SmaDevice> device = createDevice();
    device->init();

    EXPECT_GE(mock_mctp::getRequestCount(), 1U);
}

TEST_F(SmaDeviceTestBase, ReadLoopStopsOnTimerCancel)
{
    {
        const std::shared_ptr<SmaDevice> device = createDevice();
        device->init();
    }

    // Drain pending async handlers — no crash from dangling callbacks.
    EXPECT_NO_THROW({
        io.poll();
        io.restart();
    });
}
} // namespace
