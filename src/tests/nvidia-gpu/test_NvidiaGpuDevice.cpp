/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuDevice.hpp"
#include "NvidiaSensorConfig.hpp"
#include "Utils.hpp"

#include <sdbusplus/exception.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <tuple>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr uint8_t defaultEid = 20;

constexpr const char* dimmIface = "xyz.openbmc_project.Inventory.Item.Dimm";

constexpr const char* versionIface = "xyz.openbmc_project.Software.Version";

// Short enough that a second poll round lands well inside pollTimeout.
constexpr uint64_t fastPollMs = 10;

// Upper bound on how long the read loop may take to poll again.
constexpr std::chrono::seconds pollTimeout{5};

// Several fastPollMs intervals, so a loop that kept running would be caught.
constexpr std::chrono::seconds quietWindow{1};

class NvidiaGpuDeviceTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<GpuDevice> createDevice(
        const std::string& name = "GPU_DEV", uint8_t eid = defaultEid,
        uint64_t pollRate = sensorPollRateMs)
    {
        const std::string path = "/test/gpu/" + name;
        const SensorConfigs configs{.name = name, .pollRate = pollRate};
        return std::make_shared<GpuDevice>(configs, name, path, bus(), eid,
                                           ioContext(), requester(), objects());
    }

    static std::string dramPath(const std::string& name)
    {
        return "/xyz/openbmc_project/inventory/" + name + "_DRAM_0";
    }

    static std::string firmwarePath(const std::string& name)
    {
        return "/xyz/openbmc_project/software/" + name + "_Firmware";
    }

    static std::string inventoryPath(const std::string& name)
    {
        return "/xyz/openbmc_project/inventory/" + name;
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

// The firmware object is published by the constructor, before any device
// response is decoded, so Version starts empty. Purpose has to be set for the
// Redfish FirmwareInventory entry to carry a Description.
TEST_F(NvidiaGpuDeviceTest, ConstructorCreatesFirmwareVersionInterface)
{
    const std::string name = "gpudev_fw";
    const std::shared_ptr<GpuDevice> device = createDevice(name);

    EXPECT_EQ(
        getProperty<std::string>(firmwarePath(name), versionIface, "Version"),
        "");
    EXPECT_EQ(
        getProperty<std::string>(firmwarePath(name), versionIface, "Purpose"),
        "xyz.openbmc_project.Software.Version.VersionPurpose.Other");
}

// bmcweb resolves Processor FirmwareVersion by following ran_on from the GPU
// inventory item, and fails the request if it lands on more than one object.
TEST_F(NvidiaGpuDeviceTest, ConstructorAssociatesFirmwareToInventoryItem)
{
    const std::string name = "gpudev_fw_assoc";
    const std::shared_ptr<GpuDevice> device = createDevice(name);

    const std::vector<Association> associations =
        getProperty<std::vector<Association>>(
            firmwarePath(name), association::interface, "Associations");

    ASSERT_EQ(associations.size(), 1U);
    EXPECT_EQ(std::get<0>(associations.front()), "running");
    EXPECT_EQ(std::get<1>(associations.front()), "ran_on");
    EXPECT_EQ(std::get<2>(associations.front()), inventoryPath(name));
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

TEST_F(NvidiaGpuDeviceTest, ReadLoopStopsAfterDeviceIsDestroyed)
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
        const std::shared_ptr<GpuDevice> device =
            createDevice("gpudev_readloop", defaultEid, fastPollMs);
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

// Destructor

TEST_F(NvidiaGpuDeviceTest, DestructorRemovesInterfaces)
{
    const std::string name = "gpudev_dtor";
    {
        const std::shared_ptr<GpuDevice> device = createDevice(name);
        ASSERT_NE(device, nullptr);
        // The DRAM Item.Dimm interface is published by the constructor.
        EXPECT_NO_THROW(
            getProperty<std::string>(dramPath(name), dimmIface, "MemoryType"));
    }
    drainPendingAsync();
    EXPECT_THROW(
        getProperty<std::string>(dramPath(name), dimmIface, "MemoryType"),
        sdbusplus::exception_t);
}

TEST_F(NvidiaGpuDeviceTest, DestructorRemovesFirmwareInterfaces)
{
    const std::string name = "gpudev_fw_dtor";
    {
        const std::shared_ptr<GpuDevice> device = createDevice(name);
        ASSERT_NE(device, nullptr);
        EXPECT_NO_THROW(getProperty<std::string>(firmwarePath(name),
                                                 versionIface, "Version"));
    }
    drainPendingAsync();
    EXPECT_THROW(
        getProperty<std::string>(firmwarePath(name), versionIface, "Version"),
        sdbusplus::exception_t);
    EXPECT_THROW(
        getProperty<std::vector<Association>>(
            firmwarePath(name), association::interface, "Associations"),
        sdbusplus::exception_t);
}

} // namespace
