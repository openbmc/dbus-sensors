/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaSensorConfig.hpp"
#include "NvidiaSmaDevice.hpp"
#include "OcpMctpVdm.hpp"

#include <MessagePackUnpackUtils.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

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

// A GetLeakDetectionInfo response reporting one leak sensor, so the device
// creates one.
std::vector<uint8_t> buildOneLeakSensorResponse()
{
    // numSensors + numThresholdLevels + one sensor of sensorId, leakState and
    // adcReadingMv, with no thresholds
    const uint16_t dataSize = 1 + 1 + (1 + 1 + 2);

    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + dataSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_LEAK_DETECTION_INFO));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(dataSize);
    pack.pack(static_cast<uint8_t>(1));  // numSensors
    pack.pack(static_cast<uint8_t>(0));  // numThresholdLevels
    pack.pack(static_cast<uint8_t>(0));  // sensorId
    pack.pack(static_cast<uint8_t>(0));  // leakState
    pack.pack(static_cast<uint16_t>(0)); // adcReadingMv
    return buf;
}

bool isLeakDetectionInfo(std::span<const uint8_t> request)
{
    return request[ocp::accelerator_management::messageHeaderSize] ==
           static_cast<uint8_t>(
               gpu::PlatformEnvironmentalCommands::GET_LEAK_DETECTION_INFO);
}

class NvidiaSmaDeviceTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<SmaDevice> createDevice(
        const std::string& name = "SMA", uint8_t eid = defaultEid,
        uint64_t pollRate = sensorPollRateMs)
    {
        const std::string path = "/test/chassis/" + name;
        const NvidiaMctpVdmConfig config{.path = path, .pollRate = pollRate};
        return std::make_shared<SmaDevice>(config, name, bus(), eid,
                                           ioContext(), requester(), objects());
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
    device->setOnline();
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
        device->setOnline();

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

// A device re-enumerated at a new EID keeps its sensors, so every one of them
// has to follow it there, the leak sensors included.
TEST_F(NvidiaSmaDeviceTest, SetEidRetargetsLeakSensors)
{
    constexpr uint8_t newEid = 30;
    std::vector<uint8_t> leakEids;
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault([&leakEids](uint8_t eid, std::span<const uint8_t> req,
                                   auto callback) {
            if (!isLeakDetectionInfo(req))
            {
                callback(std::make_error_code(std::errc::timed_out), {});
                return;
            }
            leakEids.push_back(eid);
            const std::vector<uint8_t> response = buildOneLeakSensorResponse();
            callback(std::error_code{}, response);
        });

    const std::shared_ptr<SmaDevice> device =
        createDevice("sma_set_eid", defaultEid, fastPollMs);
    device->init();
    ASSERT_TRUE(pumpIoUntil([&] { return !leakEids.empty(); }, pollTimeout));

    device->setEid(newEid);
    leakEids.clear();
    device->setOnline();

    ASSERT_TRUE(pumpIoUntil([&] { return !leakEids.empty(); }, pollTimeout));
    EXPECT_THAT(leakEids, testing::Each(newEid));
}

// An offline device stops polling, so every one of its sensors has to say so
// rather than keep its last reading, the leak sensors included.
TEST_F(NvidiaSmaDeviceTest, OfflineMarksLeakSensorsNonFunctional)
{
    bool leakQueried = false;
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault([&leakQueried](uint8_t /*eid*/,
                                      std::span<const uint8_t> req,
                                      auto callback) {
            if (!isLeakDetectionInfo(req))
            {
                callback(std::make_error_code(std::errc::timed_out), {});
                return;
            }
            leakQueried = true;
            const std::vector<uint8_t> response = buildOneLeakSensorResponse();
            callback(std::error_code{}, response);
        });

    const std::shared_ptr<SmaDevice> device =
        createDevice("sma_offline", defaultEid, fastPollMs);
    device->init();
    ASSERT_TRUE(pumpIoUntil([&] { return leakQueried; }, pollTimeout));

    const std::string leakPath =
        "/xyz/openbmc_project/sensors/voltage/sma_offline_LEAKDETECTOR_0";
    const std::string operationalStatus =
        "xyz.openbmc_project.State.Decorator.OperationalStatus";

    device->setOffline();
    EXPECT_FALSE(getProperty<bool>(leakPath, operationalStatus, "Functional"));

    device->setOnline();
    EXPECT_TRUE(getProperty<bool>(leakPath, operationalStatus, "Functional"));
}

} // namespace
