/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaSensorConfig.hpp"
#include "NvidiaSmaDevice.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <sdbusplus/exception.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr uint8_t defaultEid = 20;

constexpr uint8_t smaInternalSensorId = 17;

// Short enough that a second poll round lands well inside pollTimeout.
constexpr uint64_t fastPollMs = 10;

// Upper bound on how long the read loop may take to poll again.
constexpr std::chrono::seconds pollTimeout{5};

// Several fastPollMs intervals, so a loop that kept running would be caught.
constexpr std::chrono::seconds quietWindow{1};

// The sensor id a Get Temperature Reading request asks for, or nullopt for any
// other request.
std::optional<uint8_t> requestedTemperatureSensorId(
    std::span<const uint8_t> request)
{
    UnpackBuffer unpack(request);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    if (ocp::accelerator_management::unpackHeader(
            unpack, gpu::nvidiaPciVendorId, ocpMsgType, instanceId, msgType) !=
        0)
    {
        return std::nullopt;
    }

    uint8_t command = 0;
    uint8_t dataSize = 0;
    uint8_t sensorId = 0;
    unpack.unpack(command);
    unpack.unpack(dataSize);
    unpack.unpack(sensorId);

    if (unpack.getError() != 0 ||
        msgType !=
            static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL) ||
        command !=
            static_cast<uint8_t>(
                gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING))
    {
        return std::nullopt;
    }
    return sensorId;
}

class NvidiaSmaDeviceTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<SmaDevice> createDevice(
        const std::string& name = "SMA", uint8_t eid = defaultEid,
        uint64_t pollRate = sensorPollRateMs,
        const std::vector<uint8_t>& temperatureSensorIds = {
            smaInternalSensorId})
    {
        const std::string path = "/test/chassis/" + name;
        const EntityDeviceConfig config{
            .path = path, .name = name, .pollRate = pollRate};
        const SmaDeviceConfigs smaConfig{
            .temperatureSensorIds = temperatureSensorIds};
        return std::make_shared<SmaDevice>(config, smaConfig, bus(), eid,
                                           ioContext(), requester(), objects());
    }

    static double temperatureValue(const std::string& sensorName)
    {
        return getProperty<double>(
            "/xyz/openbmc_project/sensors/temperature/" + sensorName,
            "xyz.openbmc_project.Sensor.Value", "Value");
    }

    static std::string temperatureUnit(const std::string& sensorName)
    {
        return getProperty<std::string>(
            "/xyz/openbmc_project/sensors/temperature/" + sensorName,
            "xyz.openbmc_project.Sensor.Value", "Unit");
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

TEST_F(NvidiaSmaDeviceTest, PublishesATemperatureSensorPerConfiguredId)
{
    const std::shared_ptr<SmaDevice> device =
        createDevice("sma_temps", defaultEid, sensorPollRateMs,
                     {16, smaInternalSensorId, 200});
    device->init();

    const std::string degrees =
        "xyz.openbmc_project.Sensor.Value.Unit.DegreesC";
    EXPECT_EQ(temperatureUnit("sma_temps_SMA_Ext"), degrees);
    EXPECT_EQ(temperatureUnit("sma_temps_SMA_Internal"), degrees);
    EXPECT_EQ(temperatureUnit("sma_temps_TEMP_200"), degrees);
}

TEST_F(NvidiaSmaDeviceTest, EachTemperatureSensorReadsItsOwnId)
{
    // Answer each temperature request with the id it asked for, so a
    // sensor's reading shows which id it polled.
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault([](uint8_t /*eid*/, std::span<const uint8_t> reqMsg,
                          auto callback) {
            const std::optional<uint8_t> sensorId =
                requestedTemperatureSensorId(reqMsg);
            if (!sensorId)
            {
                callback(std::error_code{}, std::span<const uint8_t>{});
                return;
            }
            const std::vector<uint8_t> response =
                test_utils::buildPlatformEnvSuccessResponse(
                    gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING,
                    static_cast<int32_t>(*sensorId) * 256);
            callback(std::error_code{}, response);
        });

    const std::shared_ptr<SmaDevice> device =
        createDevice("sma_poll", defaultEid, sensorPollRateMs, {16, 100});
    device->init();
    device->setOnline();

    EXPECT_EQ(temperatureValue("sma_poll_SMA_Ext"), 16.0);
    EXPECT_EQ(temperatureValue("sma_poll_TEMP_100"), 100.0);
}

TEST_F(NvidiaSmaDeviceTest, PublishesNoTemperatureSensorWhenNoneConfigured)
{
    std::vector<uint8_t> requestedIds;
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(
            [&requestedIds](uint8_t /*eid*/, std::span<const uint8_t> reqMsg,
                            auto callback) {
                const std::optional<uint8_t> sensorId =
                    requestedTemperatureSensorId(reqMsg);
                if (sensorId)
                {
                    requestedIds.push_back(*sensorId);
                }
                callback(std::error_code{}, std::span<const uint8_t>{});
            });

    const std::shared_ptr<SmaDevice> device =
        createDevice("sma_no_temps", defaultEid, sensorPollRateMs, {});
    device->init();
    device->setOnline();

    EXPECT_TRUE(requestedIds.empty());
    EXPECT_THROW(temperatureUnit("sma_no_temps_SMA_Internal"),
                 sdbusplus::exception_t);
}

} // namespace
