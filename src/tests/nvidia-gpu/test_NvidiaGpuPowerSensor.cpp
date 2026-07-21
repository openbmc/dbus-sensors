/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaGpuPowerSensor.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"
#include "Thresholds.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

std::vector<uint8_t> buildPowerResponse(uint32_t powerMilliwatts)
{
    return test_utils::buildPlatformEnvSuccessResponse(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_POWER_DRAW,
        powerMilliwatts);
}

std::vector<uint8_t> buildPowerErrorResponse(uint8_t cc, uint16_t reasonCode)
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_POWER_DRAW, cc,
        reasonCode);
}

class NvidiaGpuPowerSensorTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaGpuPowerSensor> createSensor(
        uint8_t sensorId = gpuPowerSensorId,
        const std::string& name = "GPU_POWER",
        std::vector<thresholds::Threshold> thresholds = {},
        gpu::DeviceIdentification deviceType =
            gpu::DeviceIdentification::DEVICE_GPU,
        uint8_t eid = test_utils::defaultEid)
    {
        return std::make_shared<NvidiaGpuPowerSensor>(
            bus(), requester(), name, "/test/config", eid, sensorId, objects(),
            std::move(thresholds), deviceType);
    }

    static std::string powerPath(const std::string& name)
    {
        return "/xyz/openbmc_project/sensors/power/" + name;
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaGpuPowerSensorTest, ConstructorCreatesDbusInterfaces)
{
    const std::string name = "pwr_ctor_iface";
    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, name);
    const std::string path = powerPath(name);

    EXPECT_TRUE(std::isnan(getProperty<double>(
        path, "xyz.openbmc_project.Sensor.Value", "Value")));
    EXPECT_EQ(getProperty<double>(path, "xyz.openbmc_project.Sensor.Value",
                                  "MaxValue"),
              5000.0);
    EXPECT_EQ(getProperty<double>(path, "xyz.openbmc_project.Sensor.Value",
                                  "MinValue"),
              0.0);
    EXPECT_EQ(getProperty<std::string>(path, "xyz.openbmc_project.Sensor.Value",
                                       "Unit"),
              "xyz.openbmc_project.Sensor.Value.Unit.Watts");
}

// Constructor — PhysicalContext

TEST_F(NvidiaGpuPowerSensorTest, ConstructorWithGpuPhysicalContext)
{
    const std::string name = "pwr_gpu_ctx";
    const std::shared_ptr<NvidiaGpuPowerSensor> sensor = createSensor(
        gpuPowerSensorId, name, {}, gpu::DeviceIdentification::DEVICE_GPU);
    const std::string path = powerPath(name);

    EXPECT_EQ(
        getProperty<std::string>(
            path, "xyz.openbmc_project.Common.PhysicalContext", "Type"),
        "xyz.openbmc_project.Common.PhysicalContext.PhysicalContextType.Accelerator");
}

// Constructor — threshold interfaces

TEST_F(NvidiaGpuPowerSensorTest, ConstructorWithWarningThreshold)
{
    const std::string name = "pwr_warn_thr";
    const std::shared_ptr<NvidiaGpuPowerSensor> sensor = createSensor(
        gpuPowerSensorId, name,
        {thresholds::Threshold{thresholds::Level::WARNING,
                               thresholds::Direction::HIGH, 85.0}});
    const std::string path = powerPath(name);

    EXPECT_EQ(getProperty<double>(
                  path, "xyz.openbmc_project.Sensor.Threshold.Warning",
                  "WarningHigh"),
              85.0);
}

TEST_F(NvidiaGpuPowerSensorTest, ConstructorWithCriticalThreshold)
{
    const std::string name = "pwr_crit_thr";
    const std::shared_ptr<NvidiaGpuPowerSensor> sensor = createSensor(
        gpuPowerSensorId, name,
        {thresholds::Threshold{thresholds::Level::CRITICAL,
                               thresholds::Direction::HIGH, 100.0}});
    const std::string path = powerPath(name);

    EXPECT_EQ(getProperty<double>(
                  path, "xyz.openbmc_project.Sensor.Threshold.Critical",
                  "CriticalHigh"),
              100.0);
}

// Update — successful power readings

TEST_F(NvidiaGpuPowerSensorTest, UpdateSuccessValidPower)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildPowerResponse(150000)));

    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_valid");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 150.0);
}

TEST_F(NvidiaGpuPowerSensorTest, UpdateSuccessZeroPower)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildPowerResponse(0)));

    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_zero");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 0.0);
}

TEST_F(NvidiaGpuPowerSensorTest, UpdateSuccessFractionalPower)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildPowerResponse(1500)));

    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_frac");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 1.5);
}

TEST_F(NvidiaGpuPowerSensorTest, UpdateSuccessTwiceOverwritesValue)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildPowerResponse(150000)))
        .WillOnce(mock_mctp::respondWith({}, buildPowerResponse(200000)));

    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_twice");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 150.0);

    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 200.0);
}

// Update — error handling

TEST_F(NvidiaGpuPowerSensorTest, UpdateMctpTransportError)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_mctp_err");
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuPowerSensorTest, UpdateBadCompletionCodeError)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildPowerErrorResponse(
                    static_cast<uint8_t>(
                        ocp::accelerator_management::CompletionCode::ERROR),
                    0)));

    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_bad_cc");
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuPowerSensorTest, UpdateDecodeFailTruncatedBuffer)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, std::vector<uint8_t>(5, 0)));

    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_trunc");
    EXPECT_NO_THROW(sensor->update());
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuPowerSensorTest, UpdateDecodeFailEmptyBuffer)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_empty");
    EXPECT_NO_THROW(sensor->update());
    EXPECT_TRUE(std::isnan(sensor->value));
}

// Update — value retention after errors

TEST_F(NvidiaGpuPowerSensorTest, UpdateSuccessThenErrorKeepsPreviousValue)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildPowerResponse(150000)))
        .WillOnce(mock_mctp::respondWith(
            {}, buildPowerErrorResponse(
                    static_cast<uint8_t>(
                        ocp::accelerator_management::CompletionCode::ERROR),
                    0)));

    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_retain");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 150.0);

    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 150.0);
}

// Update — request encoding verification

TEST_F(NvidiaGpuPowerSensorTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, buildPowerResponse(150000)));

    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_eid_test", {},
                     gpu::DeviceIdentification::DEVICE_GPU, testEid);
    sensor->update();
}

// Destructor

TEST_F(NvidiaGpuPowerSensorTest, DestructorDoesNotCrash)
{
    std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_dtor");
    ASSERT_NE(sensor, nullptr);
    ASSERT_NE(sensor->sensorInterface, nullptr);
    sensor.reset();
    ASSERT_EQ(sensor, nullptr);
}

} // namespace
