/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaGpuVoltageSensor.hpp"
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

std::vector<uint8_t> buildVoltageResponse(uint32_t voltageMicrovolts)
{
    return test_utils::buildPlatformEnvSuccessResponse(
        gpu::PlatformEnvironmentalCommands::GET_VOLTAGE, voltageMicrovolts);
}

std::vector<uint8_t> buildVoltageErrorResponse(uint8_t cc, uint16_t reasonCode)
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::GET_VOLTAGE, cc, reasonCode);
}

class NvidiaGpuVoltageSensorTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaGpuVoltageSensor> createSensor(
        uint8_t sensorId = gpuVoltageSensorId,
        const std::string& name = "GPU_VOLTAGE",
        std::vector<thresholds::Threshold> thresholds = {},
        gpu::DeviceIdentification deviceType =
            gpu::DeviceIdentification::DEVICE_GPU,
        uint8_t eid = test_utils::defaultEid)
    {
        return std::make_shared<NvidiaGpuVoltageSensor>(
            bus(), requester(), name, "/test/config", eid, sensorId, objects(),
            std::move(thresholds), deviceType);
    }

    static std::string voltagePath(const std::string& name)
    {
        return "/xyz/openbmc_project/sensors/voltage/" + name;
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaGpuVoltageSensorTest, ConstructorCreatesDbusInterfaces)
{
    const std::string name = "volt_ctor_iface";
    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, name);
    const std::string path = voltagePath(name);

    EXPECT_TRUE(std::isnan(getProperty<double>(
        path, "xyz.openbmc_project.Sensor.Value", "Value")));
    EXPECT_EQ(getProperty<double>(path, "xyz.openbmc_project.Sensor.Value",
                                  "MaxValue"),
              50.0);
    EXPECT_EQ(getProperty<double>(path, "xyz.openbmc_project.Sensor.Value",
                                  "MinValue"),
              0.0);
    EXPECT_EQ(getProperty<std::string>(path, "xyz.openbmc_project.Sensor.Value",
                                       "Unit"),
              "xyz.openbmc_project.Sensor.Value.Unit.Volts");
}

// Constructor — PhysicalContext

TEST_F(NvidiaGpuVoltageSensorTest, ConstructorWithGpuPhysicalContext)
{
    const std::string name = "volt_gpu_ctx";
    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor = createSensor(
        gpuVoltageSensorId, name, {}, gpu::DeviceIdentification::DEVICE_GPU);
    const std::string path = voltagePath(name);

    EXPECT_EQ(
        getProperty<std::string>(
            path, "xyz.openbmc_project.Common.PhysicalContext", "Type"),
        "xyz.openbmc_project.Common.PhysicalContext.PhysicalContextType.Accelerator");
}

// Update — successful voltage readings

TEST_F(NvidiaGpuVoltageSensorTest, UpdateSuccessValidVoltage)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildVoltageResponse(12000000)));

    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_valid");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 12.0);
}

TEST_F(NvidiaGpuVoltageSensorTest, UpdateSuccessZeroVoltage)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildVoltageResponse(0)));

    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_zero");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 0.0);
}

TEST_F(NvidiaGpuVoltageSensorTest, UpdateSuccessFractionalVoltage)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildVoltageResponse(1500000)));

    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_frac");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 1.5);
}

TEST_F(NvidiaGpuVoltageSensorTest, UpdateSuccessTwiceOverwritesValue)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildVoltageResponse(12000000)))
        .WillOnce(mock_mctp::respondWith({}, buildVoltageResponse(5000000)));

    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_twice");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 12.0);

    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 5.0);
}

// Update — error handling

TEST_F(NvidiaGpuVoltageSensorTest, UpdateMctpTransportError)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_mctp_err");
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuVoltageSensorTest, UpdateBadCompletionCodeError)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildVoltageErrorResponse(
                    static_cast<uint8_t>(
                        ocp::accelerator_management::CompletionCode::ERROR),
                    0)));

    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_bad_cc");
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuVoltageSensorTest, UpdateDecodeFailTruncatedBuffer)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, std::vector<uint8_t>(5, 0)));

    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_trunc");
    EXPECT_NO_THROW(sensor->update());
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuVoltageSensorTest, UpdateDecodeFailEmptyBuffer)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_empty");
    EXPECT_NO_THROW(sensor->update());
    EXPECT_TRUE(std::isnan(sensor->value));
}

// Update — value retention after errors

TEST_F(NvidiaGpuVoltageSensorTest, UpdateSuccessThenErrorKeepsPreviousValue)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildVoltageResponse(12000000)))
        .WillOnce(mock_mctp::respondWith(
            {}, buildVoltageErrorResponse(
                    static_cast<uint8_t>(
                        ocp::accelerator_management::CompletionCode::ERROR),
                    0)));

    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_retain");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 12.0);

    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 12.0);
}

// Update — request encoding verification

TEST_F(NvidiaGpuVoltageSensorTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, buildVoltageResponse(12000000)));

    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_eid_test", {},
                     gpu::DeviceIdentification::DEVICE_GPU, testEid);
    sensor->update();
}

// Destructor

TEST_F(NvidiaGpuVoltageSensorTest, DestructorDoesNotCrash)
{
    std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_dtor");
    ASSERT_NE(sensor, nullptr);
    ASSERT_NE(sensor->sensorInterface, nullptr);
    sensor.reset();
    ASSERT_EQ(sensor, nullptr);
}

} // namespace
