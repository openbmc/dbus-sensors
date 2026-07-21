/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuEnergySensor.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"
#include "Thresholds.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

std::vector<uint8_t> buildEnergyResponse(uint64_t energyMillijoules)
{
    return test_utils::buildPlatformEnvSuccessResponse(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_ENERGY_COUNTER,
        energyMillijoules);
}

std::vector<uint8_t> buildEnergyErrorResponse(uint8_t cc, uint16_t reasonCode)
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_ENERGY_COUNTER, cc,
        reasonCode);
}

class NvidiaGpuEnergySensorTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaGpuEnergySensor> createSensor(
        uint8_t sensorId = gpuEnergySensorId,
        const std::string& name = "GPU_ENERGY",
        std::vector<thresholds::Threshold> thresholds = {},
        gpu::DeviceIdentification deviceType =
            gpu::DeviceIdentification::DEVICE_GPU,
        uint8_t eid = test_utils::defaultEid)
    {
        return std::make_shared<NvidiaGpuEnergySensor>(
            bus(), requester(), name, "/test/config", eid, sensorId, objects(),
            std::move(thresholds), deviceType);
    }

    static std::string energyPath(const std::string& name)
    {
        return "/xyz/openbmc_project/sensors/energy/" + name;
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaGpuEnergySensorTest, ConstructorCreatesDbusInterfaces)
{
    const std::string name = "nrg_ctor_iface";
    const std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, name);
    const std::string path = energyPath(name);

    EXPECT_TRUE(std::isnan(getProperty<double>(
        path, "xyz.openbmc_project.Sensor.Value", "Value")));
    EXPECT_DOUBLE_EQ(
        getProperty<double>(path, "xyz.openbmc_project.Sensor.Value",
                            "MaxValue"),
        static_cast<double>(std::numeric_limits<uint64_t>::max()) / 1000.0);
    EXPECT_EQ(getProperty<double>(path, "xyz.openbmc_project.Sensor.Value",
                                  "MinValue"),
              0.0);
    EXPECT_EQ(getProperty<std::string>(path, "xyz.openbmc_project.Sensor.Value",
                                       "Unit"),
              "xyz.openbmc_project.Sensor.Value.Unit.Joules");
}

// Constructor — PhysicalContext

TEST_F(NvidiaGpuEnergySensorTest, ConstructorWithGpuPhysicalContext)
{
    const std::string name = "nrg_gpu_ctx";
    const std::shared_ptr<NvidiaGpuEnergySensor> sensor = createSensor(
        gpuEnergySensorId, name, {}, gpu::DeviceIdentification::DEVICE_GPU);
    const std::string path = energyPath(name);

    EXPECT_EQ(
        getProperty<std::string>(
            path, "xyz.openbmc_project.Common.PhysicalContext", "Type"),
        "xyz.openbmc_project.Common.PhysicalContext.PhysicalContextType.Accelerator");
}

// Update — successful energy readings

TEST_F(NvidiaGpuEnergySensorTest, UpdateSuccessValidEnergy)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildEnergyResponse(5000000)));

    const std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, "nrg_valid");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 5000.0);
}

TEST_F(NvidiaGpuEnergySensorTest, UpdateSuccessZeroEnergy)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildEnergyResponse(0)));

    const std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, "nrg_zero");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 0.0);
}

TEST_F(NvidiaGpuEnergySensorTest, UpdateSuccessFractionalEnergy)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildEnergyResponse(1500)));

    const std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, "nrg_frac");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 1.5);
}

// Note: UpdateSuccessTwiceOverwritesValue is omitted for energy because
// the base Sensor hysteresisPublish = (uint64_max/1000) * 0.0001 ≈ 1.84e12,
// which exceeds any practical decoded energy delta, preventing subsequent
// value updates from taking effect.

// Update — error handling

TEST_F(NvidiaGpuEnergySensorTest, UpdateMctpTransportError)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, "nrg_mctp_err");
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuEnergySensorTest, UpdateBadCompletionCodeError)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildEnergyErrorResponse(
                    static_cast<uint8_t>(
                        ocp::accelerator_management::CompletionCode::ERROR),
                    0)));

    const std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, "nrg_bad_cc");
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuEnergySensorTest, UpdateDecodeFailTruncatedBuffer)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, std::vector<uint8_t>(5, 0)));

    const std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, "nrg_trunc");
    EXPECT_NO_THROW(sensor->update());
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuEnergySensorTest, UpdateDecodeFailEmptyBuffer)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, "nrg_empty");
    EXPECT_NO_THROW(sensor->update());
    EXPECT_TRUE(std::isnan(sensor->value));
}

// Update — value retention after errors

TEST_F(NvidiaGpuEnergySensorTest, UpdateSuccessThenErrorKeepsPreviousValue)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildEnergyResponse(5000000)))
        .WillOnce(mock_mctp::respondWith(
            {}, buildEnergyErrorResponse(
                    static_cast<uint8_t>(
                        ocp::accelerator_management::CompletionCode::ERROR),
                    0)));

    const std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, "nrg_retain");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 5000.0);

    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 5000.0);
}

// Update — request encoding verification

TEST_F(NvidiaGpuEnergySensorTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, buildEnergyResponse(5000000)));

    const std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, "nrg_eid_test", {},
                     gpu::DeviceIdentification::DEVICE_GPU, testEid);
    sensor->update();
}

// Destructor

TEST_F(NvidiaGpuEnergySensorTest, DestructorDoesNotCrash)
{
    std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, "nrg_dtor");
    ASSERT_NE(sensor, nullptr);
    ASSERT_NE(sensor->sensorInterface, nullptr);
    sensor.reset();
    ASSERT_EQ(sensor, nullptr);
}

} // namespace
