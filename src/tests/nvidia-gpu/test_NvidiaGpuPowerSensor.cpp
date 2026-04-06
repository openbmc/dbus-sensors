/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaGpuPowerSensor.hpp"
#include "OcpMctpVdm.hpp"
#include "Thresholds.hpp"

#include <MessagePackUnpackUtils.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace
{

constexpr uint8_t defaultEid = 10;
constexpr size_t powerResponseSize = 15;
constexpr size_t errorResponseSize = 9;

std::vector<uint8_t> buildPowerResponse(uint32_t powerMilliwatts)
{
    std::vector<uint8_t> buf(powerResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(0x03)); // command: GET_CURRENT_POWER_DRAW
    pack.pack(static_cast<uint8_t>(0x00)); // cc: SUCCESS
    pack.pack(static_cast<uint16_t>(0));   // reserved
    pack.pack(static_cast<uint16_t>(4));   // data_size: sizeof(uint32_t)
    pack.pack(powerMilliwatts);
    return buf;
}

std::vector<uint8_t> buildPowerErrorResponse(uint8_t cc, uint16_t reasonCode)
{
    std::vector<uint8_t> buf(errorResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(0x03)); // command: GET_CURRENT_POWER_DRAW
    pack.pack(cc);
    pack.pack(reasonCode);
    return buf;
}

class NvidiaGpuPowerSensorTestBase : public DbusMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaGpuPowerSensor> createSensor(
        uint8_t sensorId = gpuPowerSensorId,
        const std::string& name = "GPU_POWER",
        std::vector<thresholds::Threshold> thresholds = {},
        gpu::DeviceIdentification deviceType =
            gpu::DeviceIdentification::DEVICE_GPU,
        uint8_t eid = defaultEid)
    {
        return std::make_shared<NvidiaGpuPowerSensor>(
            conn, *mctpRequester, name, "/test/config", eid, sensorId,
            *objectServer, std::move(thresholds), deviceType);
    }

    static std::string powerPath(const std::string& name)
    {
        return "/xyz/openbmc_project/sensors/power/" + name;
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaGpuPowerSensorTestBase, ConstructorCreatesDbusInterfaces)
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

TEST_F(NvidiaGpuPowerSensorTestBase, ConstructorWithGpuPhysicalContext)
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

TEST_F(NvidiaGpuPowerSensorTestBase, ConstructorWithWarningThreshold)
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

TEST_F(NvidiaGpuPowerSensorTestBase, ConstructorWithCriticalThreshold)
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

TEST_F(NvidiaGpuPowerSensorTestBase, UpdateSuccessValidPower)
{
    mock_mctp::setNextResponse({}, buildPowerResponse(150000));
    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_valid");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 150.0);
}

TEST_F(NvidiaGpuPowerSensorTestBase, UpdateSuccessZeroPower)
{
    mock_mctp::setNextResponse({}, buildPowerResponse(0));
    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_zero");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 0.0);
}

TEST_F(NvidiaGpuPowerSensorTestBase, UpdateSuccessFractionalPower)
{
    mock_mctp::setNextResponse({}, buildPowerResponse(1500));
    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_frac");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 1.5);
}

TEST_F(NvidiaGpuPowerSensorTestBase, UpdateSuccessTwiceOverwritesValue)
{
    mock_mctp::setNextResponse({}, buildPowerResponse(150000));
    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_twice");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 150.0);

    mock_mctp::setNextResponse({}, buildPowerResponse(200000));
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 200.0);
}

// Update — error handling

TEST_F(NvidiaGpuPowerSensorTestBase, UpdateMctpTransportError)
{
    mock_mctp::setNextResponse(std::make_error_code(std::errc::timed_out), {});
    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_mctp_err");
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuPowerSensorTestBase, UpdateBadCompletionCodeError)
{
    mock_mctp::setNextResponse(
        {}, buildPowerErrorResponse(
                static_cast<uint8_t>(
                    ocp::accelerator_management::CompletionCode::ERROR),
                0));
    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_bad_cc");
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuPowerSensorTestBase, UpdateDecodeFailTruncatedBuffer)
{
    mock_mctp::setNextResponse({}, std::vector<uint8_t>(5, 0));
    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_trunc");
    EXPECT_NO_THROW(sensor->update());
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuPowerSensorTestBase, UpdateDecodeFailEmptyBuffer)
{
    mock_mctp::setNextResponse({}, {});
    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_empty");
    EXPECT_NO_THROW(sensor->update());
    EXPECT_TRUE(std::isnan(sensor->value));
}

// Update — value retention after errors

TEST_F(NvidiaGpuPowerSensorTestBase, UpdateSuccessThenErrorKeepsPreviousValue)
{
    mock_mctp::setNextResponse({}, buildPowerResponse(150000));
    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_retain");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 150.0);

    mock_mctp::setNextResponse(
        {}, buildPowerErrorResponse(
                static_cast<uint8_t>(
                    ocp::accelerator_management::CompletionCode::ERROR),
                0));
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 150.0);
}

// Update — request encoding verification

TEST_F(NvidiaGpuPowerSensorTestBase, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    mock_mctp::setNextResponse({}, buildPowerResponse(150000));
    const std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_eid_test", {},
                     gpu::DeviceIdentification::DEVICE_GPU, testEid);
    sensor->update();
    EXPECT_EQ(mock_mctp::getLastEid(), testEid);
}

// Destructor

TEST_F(NvidiaGpuPowerSensorTestBase, DestructorDoesNotCrash)
{
    std::shared_ptr<NvidiaGpuPowerSensor> sensor =
        createSensor(gpuPowerSensorId, "pwr_dtor");
    ASSERT_NE(sensor, nullptr);
    ASSERT_NE(sensor->sensorInterface, nullptr);
    sensor.reset();
    ASSERT_EQ(sensor, nullptr);
}

} // namespace
