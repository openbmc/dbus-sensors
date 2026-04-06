/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaGpuVoltageSensor.hpp"
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
constexpr size_t voltageResponseSize = 15;
constexpr size_t errorResponseSize = 9;

std::vector<uint8_t> buildVoltageResponse(uint32_t voltageMicrovolts)
{
    std::vector<uint8_t> buf(voltageResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(0x0F)); // command: GET_VOLTAGE
    pack.pack(static_cast<uint8_t>(0x00)); // cc: SUCCESS
    pack.pack(static_cast<uint16_t>(0));   // reserved
    pack.pack(static_cast<uint16_t>(4));   // data_size: sizeof(uint32_t)
    pack.pack(voltageMicrovolts);
    return buf;
}

std::vector<uint8_t> buildVoltageErrorResponse(uint8_t cc, uint16_t reasonCode)
{
    std::vector<uint8_t> buf(errorResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(0x0F)); // command: GET_VOLTAGE
    pack.pack(cc);
    pack.pack(reasonCode);
    return buf;
}

class NvidiaGpuVoltageSensorTestBase : public DbusMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaGpuVoltageSensor> createSensor(
        uint8_t sensorId = gpuVoltageSensorId,
        const std::string& name = "GPU_VOLTAGE",
        std::vector<thresholds::Threshold> thresholds = {},
        gpu::DeviceIdentification deviceType =
            gpu::DeviceIdentification::DEVICE_GPU,
        uint8_t eid = defaultEid)
    {
        return std::make_shared<NvidiaGpuVoltageSensor>(
            conn, *mctpRequester, name, "/test/config", eid, sensorId,
            *objectServer, std::move(thresholds), deviceType);
    }

    static std::string voltagePath(const std::string& name)
    {
        return "/xyz/openbmc_project/sensors/voltage/" + name;
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaGpuVoltageSensorTestBase, ConstructorCreatesDbusInterfaces)
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

TEST_F(NvidiaGpuVoltageSensorTestBase, ConstructorWithGpuPhysicalContext)
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

TEST_F(NvidiaGpuVoltageSensorTestBase, UpdateSuccessValidVoltage)
{
    mock_mctp::setNextResponse({}, buildVoltageResponse(12000000));
    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_valid");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 12.0);
}

TEST_F(NvidiaGpuVoltageSensorTestBase, UpdateSuccessZeroVoltage)
{
    mock_mctp::setNextResponse({}, buildVoltageResponse(0));
    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_zero");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 0.0);
}

TEST_F(NvidiaGpuVoltageSensorTestBase, UpdateSuccessFractionalVoltage)
{
    mock_mctp::setNextResponse({}, buildVoltageResponse(1500000));
    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_frac");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 1.5);
}

TEST_F(NvidiaGpuVoltageSensorTestBase, UpdateSuccessTwiceOverwritesValue)
{
    mock_mctp::setNextResponse({}, buildVoltageResponse(12000000));
    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_twice");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 12.0);

    mock_mctp::setNextResponse({}, buildVoltageResponse(5000000));
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 5.0);
}

// Update — error handling

TEST_F(NvidiaGpuVoltageSensorTestBase, UpdateMctpTransportError)
{
    mock_mctp::setNextResponse(std::make_error_code(std::errc::timed_out), {});
    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_mctp_err");
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuVoltageSensorTestBase, UpdateBadCompletionCodeError)
{
    mock_mctp::setNextResponse(
        {}, buildVoltageErrorResponse(
                static_cast<uint8_t>(
                    ocp::accelerator_management::CompletionCode::ERROR),
                0));
    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_bad_cc");
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuVoltageSensorTestBase, UpdateDecodeFailTruncatedBuffer)
{
    mock_mctp::setNextResponse({}, std::vector<uint8_t>(5, 0));
    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_trunc");
    EXPECT_NO_THROW(sensor->update());
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuVoltageSensorTestBase, UpdateDecodeFailEmptyBuffer)
{
    mock_mctp::setNextResponse({}, {});
    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_empty");
    EXPECT_NO_THROW(sensor->update());
    EXPECT_TRUE(std::isnan(sensor->value));
}

// Update — value retention after errors

TEST_F(NvidiaGpuVoltageSensorTestBase, UpdateSuccessThenErrorKeepsPreviousValue)
{
    mock_mctp::setNextResponse({}, buildVoltageResponse(12000000));
    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_retain");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 12.0);

    mock_mctp::setNextResponse(
        {}, buildVoltageErrorResponse(
                static_cast<uint8_t>(
                    ocp::accelerator_management::CompletionCode::ERROR),
                0));
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 12.0);
}

// Update — request encoding verification

TEST_F(NvidiaGpuVoltageSensorTestBase, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    mock_mctp::setNextResponse({}, buildVoltageResponse(12000000));
    const std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_eid_test", {},
                     gpu::DeviceIdentification::DEVICE_GPU, testEid);
    sensor->update();
    EXPECT_EQ(mock_mctp::getLastEid(), testEid);
}

// Destructor

TEST_F(NvidiaGpuVoltageSensorTestBase, DestructorDoesNotCrash)
{
    std::shared_ptr<NvidiaGpuVoltageSensor> sensor =
        createSensor(gpuVoltageSensorId, "volt_dtor");
    ASSERT_NE(sensor, nullptr);
    ASSERT_NE(sensor->sensorInterface, nullptr);
    sensor.reset();
    ASSERT_EQ(sensor, nullptr);
}

} // namespace
