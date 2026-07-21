/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaGpuTempSensor.hpp"
#include "NvidiaSensorUtils.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"
#include "Thresholds.hpp"

#include <sdbusplus/exception.hpp>

#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

std::vector<uint8_t> buildTempResponse(double tempCelsius)
{
    return test_utils::buildPlatformEnvSuccessResponse(
        gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING,
        static_cast<int32_t>(tempCelsius * 256));
}

std::vector<uint8_t> buildTempErrorResponse(uint8_t cc, uint16_t reasonCode)
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING, cc,
        reasonCode);
}

class NvidiaGpuTempSensorTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaGpuTempSensor> createSensor(
        uint8_t sensorId = gpuTempSensorId,
        const std::string& name = "GPU_TEMP",
        std::vector<thresholds::Threshold> thresholds = {},
        gpu::DeviceIdentification deviceType =
            gpu::DeviceIdentification::DEVICE_GPU,
        uint8_t eid = test_utils::defaultEid)
    {
        return std::make_shared<NvidiaGpuTempSensor>(
            bus(), requester(), name, "/test/config", eid, sensorId, objects(),
            std::move(thresholds), deviceType);
    }

    static std::string tempPath(const std::string& name)
    {
        return "/xyz/openbmc_project/sensors/temperature/" + name;
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaGpuTempSensorTest, ConstructorCreatesDbusInterfaces)
{
    const std::string name = "ctor_iface";
    const std::shared_ptr<NvidiaGpuTempSensor> sensor =
        createSensor(gpuTempSensorId, name);
    const std::string path = tempPath(name);

    EXPECT_TRUE(std::isnan(getProperty<double>(
        path, "xyz.openbmc_project.Sensor.Value", "Value")));
    EXPECT_EQ(getProperty<double>(path, "xyz.openbmc_project.Sensor.Value",
                                  "MaxValue"),
              127.0);
    EXPECT_EQ(getProperty<double>(path, "xyz.openbmc_project.Sensor.Value",
                                  "MinValue"),
              -128.0);
    EXPECT_EQ(getProperty<std::string>(path, "xyz.openbmc_project.Sensor.Value",
                                       "Unit"),
              "xyz.openbmc_project.Sensor.Value.Unit.DegreesC");
}

// Constructor — SensorType interface for TLimit

TEST_F(NvidiaGpuTempSensorTest, ConstructorWithTLimitSensorId)
{
    const std::string name = "tlimit";
    const std::shared_ptr<NvidiaGpuTempSensor> sensor =
        createSensor(gpuTLimitSensorId, name);
    const std::string path = tempPath(name);

    EXPECT_EQ(getProperty<std::string>(path, "xyz.openbmc_project.Sensor.Type",
                                       "ReadingBasis"),
              "xyz.openbmc_project.Sensor.Type.ReadingBasisType.Headroom");
    EXPECT_EQ(getProperty<std::string>(path, "xyz.openbmc_project.Sensor.Type",
                                       "Implementation"),
              "xyz.openbmc_project.Sensor.Type.ImplementationType.Synthesized");
}

// Constructor — PhysicalContext per device type

TEST_F(NvidiaGpuTempSensorTest, ConstructorWithGpuDeviceType)
{
    const std::string name = "gpu_ctx";
    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor(
        gpuTempSensorId, name, {}, gpu::DeviceIdentification::DEVICE_GPU);
    const std::string path = tempPath(name);

    EXPECT_EQ(
        getProperty<std::string>(
            path, "xyz.openbmc_project.Common.PhysicalContext", "Type"),
        "xyz.openbmc_project.Common.PhysicalContext.PhysicalContextType.Accelerator");
}

TEST_F(NvidiaGpuTempSensorTest, ConstructorWithSmaDeviceType)
{
    const std::string name = "sma_ctx";
    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor(
        gpuTempSensorId, name, {}, gpu::DeviceIdentification::DEVICE_SMA);
    const std::string path = tempPath(name);

    // The PhysicalContext interface must mirror the device-type mapping:
    // present with the mapped value, or absent when there is no mapping.
    const std::optional<std::string> physCtx =
        nvidia_sensor_utils::deviceTypeToPhysicalContext(
            gpu::DeviceIdentification::DEVICE_SMA);
    if (physCtx)
    {
        EXPECT_EQ(getProperty<std::string>(
                      path, "xyz.openbmc_project.Common.PhysicalContext",
                      "Type"),
                  *physCtx);
    }
    else
    {
        EXPECT_THROW(
            getProperty<std::string>(
                path, "xyz.openbmc_project.Common.PhysicalContext", "Type"),
            sdbusplus::exception_t);
    }
}

TEST_F(NvidiaGpuTempSensorTest, ConstructorWithPcieDeviceType)
{
    const std::string name = "pcie_ctx";
    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor(
        gpuTempSensorId, name, {}, gpu::DeviceIdentification::DEVICE_PCIE);
    const std::string path = tempPath(name);

    const std::optional<std::string> physCtx =
        nvidia_sensor_utils::deviceTypeToPhysicalContext(
            gpu::DeviceIdentification::DEVICE_PCIE);
    if (physCtx)
    {
        EXPECT_EQ(getProperty<std::string>(
                      path, "xyz.openbmc_project.Common.PhysicalContext",
                      "Type"),
                  *physCtx);
    }
    else
    {
        EXPECT_THROW(
            getProperty<std::string>(
                path, "xyz.openbmc_project.Common.PhysicalContext", "Type"),
            sdbusplus::exception_t);
    }
}

// Constructor — threshold interfaces

TEST_F(NvidiaGpuTempSensorTest, ConstructorWithWarningThreshold)
{
    const std::string name = "warn_thr";
    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor(
        gpuTempSensorId, name,
        {thresholds::Threshold{thresholds::Level::WARNING,
                               thresholds::Direction::HIGH, 85.0}});
    const std::string path = tempPath(name);

    EXPECT_EQ(getProperty<double>(
                  path, "xyz.openbmc_project.Sensor.Threshold.Warning",
                  "WarningHigh"),
              85.0);
}

TEST_F(NvidiaGpuTempSensorTest, ConstructorWithCriticalThreshold)
{
    const std::string name = "crit_thr";
    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor(
        gpuTempSensorId, name,
        {thresholds::Threshold{thresholds::Level::CRITICAL,
                               thresholds::Direction::HIGH, 100.0}});
    const std::string path = tempPath(name);

    EXPECT_EQ(getProperty<double>(
                  path, "xyz.openbmc_project.Sensor.Threshold.Critical",
                  "CriticalHigh"),
              100.0);
}

TEST_F(NvidiaGpuTempSensorTest, ConstructorWithMultipleThresholds)
{
    const std::string name = "multi_thr";
    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor(
        gpuTempSensorId, name,
        {thresholds::Threshold{thresholds::Level::WARNING,
                               thresholds::Direction::HIGH, 85.0},
         thresholds::Threshold{thresholds::Level::CRITICAL,
                               thresholds::Direction::HIGH, 100.0}});
    const std::string path = tempPath(name);

    EXPECT_EQ(getProperty<double>(
                  path, "xyz.openbmc_project.Sensor.Threshold.Warning",
                  "WarningHigh"),
              85.0);
    EXPECT_EQ(getProperty<double>(
                  path, "xyz.openbmc_project.Sensor.Threshold.Critical",
                  "CriticalHigh"),
              100.0);
}

// Constructor — negative: no SensorType when not TLimit

TEST_F(NvidiaGpuTempSensorTest, ConstructorNoTLimitNoSensorTypeInterface)
{
    const std::string name = "no_tlimit";
    const std::shared_ptr<NvidiaGpuTempSensor> sensor =
        createSensor(gpuTempSensorId, name);
    const std::string path = tempPath(name);

    EXPECT_THROW(getProperty<std::string>(
                     path, "xyz.openbmc_project.Sensor.Type", "ReadingBasis"),
                 sdbusplus::exception_t);
}

// Update — successful temperature readings

TEST_F(NvidiaGpuTempSensorTest, UpdateSuccessValidTemperature)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildTempResponse(25.5)));

    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 25.5);
}

TEST_F(NvidiaGpuTempSensorTest, UpdateSuccessNegativeTemperature)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildTempResponse(-10.0)));

    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, -10.0);
}

TEST_F(NvidiaGpuTempSensorTest, UpdateSuccessMinTemperature)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildTempResponse(-128.0)));

    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, -128.0);
}

TEST_F(NvidiaGpuTempSensorTest, UpdateSuccessFractionalTemperature)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildTempResponse(50.25)));

    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();
    EXPECT_NEAR(sensor->value, 50.25, 0.01);
}

TEST_F(NvidiaGpuTempSensorTest, UpdateSuccessTwiceOverwritesValue)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildTempResponse(25.0)))
        .WillOnce(mock_mctp::respondWith({}, buildTempResponse(30.0)));

    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 25.0);

    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 30.0);
}

// Update — error handling

TEST_F(NvidiaGpuTempSensorTest, UpdateMctpTransportError)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuTempSensorTest, UpdateBadCompletionCodeError)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildTempErrorResponse(
                    static_cast<uint8_t>(
                        ocp::accelerator_management::CompletionCode::ERROR),
                    0)));

    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuTempSensorTest, UpdateDecodeFailTruncatedBuffer)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, std::vector<uint8_t>(5, 0)));

    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuTempSensorTest, UpdateDecodeFailEmptyBuffer)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

// Update — value retention after errors

TEST_F(NvidiaGpuTempSensorTest, UpdateSuccessThenErrorKeepsPreviousValue)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildTempResponse(25.0)))
        .WillOnce(mock_mctp::respondWith(
            {}, buildTempErrorResponse(
                    static_cast<uint8_t>(
                        ocp::accelerator_management::CompletionCode::ERROR),
                    0)));

    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 25.0);

    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 25.0);
}

TEST_F(NvidiaGpuTempSensorTest, UpdateSuccessThenMctpErrorKeepsPreviousValue)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildTempResponse(25.0)))
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 25.0);

    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 25.0);
}

// Update — request encoding verification

TEST_F(NvidiaGpuTempSensorTest, UpdateVerifiesRequestEncoding)
{
    // Copy the request bytes before completing the call: the reqMsg span is
    // a view into caller-owned memory, valid only during the call.
    std::vector<uint8_t> lastRequest;
    const std::vector<uint8_t> response = buildTempResponse(25.0);
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce([&](uint8_t /*eid*/, std::span<const uint8_t> reqMsg,
                      auto callback) {
            lastRequest.assign(reqMsg.begin(), reqMsg.end());
            callback(std::error_code{}, response);
        });

    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();

    ASSERT_FALSE(lastRequest.empty());

    UnpackBuffer unpack(lastRequest);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    const int rc = ocp::accelerator_management::unpackHeader(
        unpack, gpu::nvidiaPciVendorId, ocpMsgType, instanceId, msgType);
    EXPECT_EQ(rc, 0);
    EXPECT_EQ(ocpMsgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(msgType,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    uint8_t command = 0;
    unpack.unpack(command);
    EXPECT_EQ(command,
              static_cast<uint8_t>(
                  gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING));

    EXPECT_EQ(unpack.getError(), 0);
}

TEST_F(NvidiaGpuTempSensorTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, buildTempResponse(25.0)));

    const std::shared_ptr<NvidiaGpuTempSensor> sensor =
        createSensor(gpuTempSensorId, "eid_test", {},
                     gpu::DeviceIdentification::DEVICE_GPU, testEid);
    sensor->update();
}

// Destructor

TEST_F(NvidiaGpuTempSensorTest, DestructorRemovesDbusInterfaces)
{
    const std::string name = "dtor";
    std::shared_ptr<NvidiaGpuTempSensor> sensor =
        createSensor(gpuTempSensorId, name);
    const std::string path = tempPath(name);

    EXPECT_EQ(getProperty<double>(path, "xyz.openbmc_project.Sensor.Value",
                                  "MaxValue"),
              127.0);

    sensor.reset();
    drainPendingAsync();

    EXPECT_THROW(getProperty<double>(path, "xyz.openbmc_project.Sensor.Value",
                                     "MaxValue"),
                 sdbusplus::exception_t);
}

} // namespace
