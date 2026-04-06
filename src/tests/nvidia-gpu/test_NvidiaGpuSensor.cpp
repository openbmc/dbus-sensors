/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaGpuSensor.hpp"
#include "NvidiaSensorUtils.hpp"
#include "OcpMctpVdm.hpp"
#include "Thresholds.hpp"

#include <MessagePackUnpackUtils.hpp>
#include <sdbusplus/exception.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace
{

constexpr uint8_t defaultEid = 10;

constexpr size_t tempResponseSize =
    ocp::accelerator_management::commonResponseSize + sizeof(int32_t);

// Message header + command(1) + cc(1) + reasonCode(2)
constexpr size_t errorResponseSize =
    ocp::accelerator_management::messageHeaderSize + 4;

std::vector<uint8_t> buildTempResponse(double tempCelsius)
{
    std::vector<uint8_t> buf(tempResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(0x00)); // command: GET_TEMPERATURE_READING
    pack.pack(static_cast<uint8_t>(0x00)); // cc: SUCCESS
    pack.pack(static_cast<uint16_t>(0));   // reserved
    pack.pack(static_cast<uint16_t>(4));   // data_size: sizeof(int32_t)
    pack.pack(static_cast<int32_t>(tempCelsius * 256));
    return buf;
}

std::vector<uint8_t> buildTempErrorResponse(uint8_t cc, uint16_t reasonCode)
{
    std::vector<uint8_t> buf(errorResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(0x00)); // command: GET_TEMPERATURE_READING
    pack.pack(cc);
    pack.pack(reasonCode);
    return buf;
}

class NvidiaGpuTempSensorTestBase : public DbusMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaGpuTempSensor> createSensor(
        uint8_t sensorId = gpuTempSensorId,
        const std::string& name = "GPU_TEMP",
        std::vector<thresholds::Threshold> thresholds = {},
        gpu::DeviceIdentification deviceType =
            gpu::DeviceIdentification::DEVICE_GPU,
        uint8_t eid = defaultEid)
    {
        return std::make_shared<NvidiaGpuTempSensor>(
            conn, *mctpRequester, name, "/test/config", eid, sensorId,
            *objectServer, std::move(thresholds), deviceType);
    }

    static std::string tempPath(const std::string& name)
    {
        return "/xyz/openbmc_project/sensors/temperature/" + name;
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaGpuTempSensorTestBase, ConstructorCreatesDbusInterfaces)
{
    const std::string name = "ctor_iface";
    const std::shared_ptr<NvidiaGpuTempSensor> sensor =
        createSensor(gpuTempSensorId, name);
    const std::string path = tempPath(name);

    EXPECT_NO_THROW(
        getProperty<double>(path, "xyz.openbmc_project.Sensor.Value", "Value"));
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

TEST_F(NvidiaGpuTempSensorTestBase, ConstructorWithTLimitSensorId)
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

TEST_F(NvidiaGpuTempSensorTestBase, ConstructorWithGpuDeviceType)
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

TEST_F(NvidiaGpuTempSensorTestBase, ConstructorWithSmaDeviceType)
{
    const std::string name = "sma_ctx";
    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor(
        gpuTempSensorId, name, {}, gpu::DeviceIdentification::DEVICE_SMA);
    const std::string path = tempPath(name);

    const std::optional<std::string> physCtx =
        nvidia_sensor_utils::deviceTypeToPhysicalContext(
            gpu::DeviceIdentification::DEVICE_SMA);
    if (!physCtx)
    {
        EXPECT_THROW(
            getProperty<std::string>(
                path, "xyz.openbmc_project.Common.PhysicalContext", "Type"),
            sdbusplus::exception_t);
    }
}

TEST_F(NvidiaGpuTempSensorTestBase, ConstructorWithPcieDeviceType)
{
    const std::string name = "pcie_ctx";
    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor(
        gpuTempSensorId, name, {}, gpu::DeviceIdentification::DEVICE_PCIE);
    const std::string path = tempPath(name);

    const std::optional<std::string> physCtx =
        nvidia_sensor_utils::deviceTypeToPhysicalContext(
            gpu::DeviceIdentification::DEVICE_PCIE);
    if (!physCtx)
    {
        EXPECT_THROW(
            getProperty<std::string>(
                path, "xyz.openbmc_project.Common.PhysicalContext", "Type"),
            sdbusplus::exception_t);
    }
}

// Constructor — threshold interfaces

TEST_F(NvidiaGpuTempSensorTestBase, ConstructorWithWarningThreshold)
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

TEST_F(NvidiaGpuTempSensorTestBase, ConstructorWithCriticalThreshold)
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

TEST_F(NvidiaGpuTempSensorTestBase, ConstructorWithMultipleThresholds)
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

TEST_F(NvidiaGpuTempSensorTestBase, ConstructorNoTLimitNoSensorTypeInterface)
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

TEST_F(NvidiaGpuTempSensorTestBase, UpdateSuccessValidTemperature)
{
    mock_mctp::setNextResponse({}, buildTempResponse(25.5));
    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 25.5);
}

TEST_F(NvidiaGpuTempSensorTestBase, UpdateSuccessNegativeTemperature)
{
    mock_mctp::setNextResponse({}, buildTempResponse(-10.0));
    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, -10.0);
}

TEST_F(NvidiaGpuTempSensorTestBase, UpdateSuccessMinTemperature)
{
    mock_mctp::setNextResponse({}, buildTempResponse(-128.0));
    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, -128.0);
}

TEST_F(NvidiaGpuTempSensorTestBase, UpdateSuccessFractionalTemperature)
{
    mock_mctp::setNextResponse({}, buildTempResponse(50.25));
    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();
    EXPECT_NEAR(sensor->value, 50.25, 0.01);
}

TEST_F(NvidiaGpuTempSensorTestBase, UpdateSuccessTwiceOverwritesValue)
{
    mock_mctp::setNextResponse({}, buildTempResponse(25.0));
    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 25.0);

    mock_mctp::setNextResponse({}, buildTempResponse(30.0));
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 30.0);
}

// Update — error handling

TEST_F(NvidiaGpuTempSensorTestBase, UpdateMctpTransportError)
{
    mock_mctp::setNextResponse(std::make_error_code(std::errc::timed_out), {});
    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuTempSensorTestBase, UpdateBadCompletionCodeError)
{
    mock_mctp::setNextResponse(
        {}, buildTempErrorResponse(
                static_cast<uint8_t>(
                    ocp::accelerator_management::CompletionCode::ERROR),
                0));
    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuTempSensorTestBase, UpdateDecodeFailTruncatedBuffer)
{
    mock_mctp::setNextResponse({}, std::vector<uint8_t>(5, 0));
    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    EXPECT_NO_THROW(sensor->update());
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuTempSensorTestBase, UpdateDecodeFailEmptyBuffer)
{
    mock_mctp::setNextResponse({}, {});
    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    EXPECT_NO_THROW(sensor->update());
    EXPECT_TRUE(std::isnan(sensor->value));
}

// Update — value retention after errors

TEST_F(NvidiaGpuTempSensorTestBase, UpdateSuccessThenErrorKeepsPreviousValue)
{
    mock_mctp::setNextResponse({}, buildTempResponse(25.0));
    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 25.0);

    mock_mctp::setNextResponse(
        {}, buildTempErrorResponse(
                static_cast<uint8_t>(
                    ocp::accelerator_management::CompletionCode::ERROR),
                0));
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 25.0);
}

TEST_F(NvidiaGpuTempSensorTestBase,
       UpdateSuccessThenMctpErrorKeepsPreviousValue)
{
    mock_mctp::setNextResponse({}, buildTempResponse(25.0));
    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 25.0);

    mock_mctp::setNextResponse(std::make_error_code(std::errc::timed_out), {});
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 25.0);
}

// Update — request encoding verification

TEST_F(NvidiaGpuTempSensorTestBase, UpdateVerifiesRequestEncoding)
{
    mock_mctp::setNextResponse({}, buildTempResponse(25.0));
    const std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    sensor->update();

    const std::span<const uint8_t> req = mock_mctp::getLastRequest();
    ASSERT_FALSE(req.empty());

    UnpackBuffer unpack(req);
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

TEST_F(NvidiaGpuTempSensorTestBase, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    mock_mctp::setNextResponse({}, buildTempResponse(25.0));
    const std::shared_ptr<NvidiaGpuTempSensor> sensor =
        createSensor(gpuTempSensorId, "eid_test", {},
                     gpu::DeviceIdentification::DEVICE_GPU, testEid);
    sensor->update();
    EXPECT_EQ(mock_mctp::getLastEid(), testEid);
}

// Destructor

TEST_F(NvidiaGpuTempSensorTestBase, DestructorDoesNotCrash)
{
    std::shared_ptr<NvidiaGpuTempSensor> sensor = createSensor();
    ASSERT_NE(sensor, nullptr);
    ASSERT_NE(sensor->sensorInterface, nullptr);
    sensor.reset();
    ASSERT_EQ(sensor, nullptr);
}

} // namespace
