/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaSmaLeakSensor.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"
#include "Thresholds.hpp"

#include <MessagePackUnpackUtils.hpp>

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

std::vector<uint8_t> buildLeakDetectionInfoResponse(uint16_t adcReadingMv)
{
    // dataSize = numSensors(1) + numThresholdLevels(1) +
    //            numSensors * (sensorId(1) + leakState(1) +
    //            thresholds(2*numThresholdLevels) + adcReadingMv(2))
    // We mock 1 sensor with 0 thresholds
    uint16_t dataSize = 1 + 1 + 1 * (1 + 1 + 0 + 2);

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

    // payload
    pack.pack(static_cast<uint8_t>(1)); // numSensors
    pack.pack(static_cast<uint8_t>(0)); // numThresholdLevels

    // sensor 1
    pack.pack(static_cast<uint8_t>(0)); // sensorId
    pack.pack(static_cast<uint8_t>(0)); // leakState
    pack.pack(adcReadingMv);            // adcReadingMv

    return buf;
}

std::vector<uint8_t> buildLeakDetectionInfoEmptyResponse()
{
    // dataSize = numSensors(1) + numThresholdLevels(1)
    // We mock 0 sensors
    uint16_t dataSize = 1 + 1;

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

    // payload
    pack.pack(static_cast<uint8_t>(0)); // numSensors
    pack.pack(static_cast<uint8_t>(0)); // numThresholdLevels

    return buf;
}

std::vector<uint8_t> buildLeakErrorResponse(uint8_t cc, uint16_t reasonCode)
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::GET_LEAK_DETECTION_INFO, cc,
        reasonCode);
}

class NvidiaSmaLeakSensorTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaSmaLeakSensor> createSensor(
        const std::string& name = "GPU_LEAK",
        std::vector<thresholds::Threshold> thresholds = {},
        gpu::DeviceIdentification deviceType =
            gpu::DeviceIdentification::DEVICE_GPU,
        uint8_t eid = test_utils::defaultEid)
    {
        return std::make_shared<NvidiaSmaLeakSensor>(
            bus(), requester(), name, "/test/config", eid, objects(),
            std::move(thresholds), deviceType);
    }

    static std::string leakPath(const std::string& name)
    {
        // sensor path prefix in NvidiaSmaLeakSensor.cpp is
        // "/xyz/openbmc_project/sensors/" and it appends "voltage/" +
        // escapeName(name)
        return "/xyz/openbmc_project/sensors/voltage/" + name;
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaSmaLeakSensorTest, ConstructorCreatesDbusInterfaces)
{
    const std::string name = "leak_ctor_iface";
    const std::shared_ptr<NvidiaSmaLeakSensor> sensor = createSensor(name);
    const std::string path = leakPath(name);

    EXPECT_TRUE(std::isnan(getProperty<double>(
        path, "xyz.openbmc_project.Sensor.Value", "Value")));
    EXPECT_EQ(getProperty<double>(path, "xyz.openbmc_project.Sensor.Value",
                                  "MaxValue"),
              5.0);
    EXPECT_EQ(getProperty<double>(path, "xyz.openbmc_project.Sensor.Value",
                                  "MinValue"),
              0.0);
    EXPECT_EQ(getProperty<std::string>(path, "xyz.openbmc_project.Sensor.Value",
                                       "Unit"),
              "xyz.openbmc_project.Sensor.Value.Unit.Volts");
}

// Update — successful readings

TEST_F(NvidiaSmaLeakSensorTest, UpdateSuccessValidReading)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildLeakDetectionInfoResponse(1500)));

    const std::shared_ptr<NvidiaSmaLeakSensor> sensor =
        createSensor("leak_valid");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 1.5);
}

TEST_F(NvidiaSmaLeakSensorTest, UpdateSuccessZeroReading)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildLeakDetectionInfoResponse(0)));

    const std::shared_ptr<NvidiaSmaLeakSensor> sensor =
        createSensor("leak_zero");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 0.0);
}

TEST_F(NvidiaSmaLeakSensorTest, UpdateSuccessEmptySensors)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildLeakDetectionInfoEmptyResponse()));

    const std::shared_ptr<NvidiaSmaLeakSensor> sensor =
        createSensor("leak_empty");
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

// Update — error handling

TEST_F(NvidiaSmaLeakSensorTest, UpdateMctpTransportError)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<NvidiaSmaLeakSensor> sensor =
        createSensor("leak_mctp_err");
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaSmaLeakSensorTest, UpdateBadCompletionCodeError)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildLeakErrorResponse(
                    static_cast<uint8_t>(
                        ocp::accelerator_management::CompletionCode::ERROR),
                    0)));

    const std::shared_ptr<NvidiaSmaLeakSensor> sensor =
        createSensor("leak_bad_cc");
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

// Destructor

TEST_F(NvidiaSmaLeakSensorTest, DestructorDoesNotCrash)
{
    std::shared_ptr<NvidiaSmaLeakSensor> sensor = createSensor("leak_dtor");
    ASSERT_NE(sensor, nullptr);
    ASSERT_NE(sensor->sensorInterface, nullptr);
    sensor.reset();
    ASSERT_EQ(sensor, nullptr);
}

} // namespace
