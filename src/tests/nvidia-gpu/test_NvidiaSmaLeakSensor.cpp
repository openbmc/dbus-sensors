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
#include <sdbusplus/exception.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

std::vector<uint8_t> buildLeakDetectionInfoResponse(
    uint16_t adcReadingMv, const std::vector<uint16_t>& thresholds = {})
{
    // dataSize = numSensors(1) + numThresholdLevels(1) +
    //            numSensors * (sensorId(1) + leakState(1) +
    //            thresholds(2*numThresholdLevels) + adcReadingMv(2))
    const auto levels = static_cast<uint8_t>(thresholds.size());
    uint16_t dataSize =
        static_cast<uint16_t>(1 + 1 + 1 * (1 + 1 + (2 * levels) + 2));

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
    pack.pack(levels);                  // numThresholdLevels

    // sensor 1
    pack.pack(static_cast<uint8_t>(0)); // sensorId
    pack.pack(static_cast<uint8_t>(0)); // leakState
    for (const uint16_t threshold : thresholds)
    {
        pack.pack(threshold);
    }
    pack.pack(adcReadingMv); // adcReadingMv

    return buf;
}

std::vector<uint8_t> buildSetLeakThresholdsResponse(
    ocp::accelerator_management::CompletionCode cc =
        ocp::accelerator_management::CompletionCode::SUCCESS)
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::SET_LEAK_DETECTION_THRESHOLDS,
        static_cast<uint8_t>(cc), 0);
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

// Shorted below the first, wet below the second, open above the third.
constexpr uint16_t minLeakMv = 156;
constexpr uint16_t maxLeakMv = 1549;
constexpr uint16_t maxNormalMv = 1841;
constexpr uint16_t readingMv = 1740;

const std::vector<uint16_t> deviceThresholds = {minLeakMv, maxLeakMv,
                                                maxNormalMv};

auto captureAndRespond(std::vector<uint8_t>& into,
                       std::vector<uint8_t> response)
{
    return
        [&into, response = std::move(response)](
            uint8_t /*eid*/, std::span<const uint8_t> reqMsg, auto callback) {
            into.assign(reqMsg.begin(), reqMsg.end());
            callback(std::error_code{}, response);
        };
}

constexpr std::chrono::seconds dispatchTimeout{5};

// Skips the detector count, threshold count, sensor id and state.
std::vector<uint16_t> thresholdsInRequest(const std::vector<uint8_t>& request)
{
    UnpackBuffer unpack(std::span<const uint8_t>(request).subspan(
        ocp::accelerator_management::commonRequestSize + 4));
    std::vector<uint16_t> values(gpu::leakDetectorThresholdCount);
    for (uint16_t& value : values)
    {
        unpack.unpack(value);
    }
    return values;
}

class NvidiaSmaLeakSensorTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaSmaLeakSensorCarrier> createCarrier(
        const std::string& name = "GPU_LEAK",
        std::vector<thresholds::Threshold> thresholds = {},
        gpu::DeviceIdentification deviceType =
            gpu::DeviceIdentification::DEVICE_GPU,
        uint8_t eid = test_utils::defaultEid)
    {
        return std::make_shared<NvidiaSmaLeakSensorCarrier>(
            bus(), requester(), name, "/test/config", eid, objects(),
            std::move(thresholds), deviceType);
    }

    static std::shared_ptr<NvidiaSmaLeakSensor> createSensor(
        const std::string& name = "GPU_LEAK_0",
        std::vector<thresholds::Threshold> thresholds = {},
        gpu::DeviceIdentification deviceType =
            gpu::DeviceIdentification::DEVICE_GPU)
    {
        return std::make_shared<NvidiaSmaLeakSensor>(
            bus(), requester(), name, "/test/config", test_utils::defaultEid, 0,
            objects(), std::move(thresholds), deviceType);
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

// Constructor — threshold interfaces

TEST_F(NvidiaSmaLeakSensorTest, ConstructorWithWarningThreshold)
{
    const std::string name = "warn_thr";
    const std::shared_ptr<NvidiaSmaLeakSensor> sensor = createSensor(
        name, {thresholds::Threshold{thresholds::Level::WARNING,
                                     thresholds::Direction::HIGH, 1.7}});
    const std::string path = leakPath(name);

    EXPECT_EQ(getProperty<double>(
                  path, "xyz.openbmc_project.Sensor.Threshold.Warning",
                  "WarningHigh"),
              1.7);
}

TEST_F(NvidiaSmaLeakSensorTest, ConstructorWithCriticalThreshold)
{
    const std::string name = "crit_thr";
    const std::shared_ptr<NvidiaSmaLeakSensor> sensor = createSensor(
        name, {thresholds::Threshold{thresholds::Level::CRITICAL,
                                     thresholds::Direction::HIGH, 1.8}});
    const std::string path = leakPath(name);

    EXPECT_EQ(getProperty<double>(
                  path, "xyz.openbmc_project.Sensor.Threshold.Critical",
                  "CriticalHigh"),
              1.8);
}

TEST_F(NvidiaSmaLeakSensorTest, ConstructorWithMultipleThresholds)
{
    const std::string name = "multi_thr";
    const std::shared_ptr<NvidiaSmaLeakSensor> sensor = createSensor(
        name, {thresholds::Threshold{thresholds::Level::WARNING,
                                     thresholds::Direction::HIGH, 1.7},
               thresholds::Threshold{thresholds::Level::CRITICAL,
                                     thresholds::Direction::HIGH, 1.8}});
    const std::string path = leakPath(name);

    EXPECT_EQ(getProperty<double>(
                  path, "xyz.openbmc_project.Sensor.Threshold.Warning",
                  "WarningHigh"),
              1.7);
    EXPECT_EQ(getProperty<double>(
                  path, "xyz.openbmc_project.Sensor.Threshold.Critical",
                  "CriticalHigh"),
              1.8);
}

// Carrier Update — successful readings

TEST_F(NvidiaSmaLeakSensorTest, UpdateSuccessValidReading)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildLeakDetectionInfoResponse(1500)));

    const std::shared_ptr<NvidiaSmaLeakSensorCarrier> carrier =
        createCarrier("leak_valid");
    carrier->init();

    EXPECT_DOUBLE_EQ(
        getProperty<double>(leakPath("leak_valid_0"),
                            "xyz.openbmc_project.Sensor.Value", "Value"),
        1.5);
}

TEST_F(NvidiaSmaLeakSensorTest, UpdateSuccessZeroReading)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildLeakDetectionInfoResponse(0)));

    const std::shared_ptr<NvidiaSmaLeakSensorCarrier> carrier =
        createCarrier("leak_zero");
    carrier->init();

    EXPECT_DOUBLE_EQ(
        getProperty<double>(leakPath("leak_zero_0"),
                            "xyz.openbmc_project.Sensor.Value", "Value"),
        0.0);
}

TEST_F(NvidiaSmaLeakSensorTest, UpdateSuccessEmptySensors)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildLeakDetectionInfoEmptyResponse()));

    const std::shared_ptr<NvidiaSmaLeakSensorCarrier> carrier =
        createCarrier("leak_empty");
    carrier->init();

    // With empty response, sensor is never created, so path shouldn't exist
    EXPECT_THROW(
        getProperty<double>(leakPath("leak_empty_0"),
                            "xyz.openbmc_project.Sensor.Value", "Value"),
        sdbusplus::exception_t);
}

// Carrier Update — threshold crossed reading

TEST_F(NvidiaSmaLeakSensorTest, UpdateWarningThresholdCrossed)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildLeakDetectionInfoResponse(1000)));

    const std::string name = "warn_thr_cros";
    const std::shared_ptr<NvidiaSmaLeakSensorCarrier> carrier = createCarrier(
        name, {thresholds::Threshold{thresholds::Level::WARNING,
                                     thresholds::Direction::LOW, 1.5}});
    carrier->init();

    EXPECT_DOUBLE_EQ(
        getProperty<double>(leakPath("warn_thr_cros_0"),
                            "xyz.openbmc_project.Sensor.Value", "Value"),
        1.0);

    // Verify the dbus alarm property was asserted
    EXPECT_TRUE(getProperty<bool>(
        leakPath("warn_thr_cros_0"),
        "xyz.openbmc_project.Sensor.Threshold.Warning", "WarningAlarmLow"));
}

TEST_F(NvidiaSmaLeakSensorTest, UpdateCriticalThresholdCrossed)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildLeakDetectionInfoResponse(1000)));

    const std::string name = "crit_thr_cros";
    const std::shared_ptr<NvidiaSmaLeakSensorCarrier> carrier = createCarrier(
        name, {thresholds::Threshold{thresholds::Level::CRITICAL,
                                     thresholds::Direction::LOW, 1.2}});
    carrier->init();

    EXPECT_DOUBLE_EQ(
        getProperty<double>(leakPath("crit_thr_cros_0"),
                            "xyz.openbmc_project.Sensor.Value", "Value"),
        1.0);

    // Verify the dbus alarm property was asserted
    EXPECT_TRUE(getProperty<bool>(
        leakPath("crit_thr_cros_0"),
        "xyz.openbmc_project.Sensor.Threshold.Critical", "CriticalAlarmLow"));
}

TEST_F(NvidiaSmaLeakSensorTest, UpdateMultipleThresholdsCrossed)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildLeakDetectionInfoResponse(1000)));

    const std::string name = "mult_thr_cros";
    const std::shared_ptr<NvidiaSmaLeakSensorCarrier> carrier = createCarrier(
        name, {thresholds::Threshold{thresholds::Level::WARNING,
                                     thresholds::Direction::LOW, 1.5},
               thresholds::Threshold{thresholds::Level::CRITICAL,
                                     thresholds::Direction::LOW, 1.2}});
    carrier->init();

    EXPECT_DOUBLE_EQ(
        getProperty<double>(leakPath("mult_thr_cros_0"),
                            "xyz.openbmc_project.Sensor.Value", "Value"),
        1.0);

    // Verify the dbus alarm property was asserted
    EXPECT_TRUE(getProperty<bool>(
        leakPath("mult_thr_cros_0"),
        "xyz.openbmc_project.Sensor.Threshold.Critical", "CriticalAlarmLow"));
    EXPECT_TRUE(getProperty<bool>(
        leakPath("mult_thr_cros_0"),
        "xyz.openbmc_project.Sensor.Threshold.Warning", "WarningAlarmLow"));
}

// Carrier Update — error handling

TEST_F(NvidiaSmaLeakSensorTest, UpdateMctpTransportError)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<NvidiaSmaLeakSensorCarrier> carrier =
        createCarrier("leak_mctp_err");
    carrier->init();

    EXPECT_THROW(
        getProperty<double>(leakPath("leak_mctp_err_0"),
                            "xyz.openbmc_project.Sensor.Value", "Value"),
        sdbusplus::exception_t);
}

TEST_F(NvidiaSmaLeakSensorTest, UpdateBadCompletionCodeError)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildLeakErrorResponse(
                    static_cast<uint8_t>(
                        ocp::accelerator_management::CompletionCode::ERROR),
                    0)));

    const std::shared_ptr<NvidiaSmaLeakSensorCarrier> carrier =
        createCarrier("leak_bad_cc");
    carrier->init();

    EXPECT_THROW(
        getProperty<double>(leakPath("leak_bad_cc_0"),
                            "xyz.openbmc_project.Sensor.Value", "Value"),
        sdbusplus::exception_t);
}

// Destructor

TEST_F(NvidiaSmaLeakSensorTest, DestructorDoesNotCrash)
{
    std::shared_ptr<NvidiaSmaLeakSensor> sensor = createSensor("leak_dtor");
    ASSERT_NE(sensor, nullptr);
    ASSERT_NE(sensor->sensorInterface, nullptr);
    sensor.reset();
}

} // namespace

// Thresholds the device reports

TEST_F(NvidiaSmaLeakSensorTest, ThresholdsComeFromTheDevice)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildLeakDetectionInfoResponse(readingMv, deviceThresholds)));

    const std::shared_ptr<NvidiaSmaLeakSensorCarrier> carrier =
        createCarrier("thr_from_device");
    carrier->init();

    const std::string path = leakPath("thr_from_device_0");
    EXPECT_DOUBLE_EQ(getProperty<double>(
                         path, "xyz.openbmc_project.Sensor.Threshold.Critical",
                         "CriticalLow"),
                     minLeakMv / 1000.0);
    EXPECT_DOUBLE_EQ(
        getProperty<double>(
            path, "xyz.openbmc_project.Sensor.Threshold.Warning", "WarningLow"),
        maxLeakMv / 1000.0);
    EXPECT_DOUBLE_EQ(getProperty<double>(
                         path, "xyz.openbmc_project.Sensor.Threshold.Critical",
                         "CriticalHigh"),
                     maxNormalMv / 1000.0);
}

TEST_F(NvidiaSmaLeakSensorTest, ThresholdTheDeviceDoesNotReportStaysNaN)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildLeakDetectionInfoResponse(readingMv, deviceThresholds)));

    const std::shared_ptr<NvidiaSmaLeakSensorCarrier> carrier =
        createCarrier("thr_nan");
    carrier->init();

    const std::string path = leakPath("thr_nan_0");
    EXPECT_TRUE(std::isnan(getProperty<double>(
        path, "xyz.openbmc_project.Sensor.Threshold.Warning", "WarningHigh")));
    EXPECT_FALSE(
        getProperty<bool>(path, "xyz.openbmc_project.Sensor.Threshold.Warning",
                          "WarningAlarmHigh"));
}

TEST_F(NvidiaSmaLeakSensorTest, ThresholdBeforeTheDeviceReportsIsTheConfigured)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildLeakDetectionInfoResponse(readingMv)));

    std::vector<thresholds::Threshold> configured;
    configured.emplace_back(thresholds::Level::WARNING,
                            thresholds::Direction::LOW, 2.5);

    const std::shared_ptr<NvidiaSmaLeakSensorCarrier> carrier =
        createCarrier("thr_configured", std::move(configured));
    carrier->init();

    EXPECT_DOUBLE_EQ(
        getProperty<double>(leakPath("thr_configured_0"),
                            "xyz.openbmc_project.Sensor.Threshold.Warning",
                            "WarningLow"),
        2.5);
}

// Writing a threshold

TEST_F(NvidiaSmaLeakSensorTest, WriteBeforeTheDeviceReportsIsRefused)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildLeakDetectionInfoResponse(readingMv)));

    std::vector<thresholds::Threshold> configured;
    configured.emplace_back(thresholds::Level::WARNING,
                            thresholds::Direction::LOW, 2.5);

    const std::shared_ptr<NvidiaSmaLeakSensorCarrier> carrier =
        createCarrier("write_unknown", std::move(configured));
    carrier->init();

    EXPECT_TRUE(setProperty<double>(
        leakPath("write_unknown_0"),
        "xyz.openbmc_project.Sensor.Threshold.Warning", "WarningLow", 1.0));
}

TEST_F(NvidiaSmaLeakSensorTest, WriteReachesTheDeviceInWireOrder)
{
    std::vector<uint8_t> request;
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildLeakDetectionInfoResponse(readingMv, deviceThresholds)))
        .WillOnce(captureAndRespond(request, buildSetLeakThresholdsResponse()))
        .WillRepeatedly(mock_mctp::respondWith(
            {}, buildLeakDetectionInfoResponse(readingMv, deviceThresholds)));

    const std::shared_ptr<NvidiaSmaLeakSensorCarrier> carrier =
        createCarrier("write_order");
    carrier->init();

    EXPECT_FALSE(setProperty<double>(
        leakPath("write_order_0"),
        "xyz.openbmc_project.Sensor.Threshold.Warning", "WarningLow", 1.4));

    ASSERT_TRUE(
        pumpIoUntil([&request] { return !request.empty(); }, dispatchTimeout));
    ASSERT_EQ(request.size(), gpu::setLeakDetectionThresholdsRequestSize);
    EXPECT_THAT(thresholdsInRequest(request),
                testing::ElementsAre(minLeakMv, 1400, maxNormalMv));
}

TEST_F(NvidiaSmaLeakSensorTest, WriteThatCrossesTheThresholdsIsRefused)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildLeakDetectionInfoResponse(readingMv, deviceThresholds)));

    const std::shared_ptr<NvidiaSmaLeakSensorCarrier> carrier =
        createCarrier("write_cross");
    carrier->init();

    const std::string path = leakPath("write_cross_0");
    EXPECT_TRUE(setProperty<double>(
        path, "xyz.openbmc_project.Sensor.Threshold.Critical", "CriticalLow",
        maxLeakMv / 1000.0 + 0.1));
    EXPECT_TRUE(setProperty<double>(
        path, "xyz.openbmc_project.Sensor.Threshold.Warning", "WarningLow",
        maxNormalMv / 1000.0 + 0.1));
}

TEST_F(NvidiaSmaLeakSensorTest, WriteCarriesTheThresholdsLastReported)
{
    std::vector<uint8_t> first;
    std::vector<uint8_t> second;
    const std::vector<uint16_t> reread = {minLeakMv - 1, 1399, maxNormalMv - 1};
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildLeakDetectionInfoResponse(readingMv, deviceThresholds)))
        .WillOnce(captureAndRespond(first, buildSetLeakThresholdsResponse()))
        .WillOnce(mock_mctp::respondWith(
            {}, buildLeakDetectionInfoResponse(readingMv, reread)))
        .WillOnce(captureAndRespond(second, buildSetLeakThresholdsResponse()))
        .WillRepeatedly(mock_mctp::respondWith(
            {}, buildLeakDetectionInfoResponse(readingMv, reread)));

    const std::shared_ptr<NvidiaSmaLeakSensorCarrier> carrier =
        createCarrier("write_reread");
    carrier->init();

    const std::string path = leakPath("write_reread_0");
    EXPECT_FALSE(setProperty<double>(
        path, "xyz.openbmc_project.Sensor.Threshold.Warning", "WarningLow",
        1.4));
    ASSERT_TRUE(
        pumpIoUntil([&first] { return !first.empty(); }, dispatchTimeout));
    EXPECT_THAT(thresholdsInRequest(first),
                testing::ElementsAre(minLeakMv, 1400, maxNormalMv));

    EXPECT_FALSE(setProperty<double>(
        path, "xyz.openbmc_project.Sensor.Threshold.Warning", "WarningLow",
        1.4));
    ASSERT_TRUE(
        pumpIoUntil([&second] { return !second.empty(); }, dispatchTimeout));

    EXPECT_THAT(thresholdsInRequest(second),
                testing::ElementsAre(reread[0], 1400, reread[2]));
}

TEST_F(NvidiaSmaLeakSensorTest, SetIsFollowedByAReadingRightAway)
{
    std::vector<uint8_t> first;
    std::vector<uint8_t> second;
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildLeakDetectionInfoResponse(readingMv, deviceThresholds)))
        .WillOnce(captureAndRespond(first, buildSetLeakThresholdsResponse()))
        .WillOnce(mock_mctp::respondWith(
            {}, buildLeakDetectionInfoResponse(readingMv, deviceThresholds)))
        .WillOnce(captureAndRespond(second, buildSetLeakThresholdsResponse()))
        .WillRepeatedly(mock_mctp::respondWith(
            {}, buildLeakDetectionInfoResponse(readingMv, deviceThresholds)));

    const std::shared_ptr<NvidiaSmaLeakSensorCarrier> carrier =
        createCarrier("write_gate");
    carrier->init();

    const std::string path = leakPath("write_gate_0");
    EXPECT_FALSE(setProperty<double>(
        path, "xyz.openbmc_project.Sensor.Threshold.Warning", "WarningLow",
        1.4));
    ASSERT_TRUE(
        pumpIoUntil([&first] { return !first.empty(); }, dispatchTimeout));

    EXPECT_FALSE(setProperty<double>(
        path, "xyz.openbmc_project.Sensor.Threshold.Critical", "CriticalLow",
        0.2));
    ASSERT_TRUE(
        pumpIoUntil([&second] { return !second.empty(); }, dispatchTimeout));
    EXPECT_THAT(thresholdsInRequest(second),
                testing::ElementsAre(200, maxLeakMv, maxNormalMv));
}

TEST_F(NvidiaSmaLeakSensorTest, RejectedWriteDoesNotWedgeTheGate)
{
    std::vector<uint8_t> rejected;
    std::vector<uint8_t> next;
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildLeakDetectionInfoResponse(readingMv, deviceThresholds)))
        .WillOnce(captureAndRespond(
            rejected,
            buildSetLeakThresholdsResponse(
                ocp::accelerator_management::CompletionCode::ERR_INVALID_DATA)))
        .WillOnce(captureAndRespond(next, buildSetLeakThresholdsResponse()))
        .WillRepeatedly(mock_mctp::respondWith(
            {}, buildLeakDetectionInfoResponse(readingMv, deviceThresholds)));

    const std::shared_ptr<NvidiaSmaLeakSensorCarrier> carrier =
        createCarrier("write_rejected");
    carrier->init();

    const std::string path = leakPath("write_rejected_0");
    EXPECT_FALSE(setProperty<double>(
        path, "xyz.openbmc_project.Sensor.Threshold.Critical", "CriticalLow",
        0.2));
    ASSERT_TRUE(pumpIoUntil([&rejected] { return !rejected.empty(); },
                            dispatchTimeout));
    EXPECT_THAT(thresholdsInRequest(rejected),
                testing::ElementsAre(200, maxLeakMv, maxNormalMv));

    EXPECT_FALSE(setProperty<double>(
        path, "xyz.openbmc_project.Sensor.Threshold.Warning", "WarningLow",
        1.4));
    ASSERT_TRUE(
        pumpIoUntil([&next] { return !next.empty(); }, dispatchTimeout));
    EXPECT_THAT(thresholdsInRequest(next),
                testing::ElementsAre(minLeakMv, 1400, maxNormalMv));
}

TEST_F(NvidiaSmaLeakSensorTest, WritesInOneBurstBecomeOneRequest)
{
    std::vector<uint8_t> request;
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildLeakDetectionInfoResponse(readingMv, deviceThresholds)))
        .WillOnce(captureAndRespond(request, buildSetLeakThresholdsResponse()))
        .WillRepeatedly(mock_mctp::respondWith(
            {}, buildLeakDetectionInfoResponse(readingMv, deviceThresholds)));

    const std::shared_ptr<NvidiaSmaLeakSensorCarrier> carrier =
        createCarrier("write_burst");
    carrier->init();

    const std::string path = leakPath("write_burst_0");
    EXPECT_FALSE(setProperty<double>(
        path, "xyz.openbmc_project.Sensor.Threshold.Critical", "CriticalLow",
        0.2));
    EXPECT_FALSE(setProperty<double>(
        path, "xyz.openbmc_project.Sensor.Threshold.Warning", "WarningLow",
        1.4));

    ASSERT_TRUE(
        pumpIoUntil([&request] { return !request.empty(); }, dispatchTimeout));
    EXPECT_THAT(thresholdsInRequest(request),
                testing::ElementsAre(200, 1400, maxNormalMv));
}

TEST_F(NvidiaSmaLeakSensorTest, FailedReadingDoesNotWedgeTheGate)
{
    std::vector<uint8_t> first;
    std::vector<uint8_t> second;
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildLeakDetectionInfoResponse(readingMv, deviceThresholds)))
        .WillOnce(captureAndRespond(first, buildSetLeakThresholdsResponse()))
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}))
        .WillOnce(captureAndRespond(second, buildSetLeakThresholdsResponse()))
        .WillRepeatedly(mock_mctp::respondWith(
            {}, buildLeakDetectionInfoResponse(readingMv, deviceThresholds)));

    const std::shared_ptr<NvidiaSmaLeakSensorCarrier> carrier =
        createCarrier("read_fail");
    carrier->init();

    const std::string path = leakPath("read_fail_0");
    EXPECT_FALSE(setProperty<double>(
        path, "xyz.openbmc_project.Sensor.Threshold.Warning", "WarningLow",
        1.4));
    ASSERT_TRUE(
        pumpIoUntil([&first] { return !first.empty(); }, dispatchTimeout));

    EXPECT_FALSE(setProperty<double>(
        path, "xyz.openbmc_project.Sensor.Threshold.Critical", "CriticalLow",
        0.2));
    ASSERT_TRUE(
        pumpIoUntil([&second] { return !second.empty(); }, dispatchTimeout));
}
