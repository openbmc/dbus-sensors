/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaSensorConfig.hpp"
#include "NvidiaSmaDevice.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"
#include "Thresholds.hpp"

#include <MessagePackUnpackUtils.hpp>
#include <sdbusplus/exception.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
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

constexpr uint8_t defaultEid = 20;

constexpr uint8_t smaInternalSensorId = 17;

// Short enough that a second poll round lands well inside pollTimeout.
constexpr uint64_t fastPollMs = 10;

// Upper bound on how long the read loop may take to poll again.
constexpr std::chrono::seconds pollTimeout{5};

// Several fastPollMs intervals, so a loop that kept running would be caught.
constexpr std::chrono::seconds quietWindow{1};

// A GetLeakDetectionInfo response reporting one leak sensor, so the device
// creates one.
std::vector<uint8_t> buildOneLeakSensorResponse()
{
    // numSensors + numThresholdLevels + one sensor of sensorId, leakState and
    // adcReadingMv, with no thresholds
    const uint16_t dataSize = 1 + 1 + (1 + 1 + 2);

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
    pack.pack(static_cast<uint8_t>(1));  // numSensors
    pack.pack(static_cast<uint8_t>(0));  // numThresholdLevels
    pack.pack(static_cast<uint8_t>(0));  // sensorId
    pack.pack(static_cast<uint8_t>(0));  // leakState
    pack.pack(static_cast<uint16_t>(0)); // adcReadingMv
    return buf;
}

bool isLeakDetectionInfo(std::span<const uint8_t> request)
{
    return request[ocp::accelerator_management::messageHeaderSize] ==
           static_cast<uint8_t>(
               gpu::PlatformEnvironmentalCommands::GET_LEAK_DETECTION_INFO);
}

// The reading an SMA answers with for a sensor it cannot read.
constexpr uint32_t noReading = 0xFFFFFFFF;

// The command and sensor id of a request reading one of an SMA's sensors, or
// nullopt for any other request.
std::optional<std::pair<gpu::PlatformEnvironmentalCommands, uint8_t>>
    requestedSensor(std::span<const uint8_t> request)
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
            static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL))
    {
        return std::nullopt;
    }

    const auto readingCommand =
        static_cast<gpu::PlatformEnvironmentalCommands>(command);
    switch (readingCommand)
    {
        case gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING:
        case gpu::PlatformEnvironmentalCommands::GET_CURRENT_POWER_DRAW:
        case gpu::PlatformEnvironmentalCommands::GET_VOLTAGE:
            return std::make_pair(readingCommand, sensorId);
        default:
            return std::nullopt;
    }
}

// Answer each sensor request with the id it asked for as the reading, in
// degrees, watts or volts, so a sensor's reading shows which id it polled.
// The ids in rejectedIds are rejected the way an SMA rejects a sensor it does
// not have, and the ids in noReadingIds are answered without a reading.
auto answerSensorRequests(std::vector<uint8_t> rejectedIds = {},
                          std::vector<uint8_t> noReadingIds = {})
{
    return [rejectedIds = std::move(rejectedIds),
            noReadingIds = std::move(noReadingIds)](
               uint8_t /*eid*/, std::span<const uint8_t> reqMsg,
               auto callback) {
        const auto sensor = requestedSensor(reqMsg);
        if (!sensor)
        {
            callback(std::error_code{}, std::span<const uint8_t>{});
            return;
        }
        const auto [command, sensorId] = *sensor;
        if (std::ranges::find(rejectedIds, sensorId) != rejectedIds.end())
        {
            const std::vector<uint8_t> response =
                test_utils::buildPlatformEnvErrorResponse(
                    command,
                    static_cast<uint8_t>(ocp::accelerator_management::
                                             CompletionCode::ERR_INVALID_DATA),
                    0);
            callback(std::error_code{}, response);
            return;
        }
        if (std::ranges::find(noReadingIds, sensorId) != noReadingIds.end())
        {
            const std::vector<uint8_t> response =
                test_utils::buildPlatformEnvSuccessResponse(command, noReading);
            callback(std::error_code{}, response);
            return;
        }

        std::vector<uint8_t> response;
        switch (command)
        {
            case gpu::PlatformEnvironmentalCommands::GET_CURRENT_POWER_DRAW:
                response = test_utils::buildPlatformEnvSuccessResponse(
                    command, static_cast<uint32_t>(sensorId) * 1000);
                break;
            case gpu::PlatformEnvironmentalCommands::GET_VOLTAGE:
                response = test_utils::buildPlatformEnvSuccessResponse(
                    command, static_cast<uint32_t>(sensorId) * 1000000);
                break;
            default:
                response = test_utils::buildPlatformEnvSuccessResponse(
                    command, static_cast<int32_t>(sensorId) * 256);
                break;
        }
        callback(std::error_code{}, response);
    };
}

class NvidiaSmaDeviceTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<SmaDevice> createDevice(
        const std::string& name = "SMA", uint8_t eid = defaultEid,
        uint64_t pollRate = sensorPollRateMs,
        const std::vector<SmaSensorConfig>& sensors = {
            {.kind = SmaSensorKind::Temperature,
             .id = smaInternalSensorId,
             .name = "SMA_Internal"}})
    {
        const std::string path = "/test/chassis/" + name;
        const NvidiaMctpVdmConfig config{
            .path = path, .name = name, .pollRate = pollRate};
        const SmaDeviceConfigs smaConfig{.sensors = sensors};
        return std::make_shared<SmaDevice>(config, smaConfig, bus(), eid,
                                           ioContext(), requester(), objects());
    }

    static double sensorValue(const std::string& type,
                              const std::string& sensorName)
    {
        return getProperty<double>(
            "/xyz/openbmc_project/sensors/" + type + "/" + sensorName,
            "xyz.openbmc_project.Sensor.Value", "Value");
    }

    static std::string sensorUnit(const std::string& type,
                                  const std::string& sensorName)
    {
        return getProperty<std::string>(
            "/xyz/openbmc_project/sensors/" + type + "/" + sensorName,
            "xyz.openbmc_project.Sensor.Value", "Unit");
    }

    static double temperatureValue(const std::string& sensorName)
    {
        return sensorValue("temperature", sensorName);
    }

    static std::string temperatureUnit(const std::string& sensorName)
    {
        return sensorUnit("temperature", sensorName);
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

// A device re-enumerated at a new EID keeps its sensors, so every one of them
// has to follow it there, the leak sensors included.
TEST_F(NvidiaSmaDeviceTest, SetEidRetargetsLeakSensors)
{
    constexpr uint8_t newEid = 30;
    std::vector<uint8_t> leakEids;
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault([&leakEids](uint8_t eid, std::span<const uint8_t> req,
                                   auto callback) {
            if (!isLeakDetectionInfo(req))
            {
                callback(std::make_error_code(std::errc::timed_out), {});
                return;
            }
            leakEids.push_back(eid);
            const std::vector<uint8_t> response = buildOneLeakSensorResponse();
            callback(std::error_code{}, response);
        });

    const std::shared_ptr<SmaDevice> device =
        createDevice("sma_set_eid", defaultEid, fastPollMs);
    device->init();
    ASSERT_TRUE(pumpIoUntil([&] { return !leakEids.empty(); }, pollTimeout));

    device->setEid(newEid);
    leakEids.clear();
    device->setOnline();

    ASSERT_TRUE(pumpIoUntil([&] { return !leakEids.empty(); }, pollTimeout));
    EXPECT_THAT(leakEids, testing::Each(newEid));
}

TEST_F(NvidiaSmaDeviceTest, PublishesEachConfiguredTemperatureSensorByName)
{
    ON_CALL(mctpMock, sendRecvMsg).WillByDefault(answerSensorRequests());

    const std::shared_ptr<SmaDevice> device = createDevice(
        "sma_temps", defaultEid, sensorPollRateMs,
        {{.kind = SmaSensorKind::Temperature, .id = 16, .name = "SMA_Ext"},
         {.kind = SmaSensorKind::Temperature,
          .id = smaInternalSensorId,
          .name = "SMA_Internal"},
         {.kind = SmaSensorKind::Temperature, .id = 200, .name = "Board"}});
    device->init();
    device->setOnline();

    const std::string degrees =
        "xyz.openbmc_project.Sensor.Value.Unit.DegreesC";
    EXPECT_EQ(temperatureUnit("sma_temps_SMA_Ext"), degrees);
    EXPECT_EQ(temperatureUnit("sma_temps_SMA_Internal"), degrees);
    EXPECT_EQ(temperatureUnit("sma_temps_Board"), degrees);
}

TEST_F(NvidiaSmaDeviceTest, EachTemperatureSensorReadsItsOwnId)
{
    ON_CALL(mctpMock, sendRecvMsg).WillByDefault(answerSensorRequests());

    const std::shared_ptr<SmaDevice> device = createDevice(
        "sma_poll", defaultEid, sensorPollRateMs,
        {{.kind = SmaSensorKind::Temperature, .id = 16, .name = "SMA_Ext"},
         {.kind = SmaSensorKind::Temperature, .id = 100, .name = "Board"}});
    device->init();
    device->setOnline();

    EXPECT_EQ(temperatureValue("sma_poll_SMA_Ext"), 16.0);
    EXPECT_EQ(temperatureValue("sma_poll_Board"), 100.0);
}

TEST_F(NvidiaSmaDeviceTest, PublishesNoSensorWhenNoneConfigured)
{
    std::vector<uint8_t> requestedIds;
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(
            [&requestedIds](uint8_t /*eid*/, std::span<const uint8_t> reqMsg,
                            auto callback) {
                const auto sensor = requestedSensor(reqMsg);
                if (sensor)
                {
                    requestedIds.push_back(sensor->second);
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

TEST_F(NvidiaSmaDeviceTest, PublishesTemperatureSensorTheSmaRejects)
{
    ON_CALL(mctpMock, sendRecvMsg).WillByDefault(answerSensorRequests({19}));

    const std::shared_ptr<SmaDevice> device = createDevice(
        "sma_reject", defaultEid, sensorPollRateMs,
        {{.kind = SmaSensorKind::Temperature, .id = 16, .name = "SMA_Ext"},
         {.kind = SmaSensorKind::Temperature, .id = 19, .name = "BusBar"},
         {.kind = SmaSensorKind::Temperature, .id = 217, .name = "CPU1_SoC"}});
    device->init();
    device->setOnline();

    EXPECT_EQ(temperatureValue("sma_reject_SMA_Ext"), 16.0);
    EXPECT_TRUE(std::isnan(temperatureValue("sma_reject_BusBar")));
    EXPECT_EQ(temperatureValue("sma_reject_CPU1_SoC"), 217.0);
}

TEST_F(NvidiaSmaDeviceTest, PublishesPowerAndVoltageSensorsTheSmaReports)
{
    ON_CALL(mctpMock, sendRecvMsg).WillByDefault(answerSensorRequests());

    const std::shared_ptr<SmaDevice> device = createDevice(
        "sma_hsc", defaultEid, sensorPollRateMs,
        {{.kind = SmaSensorKind::Temperature, .id = 192, .name = "HSC"},
         {.kind = SmaSensorKind::Power, .id = 128, .name = "HSC"},
         {.kind = SmaSensorKind::Voltage, .id = 208, .name = "HSCC_Vout"}});
    device->init();
    device->setOnline();

    EXPECT_EQ(sensorValue("temperature", "sma_hsc_HSC"), 192.0);
    EXPECT_EQ(sensorUnit("power", "sma_hsc_HSC"),
              "xyz.openbmc_project.Sensor.Value.Unit.Watts");
    EXPECT_EQ(sensorValue("power", "sma_hsc_HSC"), 128.0);
    EXPECT_EQ(sensorUnit("voltage", "sma_hsc_HSCC_Vout"),
              "xyz.openbmc_project.Sensor.Value.Unit.Volts");
    EXPECT_EQ(sensorValue("voltage", "sma_hsc_HSCC_Vout"), 208.0);
}

TEST_F(NvidiaSmaDeviceTest, PublishesPowerAndVoltageSensorsWithoutAReading)
{
    ON_CALL(mctpMock, sendRecvMsg).WillByDefault(answerSensorRequests({}, {9}));

    const std::shared_ptr<SmaDevice> device = createDevice(
        "sma_unread", defaultEid, sensorPollRateMs,
        {{.kind = SmaSensorKind::Power, .id = 128, .name = "HSC"},
         {.kind = SmaSensorKind::Power, .id = 9, .name = "Aux"},
         {.kind = SmaSensorKind::Voltage, .id = 9, .name = "Aux_Vin"}});
    device->init();
    device->setOnline();

    EXPECT_EQ(sensorValue("power", "sma_unread_HSC"), 128.0);
    EXPECT_TRUE(std::isnan(sensorValue("power", "sma_unread_Aux")));
    EXPECT_TRUE(std::isnan(sensorValue("voltage", "sma_unread_Aux_Vin")));
}

// An offline device stops polling, so every one of its sensors has to say so
// rather than keep its last reading, the leak sensors included.
TEST_F(NvidiaSmaDeviceTest, OfflineMarksLeakSensorsNonFunctional)
{
    bool leakQueried = false;
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault([&leakQueried](uint8_t /*eid*/,
                                      std::span<const uint8_t> req,
                                      auto callback) {
            if (!isLeakDetectionInfo(req))
            {
                callback(std::make_error_code(std::errc::timed_out), {});
                return;
            }
            leakQueried = true;
            const std::vector<uint8_t> response = buildOneLeakSensorResponse();
            callback(std::error_code{}, response);
        });

    const std::shared_ptr<SmaDevice> device =
        createDevice("sma_offline", defaultEid, fastPollMs);
    device->init();
    ASSERT_TRUE(pumpIoUntil([&] { return leakQueried; }, pollTimeout));

    const std::string leakPath =
        "/xyz/openbmc_project/sensors/voltage/sma_offline_LEAKDETECTOR_0";
    const std::string operationalStatus =
        "xyz.openbmc_project.State.Decorator.OperationalStatus";

    device->setOffline();
    EXPECT_FALSE(getProperty<bool>(leakPath, operationalStatus, "Functional"));

    device->setOnline();
    EXPECT_TRUE(getProperty<bool>(leakPath, operationalStatus, "Functional"));
}

TEST_F(NvidiaSmaDeviceTest, PublishesTemperatureSensorThresholdsAsConfigured)
{
    ON_CALL(mctpMock, sendRecvMsg).WillByDefault(answerSensorRequests());

    const std::shared_ptr<SmaDevice> device = createDevice(
        "sma_thresholds", defaultEid, sensorPollRateMs,
        {{.kind = SmaSensorKind::Temperature,
          .id = 18,
          .name = "GPU_VR",
          .thresholds = {{thresholds::Level::WARNING,
                          thresholds::Direction::HIGH, 120.0},
                         {thresholds::Level::CRITICAL,
                          thresholds::Direction::HIGH, 130.0},
                         {thresholds::Level::HARDSHUTDOWN,
                          thresholds::Direction::HIGH, 135.0}}},
         {.kind = SmaSensorKind::Temperature, .id = 16, .name = "SMA_Ext"}});
    device->init();
    device->setOnline();

    const std::string configured =
        "/xyz/openbmc_project/sensors/temperature/sma_thresholds_GPU_VR";
    EXPECT_EQ(getProperty<double>(
                  configured, "xyz.openbmc_project.Sensor.Threshold.Warning",
                  "WarningHigh"),
              120.0);
    EXPECT_EQ(getProperty<double>(
                  configured, "xyz.openbmc_project.Sensor.Threshold.Critical",
                  "CriticalHigh"),
              130.0);
    EXPECT_EQ(
        getProperty<double>(configured,
                            "xyz.openbmc_project.Sensor.Threshold.HardShutdown",
                            "HardShutdownHigh"),
        135.0);

    const std::string unconfigured =
        "/xyz/openbmc_project/sensors/temperature/sma_thresholds_SMA_Ext";
    EXPECT_EQ(temperatureValue("sma_thresholds_SMA_Ext"), 16.0);
    EXPECT_THROW(
        getProperty<double>(unconfigured,
                            "xyz.openbmc_project.Sensor.Threshold.Warning",
                            "WarningHigh"),
        sdbusplus::exception_t);
    EXPECT_THROW(
        getProperty<double>(unconfigured,
                            "xyz.openbmc_project.Sensor.Threshold.Critical",
                            "CriticalHigh"),
        sdbusplus::exception_t);
    EXPECT_THROW(
        getProperty<double>(unconfigured,
                            "xyz.openbmc_project.Sensor.Threshold.HardShutdown",
                            "HardShutdownHigh"),
        sdbusplus::exception_t);
}

bool isReadThermalParameters(std::span<const uint8_t> request)
{
    return request.size() > ocp::accelerator_management::messageHeaderSize &&
           request[ocp::accelerator_management::messageHeaderSize] ==
               static_cast<uint8_t>(
                   gpu::PlatformEnvironmentalCommands::READ_THERMAL_PARAMETERS);
}

// Answer sensor requests as answerSensorRequests does, and Read Thermal
// Parameters with thermalParameter, or with an error when it is nullopt.
auto answerWithThermalParameter(std::optional<int32_t> thermalParameter)
{
    return [thermalParameter, answerSensors = answerSensorRequests()](
               uint8_t eid, std::span<const uint8_t> reqMsg, auto callback) {
        if (!isReadThermalParameters(reqMsg))
        {
            answerSensors(eid, reqMsg, std::move(callback));
            return;
        }
        const auto command =
            gpu::PlatformEnvironmentalCommands::READ_THERMAL_PARAMETERS;
        const std::vector<uint8_t> response =
            thermalParameter
                ? test_utils::buildPlatformEnvSuccessResponse(command,
                                                              *thermalParameter)
                : test_utils::buildPlatformEnvErrorResponse(
                      command,
                      static_cast<uint8_t>(
                          ocp::accelerator_management::CompletionCode::
                              ERR_INVALID_DATA),
                      0);
        callback(std::error_code{}, response);
    };
}

SmaSensorConfig dynamicWarningSensor()
{
    return {.kind = SmaSensorKind::Temperature,
            .id = 16,
            .name = "SMA_Ext",
            .dynamicThresholds = {
                {thresholds::Level::WARNING, thresholds::Direction::HIGH,
                 std::numeric_limits<double>::quiet_NaN()}}};
}

TEST_F(NvidiaSmaDeviceTest, PublishesDynamicThresholdsTheSmaHolds)
{
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(answerWithThermalParameter(120));

    const std::shared_ptr<SmaDevice> device = createDevice(
        "sma_dynamic", defaultEid, sensorPollRateMs, {dynamicWarningSensor()});
    device->init();
    device->setOnline();

    EXPECT_EQ(temperatureValue("sma_dynamic_SMA_Ext"), 16.0);
    EXPECT_EQ(
        getProperty<double>(
            "/xyz/openbmc_project/sensors/temperature/sma_dynamic_SMA_Ext",
            "xyz.openbmc_project.Sensor.Threshold.Warning", "WarningHigh"),
        120.0);
}

TEST_F(NvidiaSmaDeviceTest, PublishesWithoutDynamicThresholdsTheSmaRefuses)
{
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(answerWithThermalParameter(std::nullopt));

    const std::shared_ptr<SmaDevice> device =
        createDevice("sma_dynamic_refused", defaultEid, sensorPollRateMs,
                     {dynamicWarningSensor()});
    device->init();
    device->setOnline();

    EXPECT_EQ(temperatureValue("sma_dynamic_refused_SMA_Ext"), 16.0);
    EXPECT_THROW(
        getProperty<double>(
            "/xyz/openbmc_project/sensors/temperature/sma_dynamic_refused_SMA_Ext",
            "xyz.openbmc_project.Sensor.Threshold.Warning", "WarningHigh"),
        sdbusplus::exception_t);
}

TEST_F(NvidiaSmaDeviceTest, PublishesConfiguredPhysicalContext)
{
    ON_CALL(mctpMock, sendRecvMsg).WillByDefault(answerSensorRequests());

    const std::shared_ptr<SmaDevice> device = createDevice(
        "sma_ctx", defaultEid, sensorPollRateMs,
        {{.kind = SmaSensorKind::Temperature,
          .id = 18,
          .name = "GPU_VR",
          .physicalContext = "Board"},
         {.kind = SmaSensorKind::Temperature, .id = 16, .name = "SMA_Ext"},
         {.kind = SmaSensorKind::Temperature,
          .id = 19,
          .name = "BusBar",
          .physicalContext = "NoSuchContext"},
         {.kind = SmaSensorKind::Power,
          .id = 128,
          .name = "HSC",
          .physicalContext = "VoltageRegulator"},
         {.kind = SmaSensorKind::Voltage,
          .id = 192,
          .name = "HSC_Vin",
          .physicalContext = "VoltageRegulator"}});
    device->init();
    device->setOnline();

    const std::string contextIface =
        "xyz.openbmc_project.Common.PhysicalContext";
    const std::string sensors = "/xyz/openbmc_project/sensors/";
    EXPECT_EQ(getProperty<std::string>(sensors + "temperature/sma_ctx_GPU_VR",
                                       contextIface, "Type"),
              "xyz.openbmc_project.Common.PhysicalContext."
              "PhysicalContextType.Board");
    EXPECT_EQ(getProperty<std::string>(sensors + "power/sma_ctx_HSC",
                                       contextIface, "Type"),
              "xyz.openbmc_project.Common.PhysicalContext."
              "PhysicalContextType.VoltageRegulator");
    EXPECT_EQ(getProperty<std::string>(sensors + "voltage/sma_ctx_HSC_Vin",
                                       contextIface, "Type"),
              "xyz.openbmc_project.Common.PhysicalContext."
              "PhysicalContextType.VoltageRegulator");

    EXPECT_EQ(temperatureValue("sma_ctx_SMA_Ext"), 16.0);
    EXPECT_THROW(
        getProperty<std::string>(sensors + "temperature/sma_ctx_SMA_Ext",
                                 contextIface, "Type"),
        sdbusplus::exception_t);
    EXPECT_EQ(temperatureValue("sma_ctx_BusBar"), 19.0);
    EXPECT_THROW(
        getProperty<std::string>(sensors + "temperature/sma_ctx_BusBar",
                                 contextIface, "Type"),
        sdbusplus::exception_t);
}

} // namespace
