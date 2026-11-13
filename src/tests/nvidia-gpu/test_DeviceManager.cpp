/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "DbusMockTestBase.hpp"
#include "DeviceManager.hpp"
#include "MctpMockTestBase.hpp"
#include "NvidiaGpuControlErrors.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaSmaDevice.hpp"
#include "OcpMctpVdm.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"

#include <MessagePackUnpackUtils.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <sdbusplus/vtable.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

using ObjectServices = std::map<std::string, std::vector<std::string>>;
using SubTree = std::map<std::string, ObjectServices>;

constexpr const char* endpointPath =
    "/au/com/codeconstruct/mctp1/networks/1/endpoints/9";
constexpr const char* configPath =
    "/xyz/openbmc_project/inventory/system/platform/Test/Test_SMA";
constexpr const char* endpointIface = "xyz.openbmc_project.MCTP.Endpoint";
constexpr const char* configIface =
    "xyz.openbmc_project.Configuration.MCTPUSBDevice";
constexpr uint8_t endpointEid = 9;

// A retry waits out the rescan debounce of one second, so this is room for
// a retry to have happened.
constexpr std::chrono::seconds discoveryTimeout{5};

// Longer than the rescan debounce, so a retry that was going to happen would
// have by the end of it.
constexpr std::chrono::seconds quietWindow{2};

// The name stays with the connection for the rest of the binary, and sd-bus
// refuses a second request for it, so it is asked for once.
void ownMapperName(sdbusplus::asio::connection& connection)
{
    [[maybe_unused]] static const bool mapperNameOwned = [&connection] {
        connection.request_name("xyz.openbmc_project.ObjectMapper");
        return true;
    }();
}

// Stands in for the mapper and for the endpoint mctpd would publish, so the
// discovery sweep reaches the configured_by lookup and the answer to it can
// be chosen per call.
class DeviceManagerTest : public DbusMockTestBase
{
  protected:
    void SetUp() override
    {
        DbusMockTestBase::SetUp();
        if (IsSkipped())
        {
            return;
        }

        ownMapperName(*bus());
        const std::string self = bus()->get_unique_name();

        mapper = objects().add_interface("/xyz/openbmc_project/object_mapper",
                                         "xyz.openbmc_project.ObjectMapper");
        mapper->register_method(
            "GetSubTree", [self](const std::string& /*root*/, int32_t /*depth*/,
                                 const std::vector<std::string>& ifaces) {
                SubTree tree;
                if (ifaces == std::vector<std::string>{endpointIface})
                {
                    tree[endpointPath] = {{self, {endpointIface}}};
                }
                return tree;
            });
        mapper->register_method(
            "GetAssociatedSubTree",
            [this](const sdbusplus::object_path& /*association*/,
                   const sdbusplus::object_path& /*root*/, int32_t /*depth*/,
                   const std::vector<std::string>& /*ifaces*/) {
                ++lookups;
                return lookup(lookups);
            });
        mapper->initialize();

        endpoint = objects().add_interface(endpointPath, endpointIface);
        endpoint->register_property("EID", endpointEid);
        endpoint->register_property(
            "SupportedMessageTypes",
            std::vector<uint8_t>{ocp::accelerator_management::messageType});
        endpoint->initialize();

        config = objects().add_interface(configPath, configIface);
        config->register_property_r<std::string>(
            "Type", sdbusplus::vtable::property_::emits_change,
            [this](const std::string& /*current*/) {
                ++configReads;
                return std::string{"MCTPUSBDevice"};
            });
        config->initialize();

        // Publishing the interface read it once; only reads after this count.
        configReads = 0;
    }

    void TearDown() override
    {
        if (hasBus())
        {
            objects().remove_interface(config);
            objects().remove_interface(endpoint);
            objects().remove_interface(mapper);
        }
        DbusMockTestBase::TearDown();
    }

    // The mapper's answer for the configured_by lookup, by call number.
    std::function<SubTree(int)> lookup;
    int lookups = 0;
    // How often the configuration was read, which only happens once a lookup
    // has found it.
    int configReads = 0;

    std::shared_ptr<sdbusplus::asio::dbus_interface> mapper;
    std::shared_ptr<sdbusplus::asio::dbus_interface> endpoint;
    std::shared_ptr<sdbusplus::asio::dbus_interface> config;
};

TEST_F(DeviceManagerTest, RetriesTransientAssociationLookupFailure)
{
    const std::string self = bus()->get_unique_name();
    lookup = [self](int call) -> SubTree {
        if (call == 1)
        {
            throw Unavailable();
        }
        return {{configPath, {{self, {configIface}}}}};
    };

    DeviceManager manager(ioContext(), objects(), bus(), requester());
    manager.createSensors();

    EXPECT_TRUE(
        pumpIoUntil([this] { return configReads > 0; }, discoveryTimeout));
    EXPECT_EQ(lookups, 2);
}

TEST_F(DeviceManagerTest, SkipsMissingAssociation)
{
    lookup = [](int) { return SubTree{}; };

    DeviceManager manager(ioContext(), objects(), bus(), requester());
    manager.createSensors();

    ASSERT_TRUE(pumpIoUntil([this] { return lookups > 0; }, discoveryTimeout));
    pumpIoUntil([] { return false; }, quietWindow);

    EXPECT_EQ(lookups, 1);
    EXPECT_EQ(configReads, 0);
}

constexpr uint8_t smaEid = 40;
constexpr const char* smaRecordPath = "/test/board/SMA";

std::string smaElementIface(std::string_view array, size_t index)
{
    return std::format(
        "xyz.openbmc_project.Configuration.NvidiaMctpVdmSma.{}{}", array,
        index);
}

// Entity manager exports the Id of a sensor list entry as uint64_t.
SensorBaseConfigMap smaSensor(uint64_t id, const std::string& name)
{
    return {{"Id", id}, {"Name", name}};
}

MctpVdmRecord smaRecord(const SensorData& arrayElements)
{
    return {.path = std::string{smaRecordPath},
            .name = "SMA",
            .properties = {},
            .arrayElements = arrayElements};
}

// Thresholds is a settable interface, so entity manager exports its
// numbers as doubles.
SensorBaseConfigMap smaThreshold(const std::string& label, double severity,
                                 double value)
{
    return {{"Direction", std::string{"greater than"}},
            {"Label", label},
            {"Name", std::string{"upper threshold"}},
            {"Severity", severity},
            {"Value", value}};
}

thresholds::Threshold upperThreshold(thresholds::Level level, double value)
{
    return {level, thresholds::Direction::HIGH, value};
}

TEST(SmaDeviceConfigsTest, ListsTheSensorsInRecordOrder)
{
    SensorData arrayElements{
        {smaElementIface("PowerSensors", 0), smaSensor(128, "HSC")},
        {smaElementIface("VoltageSensors", 0), smaSensor(208, "HSCC_Vout")}};
    // Past ten entries, the interface names no longer sort in record order.
    for (size_t index = 0; index < 12; ++index)
    {
        arrayElements.emplace(smaElementIface("TemperatureSensors", index),
                              smaSensor(index, std::format("T{}", index)));
    }

    const SmaDeviceConfigs smaConfig =
        readSmaDeviceConfigs(smaRecord(arrayElements), smaEid);

    ASSERT_EQ(smaConfig.sensors.size(), 14U);
    for (size_t index = 0; index < 12; ++index)
    {
        EXPECT_EQ(smaConfig.sensors[index].kind, SmaSensorKind::Temperature);
        EXPECT_EQ(smaConfig.sensors[index].name, std::format("T{}", index));
    }
    EXPECT_EQ(smaConfig.sensors[12].kind, SmaSensorKind::Power);
    EXPECT_EQ(smaConfig.sensors[12].id, 128);
    EXPECT_EQ(smaConfig.sensors[12].name, "HSC");
    EXPECT_EQ(smaConfig.sensors[13].kind, SmaSensorKind::Voltage);
    EXPECT_EQ(smaConfig.sensors[13].id, 208);
    EXPECT_EQ(smaConfig.sensors[13].name, "HSCC_Vout");
}

TEST(SmaDeviceConfigsTest, LeavesOutAnEntryWithoutAnIdOrAName)
{
    const SensorData arrayElements{
        {smaElementIface("TemperatureSensors", 0),
         {{"Name", std::string{"NoId"}}}},
        {smaElementIface("TemperatureSensors", 1), {{"Id", uint64_t{17}}}},
        {smaElementIface("TemperatureSensors", 2), smaSensor(16, "SMA_Ext")}};

    const SmaDeviceConfigs smaConfig =
        readSmaDeviceConfigs(smaRecord(arrayElements), smaEid);

    ASSERT_EQ(smaConfig.sensors.size(), 1U);
    EXPECT_EQ(smaConfig.sensors[0].id, 16);
    EXPECT_EQ(smaConfig.sensors[0].name, "SMA_Ext");
}

TEST(SmaDeviceConfigsTest, ReadsAnIdOfAnyNumericType)
{
    const SensorData arrayElements{
        {smaElementIface("TemperatureSensors", 0),
         {{"Id", int64_t{5}}, {"Name", std::string{"Signed"}}}},
        {smaElementIface("TemperatureSensors", 1),
         {{"Id", 17.0}, {"Name", std::string{"Double"}}}},
        {smaElementIface("TemperatureSensors", 2),
         {{"Id", std::string{"18"}}, {"Name", std::string{"String"}}}}};

    const SmaDeviceConfigs smaConfig =
        readSmaDeviceConfigs(smaRecord(arrayElements), smaEid);

    ASSERT_EQ(smaConfig.sensors.size(), 2U);
    EXPECT_EQ(smaConfig.sensors[0].id, 5);
    EXPECT_EQ(smaConfig.sensors[0].name, "Signed");
    EXPECT_EQ(smaConfig.sensors[1].id, 17);
    EXPECT_EQ(smaConfig.sensors[1].name, "Double");
}

TEST(SmaDeviceConfigsTest, LeavesOutIdsFrom255)
{
    const SensorData arrayElements{
        {smaElementIface("TemperatureSensors", 0), smaSensor(254, "Last")},
        {smaElementIface("TemperatureSensors", 1), smaSensor(255, "All")},
        {smaElementIface("TemperatureSensors", 2), smaSensor(256, "Over")}};

    const SmaDeviceConfigs smaConfig =
        readSmaDeviceConfigs(smaRecord(arrayElements), smaEid);

    ASSERT_EQ(smaConfig.sensors.size(), 1U);
    EXPECT_EQ(smaConfig.sensors[0].id, 254);
    EXPECT_EQ(smaConfig.sensors[0].name, "Last");
}

TEST(SmaDeviceConfigsTest, KeepsTheFirstEntryOfARepeatedIdOrName)
{
    const SensorData arrayElements{
        {smaElementIface("TemperatureSensors", 2), smaSensor(16, "SMA_Ext")},
        {smaElementIface("TemperatureSensors", 3), smaSensor(17, "SMA Ext")},
        {smaElementIface("TemperatureSensors", 4), smaSensor(18, "GPU_VR")},
        {smaElementIface("TemperatureSensors", 10), smaSensor(16, "Other")},
        {smaElementIface("PowerSensors", 0), smaSensor(16, "SMA_Ext")}};

    const SmaDeviceConfigs smaConfig =
        readSmaDeviceConfigs(smaRecord(arrayElements), smaEid);

    ASSERT_EQ(smaConfig.sensors.size(), 3U);
    EXPECT_EQ(smaConfig.sensors[0].kind, SmaSensorKind::Temperature);
    EXPECT_EQ(smaConfig.sensors[0].id, 16);
    EXPECT_EQ(smaConfig.sensors[0].name, "SMA_Ext");
    EXPECT_EQ(smaConfig.sensors[1].kind, SmaSensorKind::Temperature);
    EXPECT_EQ(smaConfig.sensors[1].id, 18);
    EXPECT_EQ(smaConfig.sensors[1].name, "GPU_VR");
    EXPECT_EQ(smaConfig.sensors[2].kind, SmaSensorKind::Power);
    EXPECT_EQ(smaConfig.sensors[2].id, 16);
    EXPECT_EQ(smaConfig.sensors[2].name, "SMA_Ext");
}

TEST(SmaDeviceConfigsTest, ReadsTheInventoryAnEntryNames)
{
    SensorBaseConfigMap named = smaSensor(216, "CPU1_Die");
    named["Inventory"] = std::string{"CPU_0"};
    SensorBaseConfigMap notAString = smaSensor(217, "CPU1_SoC");
    notAString["Inventory"] = uint64_t{0};
    const SensorData arrayElements{
        {smaElementIface("TemperatureSensors", 0), named},
        {smaElementIface("TemperatureSensors", 1), notAString},
        {smaElementIface("TemperatureSensors", 2), smaSensor(17, "SMA_Int")}};

    const SmaDeviceConfigs smaConfig =
        readSmaDeviceConfigs(smaRecord(arrayElements), smaEid);

    ASSERT_EQ(smaConfig.sensors.size(), 3U);
    EXPECT_EQ(smaConfig.sensors[0].inventory, "CPU_0");
    EXPECT_EQ(smaConfig.sensors[1].name, "CPU1_SoC");
    EXPECT_EQ(smaConfig.sensors[1].inventory, std::nullopt);
    EXPECT_EQ(smaConfig.sensors[2].inventory, std::nullopt);
}

TEST(SmaDeviceConfigsTest, AssignsTheThresholdsTheRecordLists)
{
    SensorBaseConfigMap dynamic = smaThreshold("SMA_Ext", 0, 0);
    dynamic.erase("Value");
    dynamic["Dynamic"] = true;
    const SensorData arrayElements{
        {smaElementIface("TemperatureSensors", 0), smaSensor(16, "SMA_Ext")},
        {smaElementIface("TemperatureSensors", 1), smaSensor(18, "GPU_VR")},
        {smaElementIface("Thresholds", 0), dynamic},
        {smaElementIface("Thresholds", 1), smaThreshold("GPU_VR", 1, 130)}};

    const SmaDeviceConfigs smaConfig =
        readSmaDeviceConfigs(smaRecord(arrayElements), smaEid);

    ASSERT_EQ(smaConfig.sensors.size(), 2U);
    EXPECT_THAT(smaConfig.sensors[0].thresholds, testing::IsEmpty());
    EXPECT_EQ(smaConfig.sensors[0].dynamicThresholds.size(), 1U);
    EXPECT_THAT(
        smaConfig.sensors[1].thresholds,
        testing::ElementsAre(upperThreshold(thresholds::Level::CRITICAL, 130)));
    EXPECT_THAT(smaConfig.sensors[1].dynamicThresholds, testing::IsEmpty());
}

TEST(SmaThresholdsTest, AssignsThresholdsToTheTemperatureSensorTheyLabel)
{
    SmaDeviceConfigs smaConfig{
        .sensors = {
            {.kind = SmaSensorKind::Temperature, .id = 16, .name = "SMA_Ext"},
            {.kind = SmaSensorKind::Temperature,
             .id = 17,
             .name = "SMA_Internal"},
            {.kind = SmaSensorKind::Temperature, .id = 18, .name = "GPU_VR"}}};
    const SensorData thresholdConfigs{
        {smaElementIface("Thresholds", 0), smaThreshold("SMA_Ext", 0, 120)},
        {smaElementIface("Thresholds", 1), smaThreshold("GPU_VR", 4, 135)},
        {smaElementIface("Thresholds", 2), smaThreshold("GPU_VR", 1, 130)},
        {smaElementIface("Thresholds", 3), smaThreshold("GPU_VR", 0, 120)}};

    assignSmaThresholds(smaConfig, thresholdConfigs, smaEid,
                        sdbusplus::object_path{smaRecordPath});

    EXPECT_THAT(smaConfig.sensors[0].thresholds,
                testing::UnorderedElementsAre(
                    upperThreshold(thresholds::Level::WARNING, 120)));
    EXPECT_THAT(smaConfig.sensors[1].thresholds, testing::IsEmpty());
    EXPECT_THAT(smaConfig.sensors[2].thresholds,
                testing::UnorderedElementsAre(
                    upperThreshold(thresholds::Level::WARNING, 120),
                    upperThreshold(thresholds::Level::CRITICAL, 130),
                    upperThreshold(thresholds::Level::HARDSHUTDOWN, 135)));
}

TEST(SmaThresholdsTest, LeavesOutThresholdsThatLabelNoSensor)
{
    SmaDeviceConfigs smaConfig{
        .sensors = {
            {.kind = SmaSensorKind::Temperature, .id = 16, .name = "SMA_Ext"}}};
    SensorBaseConfigMap unlabeled = smaThreshold("SMA_Ext", 1, 130);
    unlabeled.erase("Label");
    const SensorData thresholdConfigs{
        {smaElementIface("Thresholds", 0), smaThreshold("SMA_EXT", 0, 120)},
        {smaElementIface("Thresholds", 1), unlabeled}};

    assignSmaThresholds(smaConfig, thresholdConfigs, smaEid,
                        sdbusplus::object_path{smaRecordPath});

    EXPECT_THAT(smaConfig.sensors[0].thresholds, testing::IsEmpty());
}

TEST(SmaThresholdsTest, KeepsTheThresholdsNextToOneThatLabelsNoSensor)
{
    SmaDeviceConfigs smaConfig{
        .sensors = {
            {.kind = SmaSensorKind::Temperature, .id = 16, .name = "SMA_Ext"}}};
    SensorBaseConfigMap dynamic = smaThreshold("SMA_Ext", 1, 0);
    dynamic.erase("Value");
    dynamic["Dynamic"] = true;
    SensorBaseConfigMap mislabeledDynamic = dynamic;
    mislabeledDynamic["Label"] = std::string{"SMA_Internal"};
    const SensorData thresholdConfigs{
        {smaElementIface("Thresholds", 0),
         smaThreshold("SMA_Internal", 0, 110)},
        {smaElementIface("Thresholds", 1), smaThreshold("SMA_Ext", 0, 120)},
        {smaElementIface("Thresholds", 2), mislabeledDynamic},
        {smaElementIface("Thresholds", 3), dynamic}};

    assignSmaThresholds(smaConfig, thresholdConfigs, smaEid,
                        sdbusplus::object_path{smaRecordPath});

    EXPECT_THAT(
        smaConfig.sensors[0].thresholds,
        testing::ElementsAre(upperThreshold(thresholds::Level::WARNING, 120)));
    EXPECT_THAT(smaConfig.sensors[0].dynamicThresholds,
                testing::ElementsAre(testing::AllOf(
                    testing::Field(&thresholds::Threshold::level,
                                   thresholds::Level::CRITICAL),
                    testing::Field(&thresholds::Threshold::direction,
                                   thresholds::Direction::HIGH))));
}

TEST(SmaThresholdsTest, KeepsTheThresholdsReadBeforeAMalformedOne)
{
    SmaDeviceConfigs smaConfig{
        .sensors = {
            {.kind = SmaSensorKind::Temperature, .id = 18, .name = "GPU_VR"},
            {.kind = SmaSensorKind::Temperature, .id = 16, .name = "SMA_Ext"}}};
    SensorBaseConfigMap noValue = smaThreshold("GPU_VR", 1, 130);
    noValue.erase("Value");
    const SensorData thresholdConfigs{
        {smaElementIface("Thresholds", 0), smaThreshold("GPU_VR", 0, 120)},
        {smaElementIface("Thresholds", 1), noValue},
        {smaElementIface("Thresholds", 2), smaThreshold("SMA_Ext", 0, 120)}};

    assignSmaThresholds(smaConfig, thresholdConfigs, smaEid,
                        sdbusplus::object_path{smaRecordPath});

    EXPECT_THAT(smaConfig.sensors[0].thresholds,
                testing::UnorderedElementsAre(
                    upperThreshold(thresholds::Level::WARNING, 120)));
    EXPECT_THAT(smaConfig.sensors[1].thresholds,
                testing::UnorderedElementsAre(
                    upperThreshold(thresholds::Level::WARNING, 120)));
}

TEST(SmaThresholdsTest, AssignsNoThresholdsToPowerOrVoltageSensors)
{
    SmaDeviceConfigs smaConfig{
        .sensors = {
            {.kind = SmaSensorKind::Temperature, .id = 192, .name = "HSC"},
            {.kind = SmaSensorKind::Power, .id = 128, .name = "HSC"},
            {.kind = SmaSensorKind::Voltage, .id = 192, .name = "HSC"}}};
    const SensorData thresholdConfigs{
        {smaElementIface("Thresholds", 0), smaThreshold("HSC", 0, 120)}};

    assignSmaThresholds(smaConfig, thresholdConfigs, smaEid,
                        sdbusplus::object_path{smaRecordPath});

    EXPECT_THAT(smaConfig.sensors[0].thresholds,
                testing::UnorderedElementsAre(
                    upperThreshold(thresholds::Level::WARNING, 120)));
    EXPECT_THAT(smaConfig.sensors[1].thresholds, testing::IsEmpty());
    EXPECT_THAT(smaConfig.sensors[2].thresholds, testing::IsEmpty());
}

TEST(SmaThresholdsTest, KeepsDynamicThresholdsApartWithoutAValue)
{
    SmaDeviceConfigs smaConfig{
        .sensors = {
            {.kind = SmaSensorKind::Temperature, .id = 16, .name = "SMA_Ext"},
            {.kind = SmaSensorKind::Temperature, .id = 18, .name = "GPU_VR"}}};
    SensorBaseConfigMap dynamic = smaThreshold("SMA_Ext", 0, 0);
    dynamic.erase("Value");
    dynamic["Dynamic"] = true;
    SensorBaseConfigMap notDynamic = smaThreshold("GPU_VR", 1, 130);
    notDynamic["Dynamic"] = false;
    const SensorData thresholdConfigs{
        {smaElementIface("Thresholds", 0), dynamic},
        {smaElementIface("Thresholds", 1), notDynamic},
        {smaElementIface("Thresholds", 2), smaThreshold("GPU_VR", 0, 120)}};

    assignSmaThresholds(smaConfig, thresholdConfigs, smaEid,
                        sdbusplus::object_path{smaRecordPath});

    EXPECT_THAT(smaConfig.sensors[0].thresholds, testing::IsEmpty());
    EXPECT_THAT(smaConfig.sensors[0].dynamicThresholds,
                testing::ElementsAre(testing::AllOf(
                    testing::Field(&thresholds::Threshold::level,
                                   thresholds::Level::WARNING),
                    testing::Field(&thresholds::Threshold::direction,
                                   thresholds::Direction::HIGH))));
    EXPECT_THAT(smaConfig.sensors[1].thresholds,
                testing::UnorderedElementsAre(
                    upperThreshold(thresholds::Level::WARNING, 120),
                    upperThreshold(thresholds::Level::CRITICAL, 130)));
    EXPECT_THAT(smaConfig.sensors[1].dynamicThresholds, testing::IsEmpty());
}

constexpr const char* probeIface = "xyz.openbmc_project.Configuration.Probe";
constexpr const char* mctpConfigIface =
    "xyz.openbmc_project.Configuration.MCTPUSBDevice";
constexpr const char* mctpConfigPath =
    "/xyz/openbmc_project/inventory/system/platform/Platform/SMA";
constexpr const char* smaIface =
    "xyz.openbmc_project.Configuration.NvidiaMctpVdmSma";
constexpr const char* boardName = "STRATA_HPM_1";
constexpr const char* boardPath =
    "/xyz/openbmc_project/inventory/system/board/STRATA_HPM_1";
constexpr const char* cpuName = "CPU_0";
constexpr const char* cpuPath =
    "/xyz/openbmc_project/inventory/system/cpu/CPU_0";

const Association chassisAssociation{"chassis", "all_sensors", boardPath};

Association inventoryAssociation(const std::string& itemPath)
{
    return {"inventory", "sensors", itemPath};
}

bool isQueryDeviceIdentification(std::span<const uint8_t> request)
{
    UnpackBuffer unpack(request);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    if (ocp::accelerator_management::unpackHeader(
            unpack, gpu::nvidiaPciVendorId, ocpMsgType, instanceId, msgType) !=
        0)
    {
        return false;
    }

    uint8_t command = 0;
    unpack.unpack(command);
    return unpack.getError() == 0 &&
           msgType == static_cast<uint8_t>(
                          gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY) &&
           command ==
               static_cast<uint8_t>(gpu::DeviceCapabilityDiscoveryCommands::
                                        QUERY_DEVICE_IDENTIFICATION);
}

std::vector<uint8_t> smaIdentificationResponse()
{
    // deviceIdentification + deviceInstance
    const uint16_t dataSize = 1 + 1;

    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + dataSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY));
    pack.pack(static_cast<uint8_t>(
        gpu::DeviceCapabilityDiscoveryCommands::QUERY_DEVICE_IDENTIFICATION));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(dataSize);
    pack.pack(static_cast<uint8_t>(gpu::DeviceIdentification::DEVICE_SMA));
    pack.pack(static_cast<uint8_t>(0)); // deviceInstance
    return buf;
}

// Stands in for entity manager, the mapper and mctpd, and answers the device
// identification as an SMA, so a discovery sweep runs as far as publishing
// the sensors the SMA record of the board lists.
class SmaInventoryTest : public MctpMockTestBase
{
  protected:
    void SetUp() override
    {
        MctpMockTestBase::SetUp();
        if (IsSkipped())
        {
            return;
        }

        ownMapperName(*bus());
        const std::string self = bus()->get_unique_name();

        auto mapper = addInterface("/xyz/openbmc_project/object_mapper",
                                   "xyz.openbmc_project.ObjectMapper");
        mapper->register_method(
            "GetSubTree",
            [this, self](const std::string& /*root*/, int32_t /*depth*/,
                         const std::vector<std::string>& ifaces) {
                SubTree tree;
                if (ifaces == std::vector<std::string>{endpointIface})
                {
                    tree[endpointPath] = {{self, {endpointIface}}};
                }
                else if (ifaces == std::vector<std::string>{probeIface})
                {
                    for (const std::string& entity : entities)
                    {
                        tree[entity] = {{self, {probeIface}}};
                    }
                }
                return tree;
            });
        mapper->register_method(
            "GetAssociatedSubTree",
            [self](const sdbusplus::object_path& /*association*/,
                   const sdbusplus::object_path& /*root*/, int32_t /*depth*/,
                   const std::vector<std::string>& /*ifaces*/) {
                return SubTree{{mctpConfigPath, {{self, {mctpConfigIface}}}}};
            });
        mapper->register_method(
            "GetObject",
            [this, self](const std::string& path,
                         const std::vector<std::string>& /*ifaces*/) {
                ObjectServices services;
                if (path == smaPath)
                {
                    services[self] = smaIfaces;
                }
                return services;
            });
        mapper->initialize();

        auto endpoint = addInterface(endpointPath, endpointIface);
        endpoint->register_property("EID", endpointEid);
        endpoint->register_property(
            "SupportedMessageTypes",
            std::vector<uint8_t>{ocp::accelerator_management::messageType});
        endpoint->initialize();

        addEntity(boardName, boardPath);

        ON_CALL(mctpMock, sendRecvMsg)
            .WillByDefault([this](uint8_t /*eid*/,
                                  std::span<const uint8_t> reqMsg,
                                  auto callback) {
                if (!isQueryDeviceIdentification(reqMsg))
                {
                    callback(std::make_error_code(std::errc::timed_out), {});
                    return;
                }
                ++identifications;
                if (withholdIdentification)
                {
                    withholdIdentification = false;
                    withheldIdentification = std::move(callback);
                    return;
                }
                const std::vector<uint8_t> response =
                    smaIdentificationResponse();
                callback(std::error_code{}, response);
            });
    }

    void TearDown() override
    {
        if (hasBus())
        {
            for (const auto& fake : fakes)
            {
                objects().remove_interface(fake);
            }
        }
        MctpMockTestBase::TearDown();
    }

    std::shared_ptr<sdbusplus::asio::dbus_interface> addInterface(
        const std::string& path, const std::string& iface)
    {
        return fakes.emplace_back(objects().add_interface(path, iface));
    }

    void addEntity(const std::string& name, const std::string& path)
    {
        auto probe = addInterface(path, probeIface);
        probe->register_property("Name", name);
        probe->initialize();
        entities.push_back(path);
    }

    void addSensorEntry(std::string_view list, size_t index, uint64_t id,
                        const std::string& name,
                        const std::optional<std::string>& inventory)
    {
        const std::string iface = smaElementIface(list, index);
        auto entry = addInterface(smaPath, iface);
        entry->register_property("Id", id);
        entry->register_property("Name", name);
        if (inventory)
        {
            entry->register_property("Inventory", *inventory);
        }
        entry->initialize();
        smaIfaces.push_back(iface);
    }

    void addSma(const std::string& name)
    {
        smaName = name;
        smaPath = std::format("{}/{}", boardPath, name);

        auto mctpConfig = addInterface(mctpConfigPath, mctpConfigIface);
        mctpConfig->register_property("Name", name);
        mctpConfig->register_property("Board", std::string{boardName});
        mctpConfig->initialize();

        auto record = addInterface(smaPath, smaIface);
        record->register_property("Name", name);
        record->initialize();
        smaIfaces.emplace_back(smaIface);

        addSensorEntry("TemperatureSensors", 0, 216, "CPU1_Die", cpuName);
        addSensorEntry("TemperatureSensors", 1, 18, "GPU_VR", boardName);
        addSensorEntry("TemperatureSensors", 2, 17, "SMA_Internal",
                       std::nullopt);
        addSensorEntry("PowerSensors", 0, 128, "HSC", "HSC_HPM_1");
    }

    std::vector<Association> associations(const std::string& type,
                                          const std::string& sensor) const
    {
        return getProperty<std::vector<Association>>(
            std::format("/xyz/openbmc_project/sensors/{}/{}_{}", type, smaName,
                        sensor),
            "xyz.openbmc_project.Association.Definitions", "Associations");
    }

    // The inventory a sensor names is looked up after the sensor is
    // published, and reading a property runs the io loop the answer needs.
    bool hasInventory(const std::string& type, const std::string& sensor,
                      const std::string& itemPath) const
    {
        for (int attempt = 0; attempt < 50; ++attempt)
        {
            if (std::ranges::contains(associations(type, sensor),
                                      inventoryAssociation(itemPath)))
            {
                return true;
            }
        }
        return false;
    }

    std::string smaName;
    std::string smaPath;
    std::vector<std::string> smaIfaces;
    // The objects entity manager exports Configuration.Probe on.
    std::vector<std::string> entities;
    std::vector<std::shared_ptr<sdbusplus::asio::dbus_interface>> fakes;

    // How many device identifications were asked for: one per sweep.
    int identifications = 0;
    // Keep the answer to the next device identification back, for the test
    // to give when it chooses.
    bool withholdIdentification = false;
    std::move_only_function<void(const std::error_code&,
                                 std::span<const uint8_t>)>
        withheldIdentification;
};

TEST_F(SmaInventoryTest, AssociatesSensorsWithTheInventoryTheirEntriesName)
{
    addSma("sma_named");
    addEntity(cpuName, cpuPath);

    DeviceManager manager(ioContext(), objects(), bus(), requester());
    manager.createSensors();
    ASSERT_TRUE(
        pumpIoUntil([this] { return identifications > 0; }, discoveryTimeout));

    ASSERT_TRUE(hasInventory("temperature", "CPU1_Die", cpuPath));
    EXPECT_THAT(associations("temperature", "CPU1_Die"),
                testing::UnorderedElementsAre(chassisAssociation,
                                              inventoryAssociation(cpuPath)));
    EXPECT_THAT(associations("temperature", "GPU_VR"),
                testing::UnorderedElementsAre(chassisAssociation,
                                              inventoryAssociation(boardPath)));
    EXPECT_THAT(associations("temperature", "SMA_Internal"),
                testing::ElementsAre(chassisAssociation));
    EXPECT_THAT(associations("power", "HSC"),
                testing::ElementsAre(chassisAssociation));
}

TEST_F(SmaInventoryTest, AssociatesSensorsOnceTheirInventoryIsExported)
{
    addSma("sma_exported_later");

    DeviceManager manager(ioContext(), objects(), bus(), requester());
    manager.createSensors();
    ASSERT_TRUE(
        pumpIoUntil([this] { return identifications > 0; }, discoveryTimeout));

    EXPECT_THAT(associations("temperature", "CPU1_Die"),
                testing::ElementsAre(chassisAssociation));

    addEntity(cpuName, cpuPath);
    manager.scheduleRescan();
    ASSERT_TRUE(
        pumpIoUntil([this] { return identifications > 1; }, discoveryTimeout));

    ASSERT_TRUE(hasInventory("temperature", "CPU1_Die", cpuPath));
    EXPECT_THAT(associations("temperature", "CPU1_Die"),
                testing::UnorderedElementsAre(chassisAssociation,
                                              inventoryAssociation(cpuPath)));
    EXPECT_THAT(associations("power", "HSC"),
                testing::ElementsAre(chassisAssociation));
}

// A sweep empties the paths it has looked up, and the answer that creates a
// device can arrive before they are looked up again.
TEST_F(SmaInventoryTest, AssociatesSensorsCreatedAfterThePathsAreEmptied)
{
    addSma("sma_collecting");
    addEntity(cpuName, cpuPath);
    withholdIdentification = true;

    DeviceManager manager(ioContext(), objects(), bus(), requester());
    manager.createSensors();
    ASSERT_TRUE(pumpIoUntil(
        [this] { return static_cast<bool>(withheldIdentification); },
        discoveryTimeout));

    manager.createSensors();
    const std::vector<uint8_t> response = smaIdentificationResponse();
    withheldIdentification(std::error_code{}, response);
    // Reading a property runs the io loop, which lets the second sweep
    // finish, so the sensors cannot be read as they were published.
    ASSERT_TRUE(
        pumpIoUntil([this] { return identifications > 1; }, discoveryTimeout));

    ASSERT_TRUE(hasInventory("temperature", "CPU1_Die", cpuPath));
    EXPECT_THAT(associations("temperature", "CPU1_Die"),
                testing::UnorderedElementsAre(chassisAssociation,
                                              inventoryAssociation(cpuPath)));
    EXPECT_THAT(associations("temperature", "GPU_VR"),
                testing::UnorderedElementsAre(chassisAssociation,
                                              inventoryAssociation(boardPath)));
}

} // namespace
