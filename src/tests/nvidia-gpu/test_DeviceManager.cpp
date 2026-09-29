/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "DbusMockTestBase.hpp"
#include "DeviceManager.hpp"
#include "NvidiaGpuControlErrors.hpp"
#include "NvidiaSmaDevice.hpp"
#include "OcpMctpVdm.hpp"
#include "Utils.hpp"

#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/exception.hpp>
#include <sdbusplus/vtable.hpp>

#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

namespace
{

// What the mapper answers when nothing is at the path it was asked about.
struct ResourceNotFound : sdbusplus::exception_t
{
    const char* name() const noexcept override
    {
        return "xyz.openbmc_project.Common.Error.ResourceNotFound";
    }
    const char* description() const noexcept override
    {
        return "The resource is not found.";
    }
    int get_errno() const noexcept override
    {
        return ENOENT;
    }
};

using ObjectServices = std::map<std::string, std::vector<std::string>>;
using SubTree = std::map<std::string, ObjectServices>;

constexpr const char* endpointPath =
    "/au/com/codeconstruct/mctp1/networks/1/endpoints/9";
constexpr const char* associationPath =
    "/au/com/codeconstruct/mctp1/networks/1/endpoints/9/configured_by";
constexpr const char* endpointIface = "xyz.openbmc_project.MCTP.Endpoint";
constexpr const char* associationIface = "xyz.openbmc_project.Association";
constexpr uint8_t endpointEid = 9;

// A retry waits out the rescan debounce of one second, so this is room for
// a retry to have happened.
constexpr std::chrono::seconds discoveryTimeout{5};

// Longer than the rescan debounce, so a retry that was going to happen would
// have by the end of it.
constexpr std::chrono::seconds quietWindow{2};

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

        // The name stays with the connection for the rest of the binary, and
        // sd-bus refuses a second request for it, so it is asked for once.
        [[maybe_unused]] static const bool mapperNameOwned = [] {
            bus()->request_name("xyz.openbmc_project.ObjectMapper");
            return true;
        }();
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
            "GetObject", [this](const std::string& /*path*/,
                                const std::vector<std::string>& /*ifaces*/) {
                ++getObjectCalls;
                return getObject(getObjectCalls);
            });
        mapper->initialize();

        endpoint = objects().add_interface(endpointPath, endpointIface);
        endpoint->register_property("EID", endpointEid);
        endpoint->register_property(
            "SupportedMessageTypes",
            std::vector<uint8_t>{ocp::accelerator_management::messageType});
        endpoint->initialize();

        association =
            objects().add_interface(associationPath, associationIface);
        association->register_property_r<std::vector<std::string>>(
            "endpoints", sdbusplus::vtable::property_::emits_change,
            [this](const std::vector<std::string>& /*current*/) {
                ++endpointsReads;
                return std::vector<std::string>{};
            });
        association->initialize();

        // Publishing the interface read it once; only reads after this count.
        endpointsReads = 0;
    }

    void TearDown() override
    {
        if (hasBus())
        {
            objects().remove_interface(association);
            objects().remove_interface(endpoint);
            objects().remove_interface(mapper);
        }
        DbusMockTestBase::TearDown();
    }

    // The mapper's answer for the configured_by lookup, by call number.
    std::function<ObjectServices(int)> getObject;
    int getObjectCalls = 0;
    // How often the association's endpoints were read, which only happens
    // once a lookup has found it.
    int endpointsReads = 0;

    std::shared_ptr<sdbusplus::asio::dbus_interface> mapper;
    std::shared_ptr<sdbusplus::asio::dbus_interface> endpoint;
    std::shared_ptr<sdbusplus::asio::dbus_interface> association;
};

TEST_F(DeviceManagerTest, RetriesTransientAssociationLookupFailure)
{
    const std::string self = bus()->get_unique_name();
    getObject = [self](int call) -> ObjectServices {
        if (call == 1)
        {
            throw Unavailable();
        }
        return {{self, {associationIface}}};
    };

    DeviceManager manager(ioContext(), objects(), bus(), requester());
    manager.createSensors();

    EXPECT_TRUE(
        pumpIoUntil([this] { return endpointsReads > 0; }, discoveryTimeout));
    EXPECT_EQ(getObjectCalls, 2);
}

TEST_F(DeviceManagerTest, SkipsMissingAssociation)
{
    getObject = [](int) -> ObjectServices { throw ResourceNotFound(); };

    DeviceManager manager(ioContext(), objects(), bus(), requester());
    manager.createSensors();

    ASSERT_TRUE(
        pumpIoUntil([this] { return getObjectCalls > 0; }, discoveryTimeout));
    pumpIoUntil([] { return false; }, quietWindow);

    EXPECT_EQ(getObjectCalls, 1);
    EXPECT_EQ(endpointsReads, 0);
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

} // namespace
