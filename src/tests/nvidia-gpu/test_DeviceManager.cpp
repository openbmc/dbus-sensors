/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "DbusMockTestBase.hpp"
#include "DeviceManager.hpp"
#include "NvidiaGpuControlErrors.hpp"
#include "OcpMctpVdm.hpp"

#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <sdbusplus/vtable.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

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

} // namespace
