/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "DbusMockTestBase.hpp"
#include "NvidiaIbPort.hpp"
#include "TestUtils.hpp"

#include <sdbusplus/exception.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace
{

// Ports are numbered from one, as GetPortTelemetryCounters requires.
constexpr uint16_t defaultPortNumber = 1;

constexpr uint8_t nodeGuidTag = 3;
constexpr uint8_t portGuidTag = 4;

constexpr uint64_t testGuid = 0x0102030405060708ULL;
constexpr const char* testGuidText = "0102-0304-0506-0708";

constexpr const char* portInterface =
    "xyz.openbmc_project.Inventory.Connector.Port";
constexpr const char* networkInterface =
    "xyz.openbmc_project.Inventory.Item.NetworkInterface";

std::string portPath(const std::string& deviceName, const std::string& name)
{
    return "/xyz/openbmc_project/inventory/" + deviceName + "/" + name;
}

std::string networkDeviceFunctionPath(const std::string& deviceName,
                                      const std::string& name)
{
    return "/xyz/openbmc_project/inventory/" + deviceName +
           "/NetworkDeviceFunctions/" + name;
}

} // namespace

class NvidiaIbPortTest : public DbusMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaIbPort> createIbPort(
        const std::string& deviceName, const std::string& name = "Port_1",
        const std::vector<std::pair<uint8_t, uint64_t>>& addresses = {})
    {
        return std::make_shared<NvidiaIbPort>(
            name, deviceName, test_utils::defaultEid, defaultPortNumber,
            objects(), addresses);
    }
};

TEST_F(NvidiaIbPortTest, ConstructorCreatesInfiniBandPort)
{
    const std::string deviceName = "ib_ctor";
    const std::shared_ptr<NvidiaIbPort> ibPort = createIbPort(deviceName);
    ASSERT_NE(ibPort, nullptr);

    EXPECT_EQ(getProperty<std::string>(portPath(deviceName, "Port_1"),
                                       portInterface, "PortProtocol"),
              "xyz.openbmc_project.Inventory.Connector.Port.PortProtocol."
              "InfiniBand");
}

TEST_F(NvidiaIbPortTest, ConstructorWithGuidsCreatesNetworkDeviceFunction)
{
    const std::string deviceName = "ib_guids";
    const std::shared_ptr<NvidiaIbPort> ibPort =
        createIbPort(deviceName, "Port_1",
                     {{nodeGuidTag, testGuid}, {portGuidTag, testGuid}});
    ASSERT_NE(ibPort, nullptr);

    const std::string path = networkDeviceFunctionPath(deviceName, "Port_1");
    EXPECT_EQ(
        getProperty<std::string>(path, networkInterface, "PermanentNodeGUID"),
        testGuidText);
    EXPECT_EQ(
        getProperty<std::string>(path, networkInterface, "PermanentPortGUID"),
        testGuidText);
}

TEST_F(NvidiaIbPortTest, ConstructorWithOneGuidLeavesTheOtherEmpty)
{
    const std::string deviceName = "ib_one_guid";
    const std::shared_ptr<NvidiaIbPort> ibPort =
        createIbPort(deviceName, "Port_1", {{nodeGuidTag, testGuid}});
    ASSERT_NE(ibPort, nullptr);

    const std::string path = networkDeviceFunctionPath(deviceName, "Port_1");
    EXPECT_EQ(
        getProperty<std::string>(path, networkInterface, "PermanentNodeGUID"),
        testGuidText);
    EXPECT_EQ(
        getProperty<std::string>(path, networkInterface, "PermanentPortGUID"),
        "");
}

TEST_F(NvidiaIbPortTest, ConstructorWithoutGuidsSkipsNetworkDeviceFunction)
{
    const std::string deviceName = "ib_no_guid";
    const std::shared_ptr<NvidiaIbPort> ibPort = createIbPort(deviceName);
    ASSERT_NE(ibPort, nullptr);

    EXPECT_THROW(getProperty<std::string>(
                     networkDeviceFunctionPath(deviceName, "Port_1"),
                     networkInterface, "PermanentNodeGUID"),
                 sdbusplus::exception_t);
}

TEST_F(NvidiaIbPortTest, DestructorRemovesInterfaces)
{
    const std::string deviceName = "ib_dtor";
    std::shared_ptr<NvidiaIbPort> ibPort =
        createIbPort(deviceName, "Port_1",
                     {{nodeGuidTag, testGuid}, {portGuidTag, testGuid}});
    ASSERT_NE(ibPort, nullptr);

    ibPort.reset();

    EXPECT_THROW(getProperty<std::string>(portPath(deviceName, "Port_1"),
                                          portInterface, "PortProtocol"),
                 sdbusplus::exception_t);
    EXPECT_THROW(getProperty<std::string>(
                     networkDeviceFunctionPath(deviceName, "Port_1"),
                     networkInterface, "PermanentNodeGUID"),
                 sdbusplus::exception_t);
}
