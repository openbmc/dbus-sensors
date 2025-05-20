#include "MCTPEndpoint.hpp"
#include "Utils.hpp"

#include <cstdint>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

TEST(I2CMCTPDDevice, matchEmptyConfig)
{
    SensorData config{};
    EXPECT_FALSE(I2CMCTPDDevice::match(config));
}

TEST(I2CMCTPDDevice, matchIrrelevantConfig)
{
    SensorData config{{"xyz.openbmc_project.Configuration.NVME1000", {}}};
    EXPECT_FALSE(I2CMCTPDDevice::match(config));
}

TEST(I2CMCTPDDevice, matchRelevantConfig)
{
    SensorData config{{"xyz.openbmc_project.Configuration.MCTPI2CTarget", {}}};
    EXPECT_TRUE(I2CMCTPDDevice::match(config));
}

TEST(I2CMCTPDDevice, fromBadIfaceNoType)
{
    SensorBaseConfigMap iface{{}};
    EXPECT_THROW(I2CMCTPDDevice::from({}, iface), std::invalid_argument);
}

TEST(I2CMCTPDDevice, fromBadIfaceWrongType)
{
    SensorBaseConfigMap iface{{"Type", "NVME1000"}};
    EXPECT_THROW(I2CMCTPDDevice::from({}, iface), std::invalid_argument);
}

TEST(I2CMCTPDDevice, fromBadIfaceNoAddress)
{
    SensorBaseConfigMap iface{
        {"Bus", "0"},
        {"Name", "test"},
        {"Type", "MCTPI2CTarget"},
    };
    EXPECT_THROW(I2CMCTPDDevice::from({}, iface), std::invalid_argument);
}

TEST(I2CMCTPDDevice, fromBadIfaceBadAddress)
{
    SensorBaseConfigMap iface{
        {"Address", "not a number"},
        {"Bus", "0"},
        {"Name", "test"},
        {"Type", "MCTPI2CTarget"},
    };
    EXPECT_THROW(I2CMCTPDDevice::from({}, iface), std::invalid_argument);
}

TEST(I2CMCTPDDevice, fromBadIfaceNoBus)
{
    SensorBaseConfigMap iface{
        {"Address", "0x1d"},
        {"Name", "test"},
        {"Type", "MCTPI2CTarget"},
    };
    EXPECT_THROW(I2CMCTPDDevice::from({}, iface), std::invalid_argument);
}

TEST(I2CMCTPDDevice, fromBadIfaceBadBus)
{
    SensorBaseConfigMap iface{
        {"Address", "0x1d"},
        {"Bus", "not a number"},
        {"Name", "test"},
        {"Type", "MCTPI2CTarget"},
    };
    EXPECT_THROW(I2CMCTPDDevice::from({}, iface), std::invalid_argument);
}

TEST(I2CMCTPDDevice, fromBadIfaceNoName)
{
    SensorBaseConfigMap iface{
        {"Address", "0x1d"},
        {"Bus", "0"},
        {"Type", "MCTPI2CTarget"},
    };
    EXPECT_THROW(I2CMCTPDDevice::from({}, iface), std::invalid_argument);
}

TEST(USBMCTPDDevice, matchEmptyConfig)
{
    SensorData config{};
    EXPECT_FALSE(USBMCTPDDevice::match(config));
}

TEST(USBMCTPDDevice, matchIrrelevantConfig)
{
    SensorData config{{"xyz.openbmc_project.Configuration.MCTPI2CTarget", {}}};
    EXPECT_FALSE(USBMCTPDDevice::match(config));
}

TEST(USBMCTPDDevice, matchRelevantConfig)
{
    SensorData config{{"xyz.openbmc_project.Configuration.MCTPUSBDevice", {}}};
    EXPECT_TRUE(USBMCTPDDevice::match(config));
}

// A device is located by walking down from a root hub, so a configuration
// has to name every step of the way: which root hub, which ports to follow,
// and which interface of which configuration of the device found at the end.
// Each test below takes this away or spoils one of those.
static SensorBaseConfigMap usbDeviceConfig()
{
    return {
        {"Configuration", "1"},   {"Interface", "0"},
        {"Name", "test"},         {"Port", std::vector<uint64_t>{1, 2, 1}},
        {"RootHubPosition", "0"}, {"Type", "MCTPUSBDevice"},
    };
}

TEST(USBMCTPDDevice, fromBadIfaceNoType)
{
    SensorBaseConfigMap iface{{}};
    EXPECT_THROW(USBMCTPDDevice::from({}, iface), std::invalid_argument);
}

TEST(USBMCTPDDevice, fromBadIfaceWrongType)
{
    SensorBaseConfigMap iface = usbDeviceConfig();
    iface["Type"] = "MCTPI2CTarget";
    EXPECT_THROW(USBMCTPDDevice::from({}, iface), std::invalid_argument);
}

TEST(USBMCTPDDevice, fromBadIfaceNoName)
{
    SensorBaseConfigMap iface = usbDeviceConfig();
    iface.erase("Name");
    EXPECT_THROW(USBMCTPDDevice::from({}, iface), std::invalid_argument);
}

TEST(USBMCTPDDevice, fromBadIfaceNoRootHubPosition)
{
    SensorBaseConfigMap iface = usbDeviceConfig();
    iface.erase("RootHubPosition");
    EXPECT_THROW(USBMCTPDDevice::from({}, iface), std::invalid_argument);
}

TEST(USBMCTPDDevice, fromBadIfaceNoPort)
{
    SensorBaseConfigMap iface = usbDeviceConfig();
    iface.erase("Port");
    EXPECT_THROW(USBMCTPDDevice::from({}, iface), std::invalid_argument);
}

TEST(USBMCTPDDevice, fromBadIfaceNoConfiguration)
{
    SensorBaseConfigMap iface = usbDeviceConfig();
    iface.erase("Configuration");
    EXPECT_THROW(USBMCTPDDevice::from({}, iface), std::invalid_argument);
}

TEST(USBMCTPDDevice, fromBadIfaceNoInterface)
{
    SensorBaseConfigMap iface = usbDeviceConfig();
    iface.erase("Interface");
    EXPECT_THROW(USBMCTPDDevice::from({}, iface), std::invalid_argument);
}

// Only the two root hubs the SOC has can be named.
TEST(USBMCTPDDevice, fromBadIfaceBadRootHubPosition)
{
    SensorBaseConfigMap iface = usbDeviceConfig();
    iface["RootHubPosition"] = "2";
    EXPECT_THROW(USBMCTPDDevice::from({}, iface), std::invalid_argument);
}

// An empty port list names the root hub itself rather than a device on it.
TEST(USBMCTPDDevice, fromBadIfaceEmptyPort)
{
    SensorBaseConfigMap iface = usbDeviceConfig();
    iface["Port"] = std::vector<uint64_t>{};
    EXPECT_THROW(USBMCTPDDevice::from({}, iface), std::invalid_argument);
}

TEST(USBMCTPDDevice, fromBadIfaceBadConfiguration)
{
    SensorBaseConfigMap iface = usbDeviceConfig();
    iface["Configuration"] = "not a number";
    EXPECT_THROW(USBMCTPDDevice::from({}, iface), std::invalid_argument);
}

// from_chars stops at the first character it cannot use rather than failing,
// so a value has to be rejected on what is left over as well.
TEST(USBMCTPDDevice, fromBadIfaceTrailingConfiguration)
{
    SensorBaseConfigMap iface = usbDeviceConfig();
    iface["Configuration"] = "1abc";
    EXPECT_THROW(USBMCTPDDevice::from({}, iface), std::invalid_argument);
}

TEST(USBMCTPDDevice, fromBadIfaceBadInterface)
{
    SensorBaseConfigMap iface = usbDeviceConfig();
    iface["Interface"] = "not a number";
    EXPECT_THROW(USBMCTPDDevice::from({}, iface), std::invalid_argument);
}

TEST(USBMCTPDDevice, fromBadIfaceTrailingInterface)
{
    SensorBaseConfigMap iface = usbDeviceConfig();
    iface["Interface"] = "0abc";
    EXPECT_THROW(USBMCTPDDevice::from({}, iface), std::invalid_argument);
}
