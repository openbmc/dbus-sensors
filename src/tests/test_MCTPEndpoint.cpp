#include "MCTPEndpoint.hpp"
#include "Utils.hpp"

#include <stdexcept>
#include <string>

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

TEST(MCTPDDevice, staticEndpointIDAbsent)
{
    SensorBaseConfigMap iface{};
    EXPECT_EQ(mctp::details::staticEndpointIDFrom(iface), std::nullopt);
}

TEST(MCTPDDevice, staticEndpointIDFromNumber)
{
    SensorBaseConfigMap iface{{"StaticEndpointID", uint64_t{42}}};
    EXPECT_EQ(mctp::details::staticEndpointIDFrom(iface), 42);
}

TEST(MCTPDDevice, staticEndpointIDFromDecimalString)
{
    SensorBaseConfigMap iface{{"StaticEndpointID", std::string{"42"}}};
    EXPECT_EQ(mctp::details::staticEndpointIDFrom(iface), 42);
}

TEST(MCTPDDevice, staticEndpointIDFromHexString)
{
    SensorBaseConfigMap iface{{"StaticEndpointID", std::string{"0x2a"}}};
    EXPECT_EQ(mctp::details::staticEndpointIDFrom(iface), 42);
}

TEST(MCTPDDevice, staticEndpointIDRejectsMalformedValue)
{
    SensorBaseConfigMap iface{{"StaticEndpointID", std::string{"42junk"}}};
    EXPECT_THROW(mctp::details::staticEndpointIDFrom(iface),
                 std::invalid_argument);
}

TEST(MCTPDDevice, staticEndpointIDRejectsOutOfRangeValue)
{
    SensorBaseConfigMap iface{{"StaticEndpointID", uint64_t{255}}};
    EXPECT_THROW(mctp::details::staticEndpointIDFrom(iface),
                 std::invalid_argument);
}

TEST(MCTPDDevice, staticEndpointIDRejectsReservedValue)
{
    SensorBaseConfigMap iface{{"StaticEndpointID", uint64_t{7}}};
    EXPECT_THROW(mctp::details::staticEndpointIDFrom(iface),
                 std::invalid_argument);
}

TEST(I3CMCTPDDevice, matchRelevantConfig)
{
    SensorData config{{"xyz.openbmc_project.Configuration.MCTPI3CTarget", {}}};
    EXPECT_TRUE(I3CMCTPDDevice::match(config));
}

TEST(MCTPDRoutedDevice, matchRelevantConfig)
{
    SensorData config{
        {"xyz.openbmc_project.Configuration.MCTPRoutedEndpoint", {}}};
    EXPECT_TRUE(MCTPDRoutedDevice::match(config));
}

TEST(MCTPDRoutedDevice, fromValidConfig)
{
    SensorBaseConfigMap iface{
        {"EID", std::string{"12"}},
        {"Name", std::string{"BIC"}},
        {"NetworkId", uint64_t{1}},
        {"Type", std::string{"MCTPRoutedEndpoint"}},
    };
    auto device = MCTPDRoutedDevice::from({}, iface);
    ASSERT_NE(device, nullptr);
    EXPECT_EQ(device->describe(), "network: 1, EID: 12");
}

TEST(MCTPDRoutedDevice, fromMissingEID)
{
    SensorBaseConfigMap iface{
        {"Name", std::string{"BIC"}},
        {"NetworkId", uint64_t{1}},
        {"Type", std::string{"MCTPRoutedEndpoint"}},
    };
    EXPECT_THROW(MCTPDRoutedDevice::from({}, iface), std::invalid_argument);
}

TEST(MCTPDRoutedDevice, fromBadNetwork)
{
    SensorBaseConfigMap iface{
        {"EID", uint64_t{12}},
        {"Name", std::string{"BIC"}},
        {"NetworkId", std::string{"1junk"}},
        {"Type", std::string{"MCTPRoutedEndpoint"}},
    };
    EXPECT_THROW(MCTPDRoutedDevice::from({}, iface), std::invalid_argument);
}
