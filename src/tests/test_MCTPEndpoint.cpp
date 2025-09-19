#include "MCTPEndpoint.hpp"
#include "utils/Utils.hpp"

#include <stdexcept>

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
