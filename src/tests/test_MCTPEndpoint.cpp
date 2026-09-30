#include "MCTPEndpoint.hpp"
#include "Utils.hpp"

#include <memory>
#include <stdexcept>
#include <system_error>

#include <gmock/gmock.h>
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

namespace
{

// setup() decides which mctpd method to call, then delegates the actual
// D-Bus call to a protected virtual seam. Overriding that seam here lets a
// test observe the decision without a live mctpd connection.
class MockMCTPDDevice : public MCTPDDevice
{
  public:
    using MCTPDDevice::MCTPDDevice;

    MOCK_METHOD(void, assignEndpoint, (AssignCallback), (override));
};

} // namespace

TEST(MCTPDDevice, setupCallsAssignEndpoint)
{
    MockMCTPDDevice dev(nullptr, "mctpi2c0", {0x20});
    EXPECT_CALL(dev, assignEndpoint(::testing::_)).Times(1);
    dev.setup([](const std::error_code&, const std::shared_ptr<MCTPEndpoint>&) {
    });
}
