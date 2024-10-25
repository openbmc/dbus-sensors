#include "DeviceMgmt.hpp"
#include "Utils.hpp"

#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

namespace fs = std::filesystem;

class TestI2C : public testing::Test
{
  public:
    fs::path testDir;
    fs::path muxHomeDir;
    SensorData sensorCfg;
    std::string sensorIntf = std::string(configInterfacePrefix) + ".Sensor";

    TestI2C()
    {
        // Create test environment
        auto dir = std::to_array("./testDirXXXXXX");
        testDir = mkdtemp(dir.data());

        if (testDir.empty())
        {
            throw std::bad_alloc();
        }
        if (!fs::exists(muxDir))
        {
            fs::create_directory(muxDir);
        }
        uint16_t i = 0;
        while (fs::exists(std::string(muxDir) + "MUX" + std::to_string(i)))
        {
            i++;
        }
        muxHomeDir = std::string(muxDir) + "MUX" + std::to_string(i);
        fs::create_directory(muxHomeDir);

        std::ofstream bus1{testDir / "i2c-1"};
        std::ofstream bus2{testDir / "i2c-2"};
        std::ofstream bus3{testDir / "i2c-3"};
        std::ofstream bus4{testDir / "i2c-4"};

        fs::create_symlink(testDir / "i2c-1", muxHomeDir / "ch0");
        fs::create_symlink(testDir / "i2c-2", muxHomeDir / "ch1");
        fs::create_symlink(testDir / "i2c-3", muxHomeDir / "ch2");
        fs::create_symlink(testDir / "i2c-4", muxHomeDir / "ch3");

        SensorBaseConfigMap muxChIntf;
        muxChIntf["MuxName"] = "MUX" + std::to_string(i);
        muxChIntf["ChannelName"] = "ch0";
        sensorCfg[sensorIntf + ".MuxChannel"] = muxChIntf;
    }

    ~TestI2C() override
    {
        fs::remove_all(testDir);
        fs::remove_all(muxHomeDir);
    }
};

TEST_F(TestI2C, findMux_non_exist)
{
    std::string channelName;
    auto ret = I2CMux::findMux(std::string(configInterfacePrefix) + ".Empty",
                               sensorCfg, "testSensor", channelName);

    EXPECT_FALSE(ret);
    EXPECT_TRUE(channelName.empty());
}

TEST_F(TestI2C, findMux_missing_MuxName)
{
    std::string channelName;
    SensorBaseConfigMap missChIntf{{"ChannelName", "ch0"}};
    sensorCfg[sensorIntf + ".MuxChannel"] = missChIntf;
    auto ret = I2CMux::findMux(sensorIntf, sensorCfg, "testSensor",
                               channelName);

    EXPECT_FALSE(ret);
    EXPECT_TRUE(channelName.empty());
}

TEST_F(TestI2C, findMux_missing_ChannelName)
{
    std::string channelName;
    SensorBaseConfigMap missChIntf{{"MuxName", "MUX0"}};
    sensorCfg[sensorIntf + ".MuxChannel"] = missChIntf;
    auto ret = I2CMux::findMux(sensorIntf, sensorCfg, "testSensor",
                               channelName);

    EXPECT_FALSE(ret);
    EXPECT_TRUE(channelName.empty());
}

TEST_F(TestI2C, findMux_exists)
{
    std::string channelName;
    auto ret = I2CMux::findMux(sensorIntf, sensorCfg, "testSensor",
                               channelName);

    EXPECT_TRUE(ret);
    EXPECT_EQ(channelName, "ch0");
}

TEST_F(TestI2C, getLogicalBus_exists)
{
    std::string channelName;
    auto mux = I2CMux::findMux(sensorIntf, sensorCfg, "testSensor", channelName)
                   .value();
    auto bus = mux.getLogicalBus(channelName);

    EXPECT_TRUE(bus);
}

TEST_F(TestI2C, getBus_valid)
{
    std::string channelName;
    auto mux = I2CMux::findMux(sensorIntf, sensorCfg, "testSensor", channelName)
                   .value();
    auto bus = mux.getLogicalBus(channelName);

    EXPECT_EQ(bus.value().getBus(), 1);
}
