#include "DeviceMgmt.hpp"
#include "Utils.hpp"

#include <array>
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

namespace fs = std::filesystem;

class TestI2C : public testing::Test
{
  public:
    fs::path testDir;
    fs::path muxDevDir;
    fs::path muxChDir;
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
        muxDevDir = std::string(testDir) + defaultMuxDir;
        if (!fs::exists(muxDevDir))
        {
            fs::create_directories(muxDevDir);
        }
        muxChDir = muxDevDir / "MUX";
        fs::create_directory(muxChDir);

        std::ofstream bus1{muxDevDir / "i2c-1"};
        std::ofstream bus2{muxDevDir / "i2c-2"};
        std::ofstream bus3{muxDevDir / "i2c-3"};
        std::ofstream bus4{muxDevDir / "i2c-4"};

        fs::create_symlink(muxDevDir / "i2c-1", muxChDir / "ch0");
        fs::create_symlink(muxDevDir / "i2c-2", muxChDir / "ch1");
        fs::create_symlink(muxDevDir / "i2c-3", muxChDir / "ch2");
        fs::create_symlink(muxDevDir / "i2c-4", muxChDir / "ch3");

        SensorBaseConfigMap muxChIntf;
        muxChIntf["MuxName"] = "MUX";
        muxChIntf["ChannelName"] = "ch0";
        sensorCfg[sensorIntf + ".MuxChannel"] = muxChIntf;
    }

    ~TestI2C() override
    {
        fs::remove_all(testDir);
    }

    TestI2C(const TestI2C&) = delete;
    TestI2C(TestI2C&&) = delete;
    TestI2C& operator=(const TestI2C&) = delete;
    TestI2C& operator=(TestI2C&&) = delete;
};

TEST_F(TestI2C, findMux_non_exist)
{
    std::string channelName;
    auto ret = I2CMux::findMux(std::string(configInterfacePrefix) + ".Empty",
                               sensorCfg, "testSensor", channelName, muxDevDir);

    EXPECT_FALSE(ret);
    EXPECT_TRUE(channelName.empty());
}

TEST_F(TestI2C, findMux_missing_MuxName)
{
    std::string channelName;
    SensorBaseConfigMap missChIntf{{"ChannelName", "ch0"}};
    sensorCfg[sensorIntf + ".MuxChannel"] = missChIntf;
    auto ret = I2CMux::findMux(sensorIntf, sensorCfg, "testSensor", channelName,
                               muxDevDir);

    EXPECT_FALSE(ret);
    EXPECT_TRUE(channelName.empty());
}

TEST_F(TestI2C, findMux_missing_ChannelName)
{
    std::string channelName;
    SensorBaseConfigMap missChIntf{{"MuxName", "MUX"}};
    sensorCfg[sensorIntf + ".MuxChannel"] = missChIntf;
    auto ret = I2CMux::findMux(sensorIntf, sensorCfg, "testSensor", channelName,
                               muxDevDir);

    EXPECT_FALSE(ret);
    EXPECT_TRUE(channelName.empty());
}

TEST_F(TestI2C, findMux_exists)
{
    std::string channelName;
    auto ret = I2CMux::findMux(sensorIntf, sensorCfg, "testSensor", channelName,
                               muxDevDir);

    EXPECT_TRUE(ret);
    EXPECT_EQ(channelName, "ch0");
}

TEST_F(TestI2C, getLogicalBus_noSymlink)
{
    std::string channelName;
    if (auto mux = I2CMux::findMux(sensorIntf, sensorCfg, "testSensor",
                                   channelName, muxDevDir); mux.has_value())
    {
        fs::remove(muxChDir / "ch0");
        auto bus = mux->getLogicalBus(channelName);

        EXPECT_FALSE(bus);
    }
}

TEST_F(TestI2C, getLogicalBus_exists)
{
    std::string channelName;
    if (auto mux = I2CMux::findMux(sensorIntf, sensorCfg, "testSensor",
        channelName, muxDevDir); mux.has_value())
    {
        auto bus = mux->getLogicalBus(channelName);

        EXPECT_TRUE(bus);
    }
}

TEST_F(TestI2C, getBus_valid)
{
    std::string channelName;
    if (auto mux = I2CMux::findMux(sensorIntf, sensorCfg, "testSensor",
        channelName, muxDevDir); mux.has_value())
    {
        if (auto bus = mux->getLogicalBus(channelName); bus.has_value())
        {
            EXPECT_EQ(bus->getBus(), 1);
        }
    }
}
