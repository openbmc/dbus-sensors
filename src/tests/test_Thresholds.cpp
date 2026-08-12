#include "Thresholds.hpp"

#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

class TestThresholds : public testing::Test
{
  protected:
    std::string testDir;

    void SetUp() override
    {
        auto dir = std::to_array("./testThresholdsXXXXXX");
        char* createdDir = mkdtemp(dir.data());
        ASSERT_NE(createdDir, nullptr);
        testDir = createdDir;
    }

    void TearDown() override
    {
        std::filesystem::remove_all(testDir);
    }
};

TEST_F(TestThresholds, labelProbeAppliesScaleFactorAndOverridesConfiguredValue)
{
    const std::filesystem::path sensorPath =
        std::filesystem::path(testDir) / "iout1_input";
    const std::filesystem::path thresholdPath =
        std::filesystem::path(testDir) / "iout1_max";
    {
        std::ofstream thresholdFile(thresholdPath);
        thresholdFile << "2500\n";
    }

    SensorBaseConfigMap thresholdConfig{
        {"Direction", "greater than"},
        {"ScaleFactor", 1000.0},
        {"Severity", uint8_t{0}},
        {"ThresholdLabel", "iout_oc_warn_limit"},
        {"Thresholds", std::vector<std::string>{}},
        {"Value", 10.0},
    };
    SensorData sensorData{
        {"xyz.openbmc_project.Configuration.PSU.Thresholds0",
         std::move(thresholdConfig)},
    };
    std::vector<thresholds::Threshold> parsedThresholds;
    const std::string sensorPathString = sensorPath.string();

    ASSERT_TRUE(thresholds::parseThresholdsFromConfigWithLabelProbe(
        sensorData, parsedThresholds, nullptr, nullptr, &sensorPathString));

    ASSERT_EQ(parsedThresholds.size(), 1U);
    EXPECT_DOUBLE_EQ(parsedThresholds.front().value, 2.5);
}
