#include "Thresholds.hpp"
#include "Utils.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

TEST(ThresholdIndex, ZeroIndexIsNotTheUnfilteredValue)
{
    SensorBaseConfigMap config{{"Direction", std::string("less than")},
                               {"Severity", uint64_t{1}},
                               {"Value", 10.0}};
    const thresholds::Threshold threshold{thresholds::Level::CRITICAL,
                                          thresholds::Direction::LOW, 10.0};
    const std::string interface = "xyz.openbmc_project.Thresholds0";
    const int zero = 0;

    const std::optional<int> indexZero{zero};

    SensorData sensorData{{interface, config}};
    std::vector<thresholds::Threshold> parsed;
    ASSERT_TRUE(thresholds::parseThresholdsFromConfig(sensorData, parsed,
                                                      nullptr, &zero));
    EXPECT_TRUE(parsed.empty());
    EXPECT_FALSE(
        thresholds::matchesThresholdConfig(config, threshold, "", indexZero));
    EXPECT_TRUE(thresholds::matchesThresholdConfig(config, threshold, ""));

    config["Index"] = int64_t{0};
    sensorData[interface] = config;
    parsed.clear();
    ASSERT_TRUE(thresholds::parseThresholdsFromConfig(sensorData, parsed,
                                                      nullptr, &zero));
    ASSERT_EQ(parsed.size(), 1U);
    EXPECT_TRUE(
        thresholds::matchesThresholdConfig(config, threshold, "", indexZero));

    config["Index"] = int64_t{3};
    sensorData[interface] = config;
    parsed.clear();
    ASSERT_TRUE(thresholds::parseThresholdsFromConfig(sensorData, parsed,
                                                      nullptr, &zero));
    EXPECT_TRUE(parsed.empty());
    EXPECT_FALSE(
        thresholds::matchesThresholdConfig(config, threshold, "", indexZero));
    EXPECT_TRUE(thresholds::matchesThresholdConfig(config, threshold, ""));

    parsed.clear();
    ASSERT_TRUE(thresholds::parseThresholdsFromConfig(sensorData, parsed));
    EXPECT_EQ(parsed.size(), 1U);
}

TEST(ThresholdIndex, MissingIndexMatchesOnlyFirstChannel)
{
    const SensorBaseConfigMap config{{"Direction", std::string("less than")},
                                     {"Severity", uint64_t{1}},
                                     {"Value", 10.0}};
    const thresholds::Threshold threshold{thresholds::Level::CRITICAL,
                                          thresholds::Direction::LOW, 10.0};
    const int firstChannelIndex = 1;
    const int secondChannelIndex = 2;
    const std::optional<int> firstChannel{firstChannelIndex};
    const std::optional<int> secondChannel{secondChannelIndex};
    const std::string interface = "xyz.openbmc_project.Thresholds0";
    SensorData sensorData{{interface, config}};
    std::vector<thresholds::Threshold> parsed;

    ASSERT_TRUE(thresholds::parseThresholdsFromConfig(
        sensorData, parsed, nullptr, &firstChannelIndex));
    ASSERT_EQ(parsed.size(), 1U);
    EXPECT_TRUE(thresholds::matchesThresholdConfig(config, threshold, "",
                                                   firstChannel));
    EXPECT_FALSE(thresholds::matchesThresholdConfig(config, threshold, "",
                                                    secondChannel));
    parsed.clear();
    ASSERT_TRUE(thresholds::parseThresholdsFromConfig(
        sensorData, parsed, nullptr, &secondChannelIndex));
    EXPECT_TRUE(parsed.empty());
}
