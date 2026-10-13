#include "Thresholds.hpp"
#include "Utils.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace
{
SensorBaseConfigMap criticalLowConfig(double value)
{
    return {{"Direction", std::string("less than")},
            {"Severity", uint64_t{1}},
            {"Value", value}};
}

const thresholds::Threshold criticalLow{thresholds::Level::CRITICAL,
                                        thresholds::Direction::LOW, 50.0};
} // namespace

TEST(ThresholdConfig, RuntimeUpdateMatchesOnlyRequestedChannel)
{
    std::array<SensorBaseConfigMap, 3> configs = {
        criticalLowConfig(10.0), criticalLowConfig(20.0),
        criticalLowConfig(30.0)};
    configs[1]["Index"] = uint64_t{2};
    configs[2]["Index"] = uint64_t{3};

    for (int requestedIndex = 1; requestedIndex <= 3; ++requestedIndex)
    {
        for (int channelIndex = 1; channelIndex <= 3; ++channelIndex)
        {
            EXPECT_EQ(thresholds::matchesThresholdConfig(
                          configs[channelIndex - 1], criticalLow, "",
                          requestedIndex),
                      channelIndex == requestedIndex)
                << "requested channel " << requestedIndex << ", config channel "
                << channelIndex;
        }
    }
}

TEST(ThresholdConfig, MissingIndexMatchesOnlyFirstChannel)
{
    const auto config = criticalLowConfig(10.0);
    EXPECT_TRUE(thresholds::matchesThresholdConfig(config, criticalLow, "", 1));
    EXPECT_FALSE(
        thresholds::matchesThresholdConfig(config, criticalLow, "", 2));
    EXPECT_FALSE(
        thresholds::matchesThresholdConfig(config, criticalLow, "", 3));
}

TEST(ThresholdConfig, ExplicitFirstChannelIndexMatches)
{
    auto config = criticalLowConfig(10.0);
    config["Index"] = uint64_t{1};
    EXPECT_TRUE(thresholds::matchesThresholdConfig(config, criticalLow, "", 1));
    EXPECT_FALSE(
        thresholds::matchesThresholdConfig(config, criticalLow, "", 2));
}

TEST(ThresholdConfig, SignedIndexMatches)
{
    auto config = criticalLowConfig(20.0);
    config["Index"] = int64_t{2};
    EXPECT_TRUE(thresholds::matchesThresholdConfig(config, criticalLow, "", 2));
    EXPECT_FALSE(
        thresholds::matchesThresholdConfig(config, criticalLow, "", 1));
}

TEST(ThresholdConfig, DefaultIndexPreservesUnfilteredMatching)
{
    auto config = criticalLowConfig(10.0);
    EXPECT_TRUE(thresholds::matchesThresholdConfig(config, criticalLow, ""));
    config["Index"] = uint64_t{3};
    EXPECT_TRUE(thresholds::matchesThresholdConfig(config, criticalLow, ""));
}

TEST(ThresholdConfig, LabelMatchingIsPreserved)
{
    auto config = criticalLowConfig(10.0);
    EXPECT_FALSE(
        thresholds::matchesThresholdConfig(config, criticalLow, "CPU"));
    config["Label"] = std::string("CPU");
    EXPECT_TRUE(thresholds::matchesThresholdConfig(config, criticalLow, "CPU"));
    EXPECT_FALSE(
        thresholds::matchesThresholdConfig(config, criticalLow, "DIMM"));
}

TEST(ThresholdConfig, LabelAndIndexMustBothMatch)
{
    auto config = criticalLowConfig(20.0);
    config["Label"] = std::string("CPU");
    config["Index"] = uint64_t{2};
    EXPECT_TRUE(
        thresholds::matchesThresholdConfig(config, criticalLow, "CPU", 2));
    EXPECT_FALSE(
        thresholds::matchesThresholdConfig(config, criticalLow, "CPU", 1));
    EXPECT_FALSE(
        thresholds::matchesThresholdConfig(config, criticalLow, "DIMM", 2));
}

TEST(ThresholdConfig, SeverityAndDirectionMustMatch)
{
    auto config = criticalLowConfig(20.0);
    config["Index"] = uint64_t{2};
    EXPECT_TRUE(thresholds::matchesThresholdConfig(config, criticalLow, "", 2));
    config["Severity"] = uint64_t{0};
    EXPECT_FALSE(
        thresholds::matchesThresholdConfig(config, criticalLow, "", 2));
    config["Severity"] = uint64_t{1};
    config["Direction"] = std::string("greater than");
    EXPECT_FALSE(
        thresholds::matchesThresholdConfig(config, criticalLow, "", 2));
}

TEST(ThresholdConfig, MissingRequiredPropertiesDoNotMatch)
{
    for (const std::string property : {"Direction", "Severity", "Value"})
    {
        auto config = criticalLowConfig(10.0);
        config.erase(property);
        EXPECT_FALSE(
            thresholds::matchesThresholdConfig(config, criticalLow, ""))
            << "missing property " << property;
    }
}

TEST(ThresholdConfig, RuntimeSelectionAgreesWithStartupForEveryChannel)
{
    SensorData sensorData;
    for (int channelIndex = 1; channelIndex <= 3; ++channelIndex)
    {
        auto config = criticalLowConfig(channelIndex * 10.0);
        if (channelIndex != 1)
        {
            config["Index"] = uint64_t(channelIndex);
        }
        sensorData["xyz.openbmc_project.Configuration.TMP432.Thresholds" +
                   std::to_string(channelIndex - 1)] = config;
    }

    for (int requestedIndex = 1; requestedIndex <= 3; ++requestedIndex)
    {
        std::vector<thresholds::Threshold> parsed;
        ASSERT_TRUE(thresholds::parseThresholdsFromConfig(
            sensorData, parsed, nullptr, &requestedIndex));
        ASSERT_EQ(parsed.size(), 1U);
        EXPECT_DOUBLE_EQ(parsed.front().value, requestedIndex * 10.0);

        size_t matches = 0;
        for (const auto& [interface, config] : sensorData)
        {
            if (thresholds::matchesThresholdConfig(config, criticalLow, "",
                                                   requestedIndex))
            {
                ++matches;
                std::vector<thresholds::Threshold> selected;
                ASSERT_TRUE(thresholds::parseThresholdsFromConfig(
                    SensorData{{interface, config}}, selected));
                ASSERT_EQ(selected.size(), 1U);
                EXPECT_EQ(selected.front(), parsed.front());
            }
        }
        EXPECT_EQ(matches, 1U);
    }
}
