#include "Thresholds.hpp"
#include "Utils.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

TEST(ThresholdIndex, ZeroIndexPreservesDifferentFilteringSemantics)
{
    SensorBaseConfigMap config{{"Direction", std::string("less than")},
                               {"Severity", uint64_t{1}},
                               {"Value", 10.0}};
    const thresholds::Threshold threshold{thresholds::Level::CRITICAL,
                                          thresholds::Direction::LOW, 10.0};
    const std::string interface = "xyz.openbmc_project.Thresholds0";
    const int zero = 0;

    for (int configuredIndex : {-1, 0, 3})
    {
        if (configuredIndex >= 0)
        {
            config["Index"] = int64_t{configuredIndex};
        }
        SensorData sensorData{{interface, config}};
        std::vector<thresholds::Threshold> parsed;
        ASSERT_TRUE(thresholds::parseThresholdsFromConfig(sensorData, parsed,
                                                          nullptr, &zero));
        EXPECT_EQ(parsed.size(), configuredIndex == 0 ? 1U : 0U);
        EXPECT_TRUE(
            thresholds::matchesThresholdConfig(config, threshold, "", zero));
        EXPECT_TRUE(thresholds::matchesThresholdConfig(config, threshold, ""));

        parsed.clear();
        ASSERT_TRUE(thresholds::parseThresholdsFromConfig(sensorData, parsed));
        EXPECT_EQ(parsed.size(), 1U);
    }
}
