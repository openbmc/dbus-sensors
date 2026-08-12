#include "Thresholds.hpp"
#include "Utils.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace
{

// EntityManager exposes each element of a configuration "Thresholds" array as
// its own D-Bus interface, suffixed with the position in the array.
const std::string thresholdIface =
    "xyz.openbmc_project.Configuration.NCT7802.Thresholds";

SensorBaseConfigMap labelledThreshold(const std::string& label, double value)
{
    return SensorBaseConfigMap{
        {"Direction", std::string("greater than")},
        {"Label", label},
        {"Name", std::string("upper critical")},
        {"Severity", uint64_t(1)},
        {"Value", value},
    };
}

SensorBaseConfigMap indexedThreshold(int64_t index, double value)
{
    return SensorBaseConfigMap{
        {"Direction", std::string("greater than")},
        {"Index", index},
        {"Name", std::string("upper critical")},
        {"Severity", uint64_t(1)},
        {"Value", value},
    };
}

// A multi-sensor chip (NCT7802 exposes temp1..temp3) configured the "Label"
// way: each threshold names the hwmon channel it belongs to.
SensorData labelledConfig()
{
    return SensorData{
        {thresholdIface + "0", labelledThreshold("temp1", 40.0)},
        {thresholdIface + "1", labelledThreshold("temp2", 50.0)},
        {thresholdIface + "2", labelledThreshold("temp3", 70.0)},
    };
}

// The same chip configured the "Index" way.
SensorData indexedConfig()
{
    return SensorData{
        {thresholdIface + "0", indexedThreshold(1, 40.0)},
        {thresholdIface + "1", indexedThreshold(2, 50.0)},
        {thresholdIface + "2", indexedThreshold(3, 70.0)},
    };
}

} // namespace

// Every threshold in these fixtures is an upper critical, so a channel is
// identified by the single value that reached it.
TEST(ParseThresholdsFromConfig, LabelledConfigRoutesToFirstChannel)
{
    std::vector<thresholds::Threshold> parsed;
    const std::string label = "temp1";
    const int index = 1;

    EXPECT_TRUE(
        parseThresholdsFromConfig(labelledConfig(), parsed, &label, &index));

    ASSERT_EQ(parsed.size(), 1U);
    EXPECT_DOUBLE_EQ(parsed[0].value, 40.0);
}

// openbmc/dbus-sensors#24. The "Label" and "Index" selectors in
// parseThresholdsFromConfig() are AND-ed: an entry carrying a Label but no
// Index is rejected by the Index check for every index except 1, so on a
// Label-configured multi-channel chip only temp1 can ever get its thresholds.
TEST(ParseThresholdsFromConfig, LabelledConfigRoutesToSecondChannel)
{
    std::vector<thresholds::Threshold> parsed;
    const std::string label = "temp2";
    const int index = 2;

    EXPECT_TRUE(
        parseThresholdsFromConfig(labelledConfig(), parsed, &label, &index));

    ASSERT_EQ(parsed.size(), 1U);
    EXPECT_DOUBLE_EQ(parsed[0].value, 50.0);
}

TEST(ParseThresholdsFromConfig, LabelledConfigRoutesToThirdChannel)
{
    std::vector<thresholds::Threshold> parsed;
    const std::string label = "temp3";
    const int index = 3;

    EXPECT_TRUE(
        parseThresholdsFromConfig(labelledConfig(), parsed, &label, &index));

    ASSERT_EQ(parsed.size(), 1U);
    EXPECT_DOUBLE_EQ(parsed[0].value, 70.0);
}

// Mirrors how HwmonTempMain.cpp calls in today: no label is requested, so
// "Label" is not consulted at all and every Index-less entry matches index 1.
// temp1 must not collect temp2's and temp3's thresholds on top of its own.
TEST(ParseThresholdsFromConfig, LabelledEntriesAreNotStackedOnOneChannel)
{
    std::vector<thresholds::Threshold> parsed;
    const int index = 1;

    EXPECT_TRUE(
        parseThresholdsFromConfig(labelledConfig(), parsed, nullptr, &index));

    EXPECT_LE(parsed.size(), 1U);
}

// A caller that passes a label must not lose Index-configured thresholds:
// entries that carry no Label at all should still be selected by Index.
TEST(ParseThresholdsFromConfig, IndexedConfigSurvivesALabelledLookup)
{
    std::vector<thresholds::Threshold> parsed;
    const std::string label = "temp2";
    const int index = 2;

    EXPECT_TRUE(
        parseThresholdsFromConfig(indexedConfig(), parsed, &label, &index));

    ASSERT_EQ(parsed.size(), 1U);
    EXPECT_DOUBLE_EQ(parsed[0].value, 50.0);
}

// Existing behaviour, guarded so a fix does not regress it.
TEST(ParseThresholdsFromConfig, IndexedConfigRoutesByIndexAlone)
{
    std::vector<thresholds::Threshold> parsed;
    const int index = 2;

    EXPECT_TRUE(
        parseThresholdsFromConfig(indexedConfig(), parsed, nullptr, &index));

    ASSERT_EQ(parsed.size(), 1U);
    EXPECT_DOUBLE_EQ(parsed[0].value, 50.0);
}

// Single-channel legacy configuration: neither selector is present, so the
// threshold belongs to index 1 and to nothing else.
TEST(ParseThresholdsFromConfig, UnselectedThresholdBelongsToFirstChannelOnly)
{
    const SensorData data{
        {thresholdIface + "0",
         SensorBaseConfigMap{
             {"Direction", std::string("greater than")},
             {"Name", std::string("upper critical")},
             {"Severity", uint64_t(1)},
             {"Value", 40.0},
         }}};

    std::vector<thresholds::Threshold> first;
    const int firstIndex = 1;
    EXPECT_TRUE(parseThresholdsFromConfig(data, first, nullptr, &firstIndex));
    EXPECT_EQ(first.size(), 1U);

    std::vector<thresholds::Threshold> second;
    const int secondIndex = 2;
    EXPECT_TRUE(parseThresholdsFromConfig(data, second, nullptr, &secondIndex));
    EXPECT_TRUE(second.empty());
}
