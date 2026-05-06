#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

#include <sdbusplus/asio/connection.hpp>

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace
{

// Minimal concrete Sensor. No D-Bus interfaces are created, so
// assertThresholds() becomes a no-op; we observe behaviour through
// Threshold::hitCount and the return value of thresholds::checkThresholds(),
// which is false only when a CRITICAL threshold is asserted in this pass.
class TestSensor : public Sensor
{
  public:
    TestSensor(std::vector<thresholds::Threshold>&& thresholdData,
               std::shared_ptr<sdbusplus::asio::connection>& conn) :
        Sensor("test_sensor", std::move(thresholdData), "/test/config",
               "TestSensor", false, false, 127.0, -128.0, conn)
    {}

    void checkThresholds() override
    {
        thresholds::checkThresholds(this);
    }
};

// Build a SensorData map containing one Thresholds configuration interface.
SensorData makeThresholdConfig(
    double value, const std::string& direction, uint64_t severity,
    bool withTolerance, uint64_t toleranceCount = 0, double hysteresis = 0.0)
{
    SensorBaseConfigMap cfg = {
        {"Value", value},
        {"Direction", direction},
        {"Severity", severity},
        {"Hysteresis", hysteresis},
    };
    if (withTolerance)
    {
        cfg["ToleranceCount"] = toleranceCount;
    }
    SensorData data;
    data["xyz.openbmc_project.Configuration.TestSensor.Thresholds0"] = cfg;
    return data;
}

class ThresholdToleranceTest : public ::testing::Test
{
  protected:
    // Feed `value` to the sensor and return whether a CRITICAL threshold
    // was asserted in this pass (checkThresholds returns false on assert).
    static bool feedAndAsserted(TestSensor& sensor, double value)
    {
        sensor.value = value;
        return !thresholds::checkThresholds(&sensor);
    }

    std::shared_ptr<sdbusplus::asio::connection> conn; // intentionally null
};

} // namespace

// ----------------------------------------------------------------------------
// Parsing
// ----------------------------------------------------------------------------

TEST_F(ThresholdToleranceTest, ParseToleranceCountFromConfig)
{
    std::vector<thresholds::Threshold> parsed;
    ASSERT_TRUE(thresholds::parseThresholdsFromConfig(
        makeThresholdConfig(80.0, "greater than", 1, true, 3), parsed));
    ASSERT_EQ(parsed.size(), 1U);
    EXPECT_EQ(parsed[0].level, thresholds::Level::CRITICAL);
    EXPECT_EQ(parsed[0].direction, thresholds::Direction::HIGH);
    EXPECT_DOUBLE_EQ(parsed[0].value, 80.0);
    EXPECT_EQ(parsed[0].toleranceCount, 3U);
    EXPECT_EQ(parsed[0].hitCount, 0U);
}

TEST_F(ThresholdToleranceTest, ParseMissingToleranceCountDefaultsToZero)
{
    std::vector<thresholds::Threshold> parsed;
    ASSERT_TRUE(thresholds::parseThresholdsFromConfig(
        makeThresholdConfig(80.0, "greater than", 1, false), parsed));
    ASSERT_EQ(parsed.size(), 1U);
    EXPECT_EQ(parsed[0].toleranceCount, 0U);
}

TEST_F(ThresholdToleranceTest, ConstructorDefaultsToleranceCountToZero)
{
    thresholds::Threshold t(thresholds::Level::WARNING,
                            thresholds::Direction::HIGH, 10.0);
    EXPECT_EQ(t.toleranceCount, 0U);
    EXPECT_EQ(t.hitCount, 0U);
    EXPECT_TRUE(t.writeable);
}

// ----------------------------------------------------------------------------
// checkThresholds() — HIGH direction
// ----------------------------------------------------------------------------

TEST_F(ThresholdToleranceTest, ZeroToleranceAssertsImmediately)
{
    std::vector<thresholds::Threshold> th{
        {thresholds::Level::CRITICAL, thresholds::Direction::HIGH, 80.0, 1.0,
         0}};
    TestSensor sensor(std::move(th), conn);

    EXPECT_TRUE(feedAndAsserted(sensor, 85.0));
    EXPECT_EQ(sensor.thresholds[0].hitCount, 0U);
}

TEST_F(ThresholdToleranceTest, HighAssertsOnlyAfterToleranceExceeded)
{
    std::vector<thresholds::Threshold> th{
        {thresholds::Level::CRITICAL, thresholds::Direction::HIGH, 80.0, 1.0,
         2}};
    TestSensor sensor(std::move(th), conn);
    auto& t = sensor.thresholds[0];

    // 1st and 2nd exceedances are tolerated
    EXPECT_FALSE(feedAndAsserted(sensor, 85.0));
    EXPECT_EQ(t.hitCount, 1U);
    EXPECT_FALSE(feedAndAsserted(sensor, 85.0));
    EXPECT_EQ(t.hitCount, 2U);

    // 3rd exceedance asserts; counter saturates at toleranceCount
    EXPECT_TRUE(feedAndAsserted(sensor, 85.0));
    EXPECT_EQ(t.hitCount, 2U);

    // Stays asserted while above threshold
    EXPECT_TRUE(feedAndAsserted(sensor, 90.0));
    EXPECT_EQ(t.hitCount, 2U);
}

TEST_F(ThresholdToleranceTest, HighReturnToNormalResetsCounter)
{
    std::vector<thresholds::Threshold> th{
        {thresholds::Level::CRITICAL, thresholds::Direction::HIGH, 80.0, 1.0,
         2}};
    TestSensor sensor(std::move(th), conn);
    auto& t = sensor.thresholds[0];

    EXPECT_FALSE(feedAndAsserted(sensor, 85.0));
    EXPECT_EQ(t.hitCount, 1U);

    // Drop clearly below (threshold - hysteresis) -> deassert + reset
    EXPECT_FALSE(feedAndAsserted(sensor, 70.0));
    EXPECT_EQ(t.hitCount, 0U);

    // Counting starts from scratch again
    EXPECT_FALSE(feedAndAsserted(sensor, 85.0));
    EXPECT_EQ(t.hitCount, 1U);
    EXPECT_FALSE(feedAndAsserted(sensor, 85.0));
    EXPECT_EQ(t.hitCount, 2U);
    EXPECT_TRUE(feedAndAsserted(sensor, 85.0));
}

TEST_F(ThresholdToleranceTest, HighHysteresisMidstateResetsCounter)
{
    std::vector<thresholds::Threshold> th{
        {thresholds::Level::CRITICAL, thresholds::Direction::HIGH, 80.0, 5.0,
         2}};
    TestSensor sensor(std::move(th), conn);
    auto& t = sensor.thresholds[0];

    EXPECT_FALSE(feedAndAsserted(sensor, 85.0));
    EXPECT_EQ(t.hitCount, 1U);

    // 77.0 is inside the hysteresis band (75.0, 80.0): no change, but reset
    EXPECT_FALSE(feedAndAsserted(sensor, 77.0));
    EXPECT_EQ(t.hitCount, 0U);
}

TEST_F(ThresholdToleranceTest, HighIntermittentSpikesNeverAssert)
{
    std::vector<thresholds::Threshold> th{
        {thresholds::Level::CRITICAL, thresholds::Direction::HIGH, 80.0, 1.0,
         2}};
    TestSensor sensor(std::move(th), conn);

    // Spike / normal / spike / normal ... never reaches 3 consecutive
    for (int i = 0; i < 10; ++i)
    {
        EXPECT_FALSE(feedAndAsserted(sensor, 85.0));
        EXPECT_FALSE(feedAndAsserted(sensor, 85.0));
        EXPECT_FALSE(feedAndAsserted(sensor, 70.0));
    }
    EXPECT_EQ(sensor.thresholds[0].hitCount, 0U);
}

// ----------------------------------------------------------------------------
// checkThresholds() — LOW direction (symmetric)
// ----------------------------------------------------------------------------

TEST_F(ThresholdToleranceTest, LowAssertsOnlyAfterToleranceExceeded)
{
    std::vector<thresholds::Threshold> th{
        {thresholds::Level::CRITICAL, thresholds::Direction::LOW, 10.0, 1.0,
         2}};
    TestSensor sensor(std::move(th), conn);
    auto& t = sensor.thresholds[0];

    EXPECT_FALSE(feedAndAsserted(sensor, 5.0));
    EXPECT_EQ(t.hitCount, 1U);
    EXPECT_FALSE(feedAndAsserted(sensor, 5.0));
    EXPECT_EQ(t.hitCount, 2U);
    EXPECT_TRUE(feedAndAsserted(sensor, 5.0));
    EXPECT_EQ(t.hitCount, 2U);
}

TEST_F(ThresholdToleranceTest, LowReturnToNormalResetsCounter)
{
    std::vector<thresholds::Threshold> th{
        {thresholds::Level::CRITICAL, thresholds::Direction::LOW, 10.0, 1.0,
         2}};
    TestSensor sensor(std::move(th), conn);
    auto& t = sensor.thresholds[0];

    EXPECT_FALSE(feedAndAsserted(sensor, 5.0));
    EXPECT_EQ(t.hitCount, 1U);

    // Rise clearly above (threshold + hysteresis) -> reset
    EXPECT_FALSE(feedAndAsserted(sensor, 20.0));
    EXPECT_EQ(t.hitCount, 0U);
}

TEST_F(ThresholdToleranceTest, LowHysteresisMidstateResetsCounter)
{
    std::vector<thresholds::Threshold> th{
        {thresholds::Level::CRITICAL, thresholds::Direction::LOW, 10.0, 5.0,
         2}};
    TestSensor sensor(std::move(th), conn);
    auto& t = sensor.thresholds[0];

    EXPECT_FALSE(feedAndAsserted(sensor, 5.0));
    EXPECT_EQ(t.hitCount, 1U);

    // 12.0 is inside the hysteresis band (10.0, 15.0)
    EXPECT_FALSE(feedAndAsserted(sensor, 12.0));
    EXPECT_EQ(t.hitCount, 0U);
}

// ----------------------------------------------------------------------------
// Independence between thresholds on the same sensor
// ----------------------------------------------------------------------------

TEST_F(ThresholdToleranceTest, CountersAreTrackedPerThreshold)
{
    std::vector<thresholds::Threshold> th{
        {thresholds::Level::WARNING, thresholds::Direction::HIGH, 70.0, 1.0, 1},
        {thresholds::Level::CRITICAL, thresholds::Direction::HIGH, 80.0, 1.0,
         3}};
    TestSensor sensor(std::move(th), conn);
    auto& warn = sensor.thresholds[0];
    auto& crit = sensor.thresholds[1];

    // 85 exceeds both; warning tolerates 1, critical tolerates 3
    EXPECT_FALSE(feedAndAsserted(sensor, 85.0)); // warn hit=1, crit hit=1
    EXPECT_EQ(warn.hitCount, 1U);
    EXPECT_EQ(crit.hitCount, 1U);

    EXPECT_FALSE(feedAndAsserted(sensor, 85.0)); // warn asserts, crit hit=2
    EXPECT_EQ(warn.hitCount, 1U);
    EXPECT_EQ(crit.hitCount, 2U);

    EXPECT_FALSE(feedAndAsserted(sensor, 85.0)); // crit hit=3
    EXPECT_EQ(crit.hitCount, 3U);

    EXPECT_TRUE(feedAndAsserted(sensor, 85.0)); // crit asserts
}
