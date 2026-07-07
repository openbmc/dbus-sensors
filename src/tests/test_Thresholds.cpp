#include "Thresholds.hpp"
#include "sensor.hpp"

#include <gtest/gtest.h>

class ThresholdTestSensor : public Sensor
{
  public:
    ThresholdTestSensor() :
        Sensor("test", {}, "", "Test", false, false, 100.0, 0.0, connection)
    {}

    void checkThresholds() override {}

  private:
    inline static std::shared_ptr<sdbusplus::asio::connection> connection;
};

TEST(Thresholds, FirstEvaluationAfterReconstructionIsForced)
{
    ThresholdTestSensor sensor;

    EXPECT_EQ(thresholds::thresholdEmit(&sensor),
          thresholds::ThresholdEmit::forced);

    sensor.hadValidValue = true;

    EXPECT_EQ(thresholds::thresholdEmit(&sensor),
          thresholds::ThresholdEmit::onChange);
}

TEST(Thresholds, ForcedEmissionBypassesUnchangedAlarmProperty)
{
    EXPECT_TRUE(thresholds::shouldEmitThresholdSignal(
        false, thresholds::ThresholdEmit::forced));
    EXPECT_FALSE(thresholds::shouldEmitThresholdSignal(
        false, thresholds::ThresholdEmit::onChange));
    EXPECT_TRUE(thresholds::shouldEmitThresholdSignal(
        true, thresholds::ThresholdEmit::onChange));
}