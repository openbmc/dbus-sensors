#include "MctpMockTestBase.hpp"
#include "Thresholds.hpp"
#include "sensor.hpp"

#include <chrono>
#include <cstdint>
#include <string>

#include <gtest/gtest.h>

namespace
{

constexpr const char* baseInterface =
    "xyz.openbmc_project.Configuration.TestSensor";
constexpr const char* thresholdInterface0 =
    "xyz.openbmc_project.Configuration.TestSensor.Thresholds0";
constexpr const char* thresholdInterface1 =
    "xyz.openbmc_project.Configuration.TestSensor.Thresholds1";
constexpr const char* thresholdPath1 =
    "/xyz/openbmc_project/configuration/test_sensor/threshold1";

class PersistThresholdTest : public DbusMockTestBase
{};

TEST_F(PersistThresholdTest, UpdatesOnlyMatchingIndexedThreshold)
{
    if (testing::Test::IsSkipped())
    {
        return;
    }

    bus()->request_name(entityManagerName);

    bool updated1 = false;
    bool updated2 = false;
    auto interface1 =
        objects().add_interface(thresholdPath1, thresholdInterface0);
    interface1->register_property("Label", std::string("Cpu Temp"));
    interface1->register_property("Severity", uint64_t(0));
    interface1->register_property("Direction", std::string("greater than"));
    interface1->register_property("Index", int32_t(1));
    interface1->register_property(
        "Value", 70.0,
        [&updated1](const double& request, double& oldValue) {
            oldValue = request;
            updated1 = true;
            return 1;
        });
    ASSERT_TRUE(interface1->initialize());

    auto interface2 =
        objects().add_interface(thresholdPath1, thresholdInterface1);
    interface2->register_property("Label", std::string("Cpu Temp"));
    interface2->register_property("Severity", uint64_t(0));
    interface2->register_property("Direction", std::string("greater than"));
    interface2->register_property("Index", int32_t(2));
    interface2->register_property(
        "Value", 80.0,
        [&updated2](const double& request, double& oldValue) {
            oldValue = request;
            updated2 = true;
            return 1;
        });
    ASSERT_TRUE(interface2->initialize());

    thresholds::Threshold threshold(thresholds::Level::WARNING,
                                    thresholds::Direction::HIGH, 55.0);
    thresholds::persistThreshold(
        thresholdPath1, baseInterface, threshold, bus(), 2, "Cpu Temp", 1);

    ASSERT_TRUE(pumpIoUntil([&updated1] { return updated1; },
                            std::chrono::seconds{5}));
    EXPECT_TRUE(updated1);
    EXPECT_FALSE(updated2);
    EXPECT_DOUBLE_EQ(getProperty<double>(thresholdPath1, thresholdInterface0,
                                         "Value"),
                     55.0);
    EXPECT_DOUBLE_EQ(getProperty<double>(thresholdPath1, thresholdInterface1,
                                         "Value"),
                     80.0);
}

} // namespace