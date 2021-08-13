
#include <SensorPaths.hpp>

#include <gtest/gtest.h>

using sensor_paths::escapePathForDbus;

TEST(SensorPaths, TestSensorPaths)
{
    EXPECT_EQ(escapePathForDbus("abc"), "abc");
    EXPECT_EQ(escapePathForDbus("abc"), "abc");
    EXPECT_EQ(escapePathForDbus("def"), "def");
    EXPECT_EQ(escapePathForDbus("-"), "_2d");
    EXPECT_EQ(escapePathForDbus(" "), "_20");
    EXPECT_EQ(escapePathForDbus("/"), "_2f");
    EXPECT_EQ(escapePathForDbus("ab_cd"), "ab_cd");
    EXPECT_EQ(escapePathForDbus("_ab_cd"), "_5fab_5fcd");
    EXPECT_EQ(escapePathForDbus("ab-c_d"), "_61b_2dc_5fd");
    EXPECT_EQ(escapePathForDbus(""), "");
}
