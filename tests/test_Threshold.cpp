#include "../src/Thresholds.hpp"

#include <nlohmann/json.hpp>

#include <fstream>

#include <gtest/gtest.h>

TEST(Threshold, getInterface)
{
    EXPECT_EQ(thresholds::getInterface(thresholds::Level::WARNING),
              "xyz.openbmc_project.Sensor.Threshold.Warning");
    EXPECT_EQ(thresholds::getInterface(thresholds::Level::ERROR), "");
}

TEST(Threshold, parseThresholdsFromAttr)
{
    std::vector<thresholds::Threshold> ts;

    std::string path =
        "temp1_input"; // type == "temp", nr == 1, item == "input"

    // attrPath = temp1_min
    // attrPath = temp1_max
    // attrPath = temp1_lcrit
    // attrPath = temp1_crit

    std::ofstream f0("temp1_lcrit");
    f0 << "0\n";
    f0.close();
    std::ofstream f1("temp1_min");
    f1 << "1\n";
    f1.close();
    std::ofstream f2("temp1_max");
    f2 << "2\n";
    f2.close();
    std::ofstream f3("temp1_crit");
    f3 << "3\n";
    f3.close();

    parseThresholdsFromAttr(ts, path, 1.0, 0.0);

    std::remove("temp1_lcrit");
    std::remove("temp1_min");
    std::remove("temp1_max");
    std::remove("temp1_crit");

    ASSERT_EQ(ts.size(), 4);

    EXPECT_EQ(ts.at(2).level, thresholds::Level::CRITICAL);
    EXPECT_EQ(ts.at(2).direction, thresholds::Direction::LOW);
    EXPECT_FLOAT_EQ(ts.at(2).value, 0);

    EXPECT_EQ(ts.at(0).level, thresholds::Level::WARNING);
    EXPECT_EQ(ts.at(0).direction, thresholds::Direction::LOW);
    EXPECT_FLOAT_EQ(ts.at(0).value, 1);

    EXPECT_EQ(ts.at(1).level, thresholds::Level::WARNING);
    EXPECT_EQ(ts.at(1).direction, thresholds::Direction::HIGH);
    EXPECT_FLOAT_EQ(ts.at(1).value, 2);

    EXPECT_EQ(ts.at(3).level, thresholds::Level::CRITICAL);
    EXPECT_EQ(ts.at(3).direction, thresholds::Direction::HIGH);
    EXPECT_FLOAT_EQ(ts.at(3).value, 3);
}

TEST(Threshold, parseThresholdsFromConfig)
{
    std::vector<thresholds::Threshold> ts;

    SensorBaseConfigMap m1 = {
        {"Name", "Test"},
        {"Direction", "less than"},
        {"Severity", 0},
        {"Value", 1.0},
    };

    SensorBaseConfigMap m2 = {
        {"Name", "Test"},
        {"Direction", "greater than"},
        {"Severity", 0},
        {"Value", 1.0},
    };

    SensorBaseConfigMap m3 = {
        {"Name", "Test"},
        {"Direction", "less than"},
        {"Severity", 1},
        {"Value", 1.0},
    };

    SensorBaseConfigMap m4 = {
        {"Name", "Test"},
        {"Direction", "greater than"},
        {"Severity", 1},
        {"Value", 1.0},
    };

    SensorData sensorData = {
        {"Thresholds1", m1},
        {"Thresholds2", m2},
        {"Thresholds3", m3},
        {"Thresholds4", m4},
    };

    parseThresholdsFromConfig(sensorData, ts, nullptr, nullptr);

    ASSERT_EQ(ts.size(), 4);

    EXPECT_EQ(ts.at(0).level, thresholds::Level::WARNING);
    EXPECT_EQ(ts.at(0).direction, thresholds::Direction::LOW);
    EXPECT_FLOAT_EQ(ts.at(0).value, 1.0);

    EXPECT_EQ(ts.at(1).level, thresholds::Level::WARNING);
    EXPECT_EQ(ts.at(1).direction, thresholds::Direction::HIGH);
    EXPECT_FLOAT_EQ(ts.at(1).value, 1.0);

    EXPECT_EQ(ts.at(2).level, thresholds::Level::CRITICAL);
    EXPECT_EQ(ts.at(2).direction, thresholds::Direction::LOW);
    EXPECT_FLOAT_EQ(ts.at(2).value, 1.0);

    EXPECT_EQ(ts.at(3).level, thresholds::Level::CRITICAL);
    EXPECT_EQ(ts.at(3).direction, thresholds::Direction::HIGH);
    EXPECT_FLOAT_EQ(ts.at(3).value, 1.0);
}

TEST(Threshold, parseThresholdsFromConfigIndexFound)
{
    std::vector<thresholds::Threshold> ts;

    SensorBaseConfigMap m1 = {
        {"Name", "Test"}, {"Direction", "less than"},
        {"Severity", 0},  {"Value", 1.0},
        {"Index", 1},
    };

    SensorData sensorData = {
        {"Thresholds1", m1},
    };

    int sensorIndex = 1;
    parseThresholdsFromConfig(sensorData, ts, nullptr, &sensorIndex);

    ASSERT_EQ(ts.size(), 1);

    EXPECT_EQ(ts.at(0).level, thresholds::Level::WARNING);
    EXPECT_EQ(ts.at(0).direction, thresholds::Direction::LOW);
    EXPECT_FLOAT_EQ(ts.at(0).value, 1.0);
}

TEST(Threshold, parseThresholdsFromConfigIndexNotFound)
{
    std::vector<thresholds::Threshold> ts;

    SensorBaseConfigMap m1 = {
        {"Name", "Test"}, {"Direction", "less than"},
        {"Severity", 0},  {"Value", 1.0},
        {"Index", 1},
    };

    SensorData sensorData = {
        {"Thresholds1", m1},
    };

    int sensorIndex = 2;
    parseThresholdsFromConfig(sensorData, ts, nullptr, &sensorIndex);

    ASSERT_EQ(ts.size(), 0);
}

TEST(Threshold, parseThresholdsFromConfigLabelFound)
{
    std::vector<thresholds::Threshold> ts;

    SensorBaseConfigMap m1 = {
        {"Name", "Test"}, {"Direction", "less than"}, {"Severity", 0},
        {"Value", 1.0},   {"Label", "label1"},
    };

    SensorData sensorData = {
        {"Thresholds1", m1},
    };

    std::string matchLabel = "label1";
    parseThresholdsFromConfig(sensorData, ts, &matchLabel, nullptr);

    ASSERT_EQ(ts.size(), 1);

    EXPECT_EQ(ts.at(0).level, thresholds::Level::WARNING);
    EXPECT_EQ(ts.at(0).direction, thresholds::Direction::LOW);
    EXPECT_FLOAT_EQ(ts.at(0).value, 1.0);
}

TEST(Threshold, parseThresholdsFromConfigLabelNotFound)
{
    std::vector<thresholds::Threshold> ts;

    SensorBaseConfigMap m1 = {
        {"Name", "Test"}, {"Direction", "less than"}, {"Severity", 0},
        {"Value", 1.0},   {"Label", "label2"},
    };

    SensorData sensorData = {
        {"Thresholds1", m1},
    };

    std::string matchLabel = "label1";
    parseThresholdsFromConfig(sensorData, ts, &matchLabel, nullptr);

    ASSERT_EQ(ts.size(), 0);
}

TEST(Threshold, parseThresholdsFromConfigHysteresis)
{
    std::vector<thresholds::Threshold> ts;

    SensorBaseConfigMap m1 = {
        {"Name", "Test"}, {"Direction", "less than"}, {"Severity", 0},
        {"Value", 1.0},   {"Hysteresis", 3.0},
    };

    SensorData sensorData = {
        {"Thresholds1", m1},
    };

    parseThresholdsFromConfig(sensorData, ts, nullptr, nullptr);

    ASSERT_EQ(ts.size(), 1);

    EXPECT_EQ(ts.at(0).level, thresholds::Level::WARNING);
    EXPECT_EQ(ts.at(0).direction, thresholds::Direction::LOW);
    EXPECT_FLOAT_EQ(ts.at(0).value, 1.0);
    EXPECT_FLOAT_EQ(ts.at(0).hysteresis, 3.0);
}
