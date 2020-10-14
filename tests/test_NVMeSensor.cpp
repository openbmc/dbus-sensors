#include <i2c.h>

#include <NVMeSensor.hpp>
#include <dbus/connection.hpp>
#include <nlohmann/json.hpp>
#include <smbus.hpp>

#include <fstream>

#include "gtest/gtest.h"

TEST(NVMeSensor, TestNVMeSensor)
{}

int main(int argc, char** argv)
{

    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
