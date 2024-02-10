#include "IpmbSensor.hpp"

#include <gtest/gtest.h>

namespace
{

TEST(IPMBSensor, Byte0)
{
    std::vector<uint8_t> data;
    data.push_back(42);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_TRUE(IpmbSensor::processReading(ReadingFormat::byte0, 0, data,
                                           responseValue, errCount));
    EXPECT_EQ(responseValue, 42.0);
}
} // namespace
