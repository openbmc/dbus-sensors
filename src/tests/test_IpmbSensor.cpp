#include "ipmb/IpmbSensor.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

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

TEST(IPMBSensor, NineBitValidPositive)
{
    std::vector<uint8_t> data;
    data.push_back(0x2a);
    data.push_back(0x00);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_TRUE(IpmbSensor::processReading(ReadingFormat::nineBit, 0, data,
                                           responseValue, errCount));
    EXPECT_EQ(responseValue, 42.0);
    EXPECT_EQ(errCount, 0U);
}

TEST(IPMBSensor, NineBitValidNegative)
{
    std::vector<uint8_t> data;
    data.push_back(0x9c);
    data.push_back(0x01);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_TRUE(IpmbSensor::processReading(ReadingFormat::nineBit, 0, data,
                                           responseValue, errCount));
    EXPECT_EQ(responseValue, -100.0);
    EXPECT_EQ(errCount, 0U);
}

TEST(IPMBSensor, NineBitMin)
{
    std::vector<uint8_t> data;
    data.push_back(0x01);
    data.push_back(0x01);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_TRUE(IpmbSensor::processReading(ReadingFormat::nineBit, 0, data,
                                           responseValue, errCount));
    EXPECT_EQ(responseValue, -255.0);
    EXPECT_EQ(errCount, 0U);
}

// The Altra Family SoC BMC Interface Specification says the maximum 9-bit value
// is 256, but that can't be represented in 9 bits, so test the max as 255.
TEST(IPMBSensor, NineBitMax)
{
    std::vector<uint8_t> data;
    data.push_back(0xff);
    data.push_back(0x00);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_TRUE(IpmbSensor::processReading(ReadingFormat::nineBit, 0, data,
                                           responseValue, errCount));
    EXPECT_EQ(responseValue, 255.0);
    EXPECT_EQ(errCount, 0U);
}

TEST(IPMBSensor, NineBitTooShort)
{
    std::vector<uint8_t> data;
    data.push_back(0x00);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_FALSE(IpmbSensor::processReading(ReadingFormat::nineBit, 0, data,
                                            responseValue, errCount));
}

TEST(IPMBSensor, NineBitTooLong)
{
    std::vector<uint8_t> data;
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(0x00);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_FALSE(IpmbSensor::processReading(ReadingFormat::nineBit, 0, data,
                                            responseValue, errCount));
}

TEST(IPMBSensor, NineBitInvalid)
{
    std::vector<uint8_t> data;
    data.push_back(0xff);
    data.push_back(0xff);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_FALSE(IpmbSensor::processReading(ReadingFormat::nineBit, 0, data,
                                            responseValue, errCount));
}

TEST(IPMBSensor, TenBitValid1)
{
    std::vector<uint8_t> data;
    data.push_back(0x08);
    data.push_back(0x00);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_TRUE(IpmbSensor::processReading(ReadingFormat::tenBit, 0, data,
                                           responseValue, errCount));
    EXPECT_EQ(responseValue, 8.0);
    EXPECT_EQ(errCount, 0U);
}

TEST(IPMBSensor, TenBitValid2)
{
    std::vector<uint8_t> data;
    data.push_back(0x30);
    data.push_back(0x02);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_TRUE(IpmbSensor::processReading(ReadingFormat::tenBit, 0, data,
                                           responseValue, errCount));

    EXPECT_EQ(responseValue, 560.0);
    EXPECT_EQ(errCount, 0U);
}

TEST(IPMBSensor, TenBitMin)
{
    std::vector<uint8_t> data;
    data.push_back(0x00);
    data.push_back(0x00);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_TRUE(IpmbSensor::processReading(ReadingFormat::tenBit, 0, data,
                                           responseValue, errCount));

    EXPECT_EQ(responseValue, 0.0);
    EXPECT_EQ(errCount, 0U);
}

TEST(IPMBSensor, TenBitValidMax)
{
    std::vector<uint8_t> data;
    data.push_back(0xff);
    data.push_back(0x03);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_TRUE(IpmbSensor::processReading(ReadingFormat::tenBit, 0, data,
                                           responseValue, errCount));

    EXPECT_EQ(responseValue, 1023.0);
    EXPECT_EQ(errCount, 0U);
}

TEST(IPMBSensor, TenBitTooShort)
{
    std::vector<uint8_t> data;
    data.push_back(0xff);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_FALSE(IpmbSensor::processReading(ReadingFormat::tenBit, 0, data,
                                            responseValue, errCount));
}

TEST(IPMBSensor, TenBitTooLong)
{
    std::vector<uint8_t> data;
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(0x00);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_FALSE(IpmbSensor::processReading(ReadingFormat::tenBit, 0, data,
                                            responseValue, errCount));
}

TEST(IPMBSensor, TenBitInvalid)
{
    std::vector<uint8_t> data;
    data.push_back(0xff);
    data.push_back(0xff);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_FALSE(IpmbSensor::processReading(ReadingFormat::tenBit, 0, data,
                                            responseValue, errCount));
}

TEST(IPMBSensor, FifteenBitValid1)
{
    std::vector<uint8_t> data;
    data.push_back(0xda);
    data.push_back(0x02);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_TRUE(IpmbSensor::processReading(ReadingFormat::fifteenBit, 0, data,
                                           responseValue, errCount));
    EXPECT_EQ(responseValue, 0.730);
    EXPECT_EQ(errCount, 0U);
}

TEST(IPMBSensor, FifteenBitMin)
{
    std::vector<uint8_t> data;
    data.push_back(0x00);
    data.push_back(0x00);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_TRUE(IpmbSensor::processReading(ReadingFormat::fifteenBit, 0, data,
                                           responseValue, errCount));
    EXPECT_EQ(responseValue, 0.0);
    EXPECT_EQ(errCount, 0U);
}

TEST(IPMBSensor, FifteenBitMax)
{
    std::vector<uint8_t> data;
    data.push_back(0xff);
    data.push_back(0x7f);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_TRUE(IpmbSensor::processReading(ReadingFormat::fifteenBit, 0, data,
                                           responseValue, errCount));
    EXPECT_EQ(responseValue, 32.767);
    EXPECT_EQ(errCount, 0U);
}

TEST(IPMBSensor, FifteenBitTooShort)
{
    std::vector<uint8_t> data;
    data.push_back(0xff);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_FALSE(IpmbSensor::processReading(ReadingFormat::fifteenBit, 0, data,
                                            responseValue, errCount));
}

TEST(IPMBSensor, FifteenBitTooLong)
{
    std::vector<uint8_t> data;
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(0x00);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_FALSE(IpmbSensor::processReading(ReadingFormat::fifteenBit, 0, data,
                                            responseValue, errCount));
}

TEST(IPMBSensor, FifteenBitInvalid)
{
    std::vector<uint8_t> data;
    data.push_back(0xff);
    data.push_back(0xff);

    double responseValue = 0.0;
    size_t errCount = 0;
    EXPECT_FALSE(IpmbSensor::processReading(ReadingFormat::fifteenBit, 0, data,
                                            responseValue, errCount));
}

} // namespace
