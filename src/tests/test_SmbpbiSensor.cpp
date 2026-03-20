/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "smbpbi/SmbpbiSensor.hpp"

#include <array>
#include <cstdint>

#include <gtest/gtest.h>

// Test standard 8-byte sensors (e.g. power, energy, voltage).
TEST(SmbpbiSensorTest, CheckInvalidReading8Bytes)
{
    // data1: all 8 bytes 0xFF (invalid reading)
    std::array<uint8_t, 8> data1 = {0xFF, 0xFF, 0xFF, 0xFF,
                                    0xFF, 0xFF, 0xFF, 0xFF};

    // data2: first 4 bytes 0xFF, rest valid (valid reading for 8-byte check)
    std::array<uint8_t, 8> data2 = {0xFF, 0xFF, 0xFF, 0xFF,
                                    0x01, 0x02, 0x04, 0x04};

    // data3: all bytes valid (valid reading)
    std::array<uint8_t, 8> data3 = {0x01, 0x02, 0x03, 0x04,
                                    0x05, 0x06, 0x07, 0x08};

    EXPECT_TRUE(checkInvalidReading(data1.data(), data1.size()));
    EXPECT_FALSE(checkInvalidReading(data2.data(), data2.size()));
    EXPECT_FALSE(checkInvalidReading(data3.data(), data3.size()));
}

// Test temperature sensors (4-byte valid range).
TEST(SmbpbiSensorTest, CheckInvalidReading4Bytes)
{
    // data1: all bytes 0xFF (invalid reading)
    std::array<uint8_t, 8> data1 = {0xFF, 0xFF, 0xFF, 0xFF,
                                    0xFF, 0xFF, 0xFF, 0xFF};

    // data2: first 4 bytes 0xFF, rest valid (invalid reading for 4-byte check)
    std::array<uint8_t, 8> data2 = {0xFF, 0xFF, 0xFF, 0xFF,
                                    0x01, 0x02, 0x04, 0x04};

    // data3: all bytes valid (valid reading)
    std::array<uint8_t, 8> data3 = {0x01, 0x02, 0x03, 0x04,
                                    0x05, 0x06, 0x07, 0x08};

    EXPECT_TRUE(checkInvalidReading(data1.data(), sizeof(uint32_t)));
    EXPECT_TRUE(checkInvalidReading(data2.data(), sizeof(uint32_t)));
    EXPECT_FALSE(checkInvalidReading(data3.data(), sizeof(uint32_t)));
}
