#include "ipmb/IpmbSDRSensor.hpp"

#include <sdbusplus/asio/connection.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

namespace
{

// IpmbSDRDevice::checkSDRData() previously trusted the device-supplied SDR
// bytes when computing the sensor-name range: it waited only for
// size >= sdrData[6] + 7, then read fixed Type 1 offsets (up to byte
// nameLengthByte=53) and formed std::string(begin()+strAddr,
// begin()+strAddr+strLen) with no bounds checks. These tests pin the
// guards by asserting their observable effect: a rejected record must
// leave sensorRecord untouched, an accepted record must publish its
// name.

class IpmbSDRSensorBoundsTest : public ::testing::Test
{
  public:
    ~IpmbSDRSensorBoundsTest() noexcept override = default;

  protected:
    std::shared_ptr<sdbusplus::asio::connection> conn;
    IpmbSDRDevice device{conn, 0x20};

    // device{conn, 0x20} -> hostIndex = 0x21, so records land in
    // sensorRecord[0x20].
    static constexpr int busIndex = 0x20;

    void SetUp() override
    {
        // sensorRecord is a process-global inline map; keep tests
        // order-independent.
        sensorRecord.clear();
        sensorValRecord.clear();
    }

    // Build a zeroed Type 1 response of the given size with the given
    // declared data length at byte 6.
    static std::vector<uint8_t> type1Record(uint8_t dataLen, size_t size)
    {
        std::vector<uint8_t> rec(size, 0);
        rec[sdr::sdrType] = 1; // SDR Type 1
        rec[sdr::dataLengthByte] = dataLen;
        return rec;
    }

    static size_t published()
    {
        auto it = sensorRecord.find(busIndex);
        if (it == sensorRecord.end())
        {
            return 0;
        }
        return it->second.size();
    }
};

TEST_F(IpmbSDRSensorBoundsTest, NameRangeBeyondBufferRejected)
{
    // Declared record length (247 bytes) makes the size guard pass, but
    // the name range computed from dataLen=240 lands 53 bytes past the
    // buffer (strAddr = 300, strLen = 1). The name-range guard must
    // reject before std::string construction reads out of bounds.
    constexpr uint8_t dataLen = 240;
    const size_t size = static_cast<size_t>(dataLen) + sdr::dataLengthByte + 1;
    auto rec = type1Record(dataLen, size);
    rec[sdrtype01::nameLengthByte] = 1; // strLen = 1 -> strAddr = 300

    device.checkSDRData(rec, static_cast<uint8_t>(size));
    EXPECT_EQ(published(), 0U); // rejection must be observable
}

TEST_F(IpmbSDRSensorBoundsTest, NegativeStrAddrRejected)
{
    // dataLen=0, strLen=31 computes strAddr = 0 + 0 - 30 = -30. The sign
    // guard is the only thing standing between this and
    // begin() + (-30) UB before any dereference.
    constexpr uint8_t dataLen = 0;
    const size_t size = sdrtype01::nameLengthByte + 1;
    auto rec = type1Record(dataLen, size);
    rec[sdrtype01::nameLengthByte] = 31; // strLen = 31 -> strAddr = -30

    device.checkSDRData(rec, static_cast<uint8_t>(size));
    EXPECT_EQ(published(), 0U);
}

TEST_F(IpmbSDRSensorBoundsTest, ShortRecordRejectedBeforeFixedFields)
{
    // size >= dataLength holds (17 >= 10 + 7), but the fixed Type 1 bytes
    // (up to nameLengthByte=53) do not exist; the central length guard
    // must reject before any fixed offset is consumed.
    constexpr uint8_t dataLen = 10;
    const size_t size = static_cast<size_t>(dataLen) + sdr::dataLengthByte + 1;
    auto rec = type1Record(dataLen, size);

    device.checkSDRData(rec, static_cast<uint8_t>(size));
    EXPECT_EQ(published(), 0U);
}

TEST_F(IpmbSDRSensorBoundsTest, StrLenOverrunRejected)
{
    // strAddr itself is in bounds but strLen runs past the end: size=64,
    // dataLen=40 -> strAddr = 40 + 8 - (31 - 1) = 18, strLen=31 ->
    // 18 + 31 = 49 <= 64... use dataLen=56: strAddr = 56 + 12 - 30 = 38,
    // 38 + 31 = 69 > 64. The third clause must reject.
    constexpr uint8_t dataLen = 56;
    const size_t size = 64;
    auto rec = type1Record(dataLen, size);
    rec[sdrtype01::nameLengthByte] = 31;

    device.checkSDRData(rec, static_cast<uint8_t>(size));
    EXPECT_EQ(published(), 0U);
}

TEST_F(IpmbSDRSensorBoundsTest, PlausibleType1NameAccepted)
{
    // End-to-end sanity for a properly sequenced record: dataLen=51 puts
    // the 4-byte name "TEMP" at strAddr=60 inside a 64-byte response.
    constexpr uint8_t dataLen = 51;
    const size_t size = 64;
    auto rec = type1Record(dataLen, size);
    rec[sdrtype01::nameLengthByte] = 4;
    const std::array<uint8_t, 4> name = {'T', 'E', 'M', 'P'};
    std::copy(name.begin(), name.end(), rec.begin() + 60);

    device.checkSDRData(rec, static_cast<uint8_t>(size));
    ASSERT_EQ(published(), 1U);
    EXPECT_EQ(sensorRecord[busIndex][0].sensorReadName, "TEMP");
}

TEST_F(IpmbSDRSensorBoundsTest, EmptyNameAtEndAccepted)
{
    // strAddr == size with strLen == 0 is the legal empty iterator range
    // [end, end); a future tightening to >= must not reject it.
    // dataLen=56, strLen=0 -> strAddr = 56 + 12 + 1 = 69 > 64. Pick
    // dataLen=51, strLen=0 -> strAddr = 51 + 12 + 1 = 64 == size.
    constexpr uint8_t dataLen = 51;
    const size_t size = 64;
    auto rec = type1Record(dataLen, size);
    rec[sdrtype01::nameLengthByte] = 0; // strLen = 0 -> strAddr = 64

    device.checkSDRData(rec, static_cast<uint8_t>(size));
    // Accepted (legal empty name); the record must publish.
    EXPECT_EQ(published(), 1U);
}

} // namespace
