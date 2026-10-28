/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NetworkAdapter.hpp"
#include "OsfpStats.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

namespace ocp::bulk::osfp_stats
{
namespace
{
constexpr size_t structSize = 876;
constexpr size_t stringFieldLength = 16;
constexpr size_t cableLengthOffset = 4;
constexpr size_t fwActiveBuildOffset = 8;
constexpr size_t txPowerOffset = 150;
constexpr size_t rxPowerOffset = 166;
constexpr size_t txBiasOffset = 182;
constexpr size_t vccOffset = 222;
constexpr size_t vendorOffset = 556;
constexpr size_t partNumOffset = 572;
constexpr size_t serialNumOffset = 588;
constexpr size_t presentOffset = 604;
constexpr size_t readyOffset = 605;
constexpr size_t fwActiveValidOffset = 609;
constexpr size_t fwActiveMajorOffset = 610;
constexpr size_t fwActiveMinorOffset = 611;
constexpr size_t flagDisconnectedOffset = 617;
constexpr size_t transTypeOffset = 606;
constexpr size_t mediaOffset = 607;
constexpr size_t stateValidOffset = 659;
constexpr size_t laneMonValidOffset = 661;
constexpr size_t supplyValidOffset = 663;

void writeWord(std::vector<uint8_t>& data, size_t offset, uint16_t value)
{
    data[offset] = static_cast<uint8_t>(value);
    data[offset + 1] = static_cast<uint8_t>(value >> 8);
}

void writeString(std::vector<uint8_t>& data, size_t offset,
                 std::string_view text)
{
    const auto field = data.begin() + static_cast<ptrdiff_t>(offset);
    std::fill_n(field, stringFieldLength, ' ');
    std::ranges::copy(text, field);
}

std::vector<uint8_t> makeOsfpPayload()
{
    std::vector<uint8_t> data(structSize, 0);
    data[cableLengthOffset] = 3;
    data[presentOffset] = 1;
    data[readyOffset] = 1;
    data[stateValidOffset] = 1;
    data[fwActiveValidOffset] = 1;
    data[fwActiveMajorOffset] = 1;
    data[fwActiveMinorOffset] = 2;
    data[fwActiveBuildOffset] = 0x39;
    data[fwActiveBuildOffset + 1] = 0x03;
    data[transTypeOffset] = 16;
    data[mediaOffset] = 2;
    data[laneMonValidOffset] = 1;
    data[supplyValidOffset] = 1;
    writeWord(data, txPowerOffset, 10000);
    writeWord(data, rxPowerOffset, 8000);
    writeWord(data, txBiasOffset, 3500);
    writeWord(data, txPowerOffset + 14, 20000);
    writeWord(data, vccOffset, 33000);
    writeString(data, vendorOffset, "Molex");
    writeString(data, partNumOffset, "2253611258");
    writeString(data, serialNumOffset, "MX261935TR3.2SOH");
    return data;
}

std::vector<uint8_t> moduleWith(size_t offset, uint8_t value)
{
    std::vector<uint8_t> data = makeOsfpPayload();
    data[offset] = value;
    return data;
}

// EXPECT_TRUE is nonfatal, so return a default value if the optional is empty.
template <typename T>
T valueOf(const std::optional<T>& value)
{
    EXPECT_TRUE(value.has_value());
    return value.value_or(T{});
}

Transceiver decoded(std::span<const uint8_t> data)
{
    return valueOf(decode(data));
}

TEST(OsfpStats, RejectsWrongSize)
{
    EXPECT_FALSE(decode(std::vector<uint8_t>(structSize - 1, 0)).has_value());
    EXPECT_FALSE(decode(std::vector<uint8_t>(structSize + 1, 0)).has_value());
    EXPECT_FALSE(decode({}).has_value());
}

TEST(OsfpStats, DecodesValidPayload)
{
    const Transceiver transceiver = decoded(makeOsfpPayload());

    EXPECT_TRUE(transceiver.present);
    EXPECT_TRUE(transceiver.functional);
    EXPECT_EQ(transceiver.manufacturer, "Molex");
    EXPECT_EQ(transceiver.partNumber, "2253611258");
    EXPECT_EQ(transceiver.serialNumber, "MX261935TR3.2SOH");
    EXPECT_EQ(transceiver.firmwareVersion, "1.2.825");
    EXPECT_EQ(transceiver.linkStatus, PortInfo::LinkStatus::LinkUp);
    EXPECT_EQ(transceiver.cableLengthMeters, 3U);
    EXPECT_NEAR(valueOf(transceiver.supplyVoltageVolts), 3.3, 1e-12);

    const auto lanes = valueOf(transceiver.lanes);
    EXPECT_NEAR(lanes[0].txOutputPowerWatts, 1e-3, 1e-12);
    EXPECT_NEAR(lanes[0].rxInputPowerWatts, 8e-4, 1e-12);
    EXPECT_NEAR(lanes[0].txBiasCurrentAmps, 7e-3, 1e-12);
    EXPECT_NEAR(lanes[7].txOutputPowerWatts, 2e-3, 1e-12);
}

TEST(OsfpStats, DecodesTransceiverDescription)
{
    using Type = TransceiverInfo::TransceiverType;
    using Medium = TransceiverInfo::MediumType;
    using Interface = TransceiverInfo::ManagementInterface;

    struct Case
    {
        uint8_t transType;
        uint8_t media;
        Type type;
        Medium medium;
        Interface interface;
    };
    constexpr std::array<Case, 4> cases{{
        {16, 2, Type::OSFP, Medium::FiberOptic, Interface::CMIS},
        {12, 2, Type::QSFP28, Medium::FiberOptic, Interface::SFP},
        {5, 1, Type::SFP28, Medium::Copper, Interface::SFP},
        {200, 200, Type::Unknown, Medium::Unknown, Interface::Unknown},
    }};

    for (const Case& c : cases)
    {
        std::vector<uint8_t> data = makeOsfpPayload();
        data[transTypeOffset] = c.transType;
        data[mediaOffset] = c.media;

        const Transceiver transceiver = decoded(data);

        EXPECT_EQ(transceiver.type, c.type) << "type " << int{c.transType};
        EXPECT_EQ(transceiver.mediumType, c.medium) << "media " << int{c.media};
        EXPECT_EQ(transceiver.managementInterface, c.interface)
            << "type " << int{c.transType};
    }
}

TEST(OsfpStats, LeavesFieldsEmptyWhenNotValid)
{
    EXPECT_FALSE(decoded(moduleWith(laneMonValidOffset, 0)).lanes.has_value());
    EXPECT_FALSE(decoded(moduleWith(supplyValidOffset, 0))
                     .supplyVoltageVolts.has_value());
    EXPECT_TRUE(
        decoded(moduleWith(fwActiveValidOffset, 0)).firmwareVersion.empty());
}

TEST(OsfpStats, DecodesLinkStatusAndHealth)
{
    std::vector<uint8_t> data = moduleWith(readyOffset, 0);
    const Transceiver notReady = decoded(data);
    EXPECT_EQ(notReady.linkStatus, PortInfo::LinkStatus::Starting);
    EXPECT_FALSE(notReady.functional);

    data[stateValidOffset] = 0;
    const Transceiver invalidState = decoded(data);
    EXPECT_EQ(invalidState.linkStatus, PortInfo::LinkStatus::Unknown);
    EXPECT_TRUE(invalidState.functional);

    EXPECT_EQ(decoded(moduleWith(stateValidOffset, 0)).linkStatus,
              PortInfo::LinkStatus::Unknown);
    EXPECT_EQ(decoded(moduleWith(presentOffset, 0)).linkStatus,
              PortInfo::LinkStatus::NoLink);
    EXPECT_EQ(decoded(moduleWith(flagDisconnectedOffset, 1)).linkStatus,
              PortInfo::LinkStatus::NoLink);
}

// An empty cage can still report itself present, so the disconnected flag
// decides as well.
TEST(OsfpStats, ReportsAnEmptyCageAsAbsent)
{
    const Transceiver absent = decoded(moduleWith(presentOffset, 0));
    EXPECT_FALSE(absent.present);
    EXPECT_EQ(absent.cableLengthMeters, 0U);

    const Transceiver transceiver =
        decoded(moduleWith(flagDisconnectedOffset, 1));

    EXPECT_FALSE(transceiver.present);
    EXPECT_EQ(transceiver.cableLengthMeters, 0U);
    EXPECT_TRUE(transceiver.functional);
    EXPECT_TRUE(transceiver.manufacturer.empty());
    EXPECT_TRUE(transceiver.partNumber.empty());
    EXPECT_TRUE(transceiver.serialNumber.empty());
    EXPECT_TRUE(transceiver.firmwareVersion.empty());
}

TEST(OsfpStats, TrimsStringFields)
{
    std::vector<uint8_t> data = makeOsfpPayload();

    writeString(data, vendorOffset, "Molex   ");
    EXPECT_EQ(decoded(data).manufacturer, "Molex");

    data[vendorOffset + 3] = '\0';
    EXPECT_EQ(decoded(data).manufacturer, "Mol");

    writeString(data, vendorOffset, "");
    EXPECT_TRUE(decoded(data).manufacturer.empty());
}

} // namespace
} // namespace ocp::bulk::osfp_stats
