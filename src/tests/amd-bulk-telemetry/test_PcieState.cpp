/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NetworkAdapter.hpp"
#include "PcieState.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

namespace ocp::bulk::pcie_state
{
namespace
{

using Generations = PcieSlotInfo::Generations;

constexpr size_t payloadSize = 204;
constexpr size_t capGenOffset = 4;
constexpr size_t capWidthOffset = 8;
constexpr size_t curGenOffset = 12;
constexpr size_t curWidthOffset = 16;
constexpr size_t priBusOffset = 21;
constexpr size_t powerDownOffset = 31;
constexpr size_t macUpOffset = 32;
constexpr size_t stateOffset = 36;
constexpr size_t faultReasonOffset = 44;
constexpr size_t subVendorIdOffset = 26;
constexpr size_t subDeviceIdOffset = 28;

std::vector<uint8_t> makePortStatePayload()
{
    return std::vector<uint8_t>(payloadSize, 0);
}

void writeU16(std::vector<uint8_t>& data, size_t offset, uint16_t value)
{
    std::memcpy(data.data() + offset, &value, sizeof(value));
}

void writeI32(std::vector<uint8_t>& data, size_t offset, int32_t value)
{
    std::memcpy(data.data() + offset, &value, sizeof(value));
}

// EXPECT_TRUE is nonfatal, so return a default value if decoding failed.
PcieLink decoded(const std::vector<uint8_t>& data)
{
    const std::optional<PcieLink> link = decode(data);
    EXPECT_TRUE(link.has_value());
    return link.value_or(PcieLink{});
}

TEST(PcieState, DecodesValidPayload)
{
    std::vector<uint8_t> data = makePortStatePayload();
    writeI32(data, capGenOffset, 6);
    writeI32(data, capWidthOffset, 16);
    writeI32(data, curGenOffset, 5);
    writeI32(data, curWidthOffset, 8);
    writeI32(data, macUpOffset, 1);
    writeU16(data, subVendorIdOffset, 0x1dd8);
    writeU16(data, subDeviceIdOffset, 0x5304);
    data[priBusOffset] = 0x42;

    const PcieLink link = decoded(data);
    EXPECT_EQ(link.generationSupported, Generations::Gen6);
    EXPECT_EQ(link.generationInUse, Generations::Gen5);
    EXPECT_EQ(link.maxLanes, 16U);
    EXPECT_EQ(link.lanesInUse, 8U);
    EXPECT_TRUE(link.available);
    EXPECT_TRUE(link.functional);
    EXPECT_EQ(link.subsystemVendorId, 0x1dd8);
    EXPECT_EQ(link.subsystemId, 0x5304);
    EXPECT_EQ(link.busNumber, 0x42);
}

TEST(PcieState, MapsPortStateToLinkStateAndStatus)
{
    using LinkState = PortInfo::LinkState;
    using LinkStatus = PortInfo::LinkStatus;

    struct Case
    {
        int32_t state;
        int32_t macUp;
        LinkState linkState;
        LinkStatus linkStatus;
    };

    constexpr std::array<Case, 11> cases{{
        {0, 0, LinkState::Disabled, LinkStatus::NoLink},
        {0, 1, LinkState::Disabled, LinkStatus::NoLink},
        {1, 1, LinkState::Enabled, LinkStatus::LinkDown},
        {2, 1, LinkState::Enabled, LinkStatus::Training},
        {2, 0, LinkState::Enabled, LinkStatus::LinkDown},
        {3, 1, LinkState::Enabled, LinkStatus::Starting},
        {3, 0, LinkState::Enabled, LinkStatus::LinkDown},
        {4, 1, LinkState::Enabled, LinkStatus::LinkUp},
        {4, 0, LinkState::Enabled, LinkStatus::LinkDown},
        {5, 1, LinkState::Enabled, LinkStatus::LinkDown},
        {6, 1, LinkState::Unknown, LinkStatus::Unknown},
    }};

    for (const Case& c : cases)
    {
        SCOPED_TRACE(testing::Message()
                     << "state " << c.state << " macUp " << c.macUp);
        std::vector<uint8_t> data = makePortStatePayload();
        writeI32(data, stateOffset, c.state);
        writeI32(data, macUpOffset, c.macUp);

        const PcieLink link = decoded(data);
        EXPECT_EQ(link.linkState, c.linkState);
        EXPECT_EQ(link.linkStatus, c.linkStatus);
        EXPECT_EQ(link.available, c.macUp != 0);
    }
}

TEST(PcieState, ReportsFaultReasonAsNotFunctional)
{
    std::vector<uint8_t> data = makePortStatePayload();
    data[faultReasonOffset] = 'F';

    const PcieLink link = decoded(data);
    EXPECT_FALSE(link.functional);
}

TEST(PcieState, ReportsOutOfRangeLinkFieldsAsUnknown)
{
    std::vector<uint8_t> data = makePortStatePayload();
    writeI32(data, capGenOffset, 7);
    writeI32(data, curGenOffset, 0);
    writeI32(data, capWidthOffset, -1);
    writeI32(data, curWidthOffset, -1);

    const PcieLink link = decoded(data);
    EXPECT_EQ(link.generationSupported, Generations::Unknown);
    EXPECT_EQ(link.generationInUse, Generations::Unknown);
    EXPECT_EQ(link.maxLanes, 0U);
    EXPECT_EQ(link.lanesInUse, 0U);
}

TEST(PcieState, ReportsPoweredDownPortAsUnavailable)
{
    std::vector<uint8_t> data = makePortStatePayload();
    writeI32(data, stateOffset, 4);
    writeI32(data, macUpOffset, 1);
    data[powerDownOffset] = 1;

    const PcieLink link = decoded(data);
    EXPECT_FALSE(link.available);
    EXPECT_EQ(link.linkStatus, PortInfo::LinkStatus::NoLink);
}

TEST(PcieState, RejectsWrongSize)
{
    std::vector<uint8_t> data = makePortStatePayload();
    data.pop_back();

    EXPECT_FALSE(decode(data).has_value());
}

} // namespace
} // namespace ocp::bulk::pcie_state
