/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "LldpFrame.hpp"

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <span>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace
{

using lldp::Frame;
using lldp::IdSubtype;
using lldp::SystemCapability;

void appendField(std::vector<uint8_t>& pdu, uint8_t type,
                 std::initializer_list<uint8_t> value)
{
    const size_t length = value.size();
    pdu.push_back(static_cast<uint8_t>((type << 1) | ((length >> 8) & 0x01)));
    pdu.push_back(static_cast<uint8_t>(length & 0xFF));
    pdu.insert(pdu.end(), value.begin(), value.end());
}

void appendEnd(std::vector<uint8_t>& pdu)
{
    appendField(pdu, 0, {});
}

// Chassis identifier 00:11:22:33:44:55, port identifier "eth0", 120 second time
// to live: the three fields IEEE 802.1AB requires, in the order it requires.
std::vector<uint8_t> mandatoryFields()
{
    std::vector<uint8_t> pdu;
    appendField(pdu, 1, {4, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55});
    appendField(pdu, 2, {5, 'e', 't', 'h', '0'});
    appendField(pdu, 3, {0x00, 0x78});
    return pdu;
}

std::vector<uint8_t> minimalUnit()
{
    std::vector<uint8_t> pdu = mandatoryFields();
    appendEnd(pdu);
    return pdu;
}

TEST(LldpFrame, DecodesTheMandatoryFields)
{
    Frame frame;
    ASSERT_TRUE(lldp::parse(minimalUnit(), frame));

    EXPECT_EQ(frame.chassisIdSubtype, IdSubtype::MacAddr);
    EXPECT_EQ(frame.chassisId, "00:11:22:33:44:55");
    EXPECT_EQ(frame.portIdSubtype, IdSubtype::IfName);
    EXPECT_EQ(frame.portId, "eth0");
}

TEST(LldpFrame, LeavesAbsentFieldsAtTheirDefaults)
{
    Frame frame;
    ASSERT_TRUE(lldp::parse(minimalUnit(), frame));

    EXPECT_TRUE(frame.systemName.empty());
    EXPECT_TRUE(frame.systemDescription.empty());
    EXPECT_TRUE(frame.systemCapabilities.empty());
    EXPECT_TRUE(frame.managementAddressIPv4.empty());
    EXPECT_TRUE(frame.managementAddressIPv6.empty());
    EXPECT_TRUE(frame.managementAddressMAC.empty());
}

TEST(LldpFrame, ClearsWhatAnEarlierUnitLeftBehind)
{
    Frame frame;
    std::vector<uint8_t> named = mandatoryFields();
    appendField(named, 5, {'s', 'w', '1'});
    appendEnd(named);
    ASSERT_TRUE(lldp::parse(named, frame));
    ASSERT_EQ(frame.systemName, "sw1");

    ASSERT_TRUE(lldp::parse(minimalUnit(), frame));
    EXPECT_TRUE(frame.systemName.empty());
}

// The transport zero-pads its payload out to a four byte boundary, so a unit
// whose length is not a multiple of four arrives with zeros after its end.
TEST(LldpFrame, AcceptsZeroPaddingAfterTheEndMarker)
{
    std::vector<uint8_t> pdu = minimalUnit();
    ASSERT_EQ(pdu.size() % 4, 2U);
    pdu.push_back(0);
    pdu.push_back(0);

    Frame frame;
    ASSERT_TRUE(lldp::parse(pdu, frame));
    EXPECT_EQ(frame.portId, "eth0");
}

TEST(LldpFrame, RejectsNonZeroBytesAfterTheEndMarker)
{
    std::vector<uint8_t> pdu = minimalUnit();
    pdu.push_back(0);
    pdu.push_back(0x42);

    Frame frame;
    EXPECT_FALSE(lldp::parse(pdu, frame));
}

TEST(LldpFrame, RejectsAnEmptyUnit)
{
    Frame frame;
    EXPECT_FALSE(lldp::parse({}, frame));
}

TEST(LldpFrame, RejectsAUnitWithNoEndMarker)
{
    Frame frame;
    EXPECT_FALSE(lldp::parse(mandatoryFields(), frame));
}

TEST(LldpFrame, RejectsAFieldRunningPastTheEndOfTheUnit)
{
    std::vector<uint8_t> pdu = mandatoryFields();
    pdu.pop_back();

    Frame frame;
    EXPECT_FALSE(lldp::parse(pdu, frame));
}

TEST(LldpFrame, RejectsAnEndMarkerCarryingAValue)
{
    std::vector<uint8_t> pdu = mandatoryFields();
    appendField(pdu, 0, {0x00});

    Frame frame;
    EXPECT_FALSE(lldp::parse(pdu, frame));
}

TEST(LldpFrame, RejectsMandatoryFieldsOutOfOrder)
{
    std::vector<uint8_t> pdu;
    appendField(pdu, 2, {5, 'e', 't', 'h', '0'});
    appendField(pdu, 1, {4, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55});
    appendField(pdu, 3, {0x00, 0x78});
    appendEnd(pdu);

    Frame frame;
    EXPECT_FALSE(lldp::parse(pdu, frame));
}

TEST(LldpFrame, RejectsAnOptionalFieldBeforeTheMandatoryOnes)
{
    std::vector<uint8_t> pdu;
    appendField(pdu, 5, {'s', 'w', '1'});
    appendField(pdu, 1, {4, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55});
    appendField(pdu, 2, {5, 'e', 't', 'h', '0'});
    appendField(pdu, 3, {0x00, 0x78});
    appendEnd(pdu);

    Frame frame;
    EXPECT_FALSE(lldp::parse(pdu, frame));
}

TEST(LldpFrame, RejectsATimeToLiveOfTheWrongLength)
{
    std::vector<uint8_t> pdu;
    appendField(pdu, 1, {4, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55});
    appendField(pdu, 2, {5, 'e', 't', 'h', '0'});
    appendField(pdu, 3, {0x78});
    appendEnd(pdu);

    Frame frame;
    EXPECT_FALSE(lldp::parse(pdu, frame));
}

// Tables 8-2 and 8-3 number the subtypes independently, so the same byte means
// one thing in a chassis identifier and another in a port identifier.
TEST(LldpFrame, ReadsChassisAndPortSubtypesFromSeparateTables)
{
    std::vector<uint8_t> pdu;
    appendField(pdu, 1, {3, 'a'});
    appendField(pdu, 2, {3, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55});
    appendField(pdu, 3, {0x00, 0x78});
    appendEnd(pdu);

    Frame frame;
    ASSERT_TRUE(lldp::parse(pdu, frame));
    EXPECT_EQ(frame.chassisIdSubtype, IdSubtype::PortComp);
    EXPECT_EQ(frame.portIdSubtype, IdSubtype::MacAddr);
    EXPECT_EQ(frame.portId, "00:11:22:33:44:55");
}

// NotTransmitted is all this interface can say about a subtype neither table
// names, and it says the identifier is not there, so the identifier is not
// reported either.
TEST(LldpFrame, LeavesOutAnIdentifierOfAnUnnamedSubtype)
{
    std::vector<uint8_t> pdu;
    appendField(pdu, 1, {9, 0xDE, 0xAD});
    appendField(pdu, 2, {5, 'e', 't', 'h', '0'});
    appendField(pdu, 3, {0x00, 0x78});
    appendEnd(pdu);

    Frame frame;
    ASSERT_TRUE(lldp::parse(pdu, frame));
    EXPECT_EQ(frame.chassisIdSubtype, IdSubtype::NotTransmitted);
    EXPECT_TRUE(frame.chassisId.empty());
}

TEST(LldpFrame, DecodesTheSystemNameAndDescription)
{
    std::vector<uint8_t> pdu = mandatoryFields();
    appendField(pdu, 5, {'s', 'w', '1'});
    appendField(pdu, 6, {'a', ' ', 's', 'w', 'i', 't', 'c', 'h'});
    appendEnd(pdu);

    Frame frame;
    ASSERT_TRUE(lldp::parse(pdu, frame));
    EXPECT_EQ(frame.systemName, "sw1");
    EXPECT_EQ(frame.systemDescription, "a switch");
}

TEST(LldpFrame, DecodesTheCapabilitiesTheDeviceClaims)
{
    std::vector<uint8_t> pdu = mandatoryFields();
    // Bridge and router, of which only bridge is enabled; the enabled half of
    // the field does not describe the device itself.
    appendField(pdu, 7, {0x00, 0x14, 0x00, 0x04});
    appendEnd(pdu);

    Frame frame;
    ASSERT_TRUE(lldp::parse(pdu, frame));
    EXPECT_EQ(frame.systemCapabilities,
              (std::vector<SystemCapability>{SystemCapability::Bridge,
                                             SystemCapability::Router}));
}

TEST(LldpFrame, TellsClaimingNothingApartFromSayingNothing)
{
    std::vector<uint8_t> pdu = mandatoryFields();
    appendField(pdu, 7, {0x00, 0x00, 0x00, 0x00});
    appendEnd(pdu);

    Frame frame;
    ASSERT_TRUE(lldp::parse(pdu, frame));
    EXPECT_EQ(frame.systemCapabilities,
              (std::vector<SystemCapability>{SystemCapability::None}));

    ASSERT_TRUE(lldp::parse(minimalUnit(), frame));
    EXPECT_TRUE(frame.systemCapabilities.empty());
}

TEST(LldpFrame, PassesOverACapabilitiesFieldOfTheWrongLength)
{
    std::vector<uint8_t> pdu = mandatoryFields();
    appendField(pdu, 7, {0x00, 0x04});
    appendEnd(pdu);

    Frame frame;
    ASSERT_TRUE(lldp::parse(pdu, frame));
    EXPECT_TRUE(frame.systemCapabilities.empty());
    EXPECT_EQ(frame.portId, "eth0");
}

// Table 8-4 names functions above the eight this interface carries. A device
// claiming only those has said something, so it must not read as None, which
// is what a device claiming nothing at all says.
TEST(LldpFrame, ReportsNothingForCapabilitiesItCannotName)
{
    std::vector<uint8_t> pdu = mandatoryFields();
    appendField(pdu, 7, {0x01, 0x00, 0x01, 0x00});
    appendEnd(pdu);

    Frame frame;
    ASSERT_TRUE(lldp::parse(pdu, frame));
    EXPECT_TRUE(frame.systemCapabilities.empty());
}

TEST(LldpFrame, DecodesManagementAddressesOfEachFamily)
{
    std::vector<uint8_t> pdu = mandatoryFields();
    appendField(pdu, 8, {5, 1, 10, 0, 0, 1, 2, 0, 0, 0, 1, 0});
    appendField(pdu, 8, {17, 2, 0x20, 0x01, 0x0d, 0xb8, 0, 0, 0, 0, 0, 0,
                         0,  0, 0,    0,    0,    1,    2, 0, 0, 0, 1, 0});
    appendField(pdu, 8,
                {7, 6, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 2, 0, 0, 0, 1, 0});
    appendEnd(pdu);

    Frame frame;
    ASSERT_TRUE(lldp::parse(pdu, frame));
    EXPECT_EQ(frame.managementAddressIPv4, "10.0.0.1");
    EXPECT_EQ(frame.managementAddressIPv6, "2001:db8::1");
    EXPECT_EQ(frame.managementAddressMAC, "AA:BB:CC:DD:EE:FF");
}

TEST(LldpFrame, KeepsOnlyTheFirstManagementAddressOfAFamily)
{
    std::vector<uint8_t> pdu = mandatoryFields();
    appendField(pdu, 8, {5, 1, 10, 0, 0, 1, 2, 0, 0, 0, 1, 0});
    appendField(pdu, 8, {5, 1, 10, 0, 0, 2, 2, 0, 0, 0, 1, 0});
    appendEnd(pdu);

    Frame frame;
    ASSERT_TRUE(lldp::parse(pdu, frame));
    EXPECT_EQ(frame.managementAddressIPv4, "10.0.0.1");
}

// A sender that gets one optional field wrong must not cost a consumer the
// fields it got right.
TEST(LldpFrame, KeepsWhatDecodedWhenAnOptionalFieldIsMalformed)
{
    std::vector<uint8_t> pdu = mandatoryFields();
    // Claims a five byte address string but carries one byte fewer than the
    // trailer needs.
    appendField(pdu, 8, {5, 1, 10, 0, 0, 1, 2, 0, 0, 0, 1});
    appendField(pdu, 5, {'s', 'w', '1'});
    appendEnd(pdu);

    Frame frame;
    ASSERT_TRUE(lldp::parse(pdu, frame));
    EXPECT_TRUE(frame.managementAddressIPv4.empty());
    EXPECT_EQ(frame.systemName, "sw1");
    EXPECT_EQ(frame.chassisId, "00:11:22:33:44:55");
}

TEST(LldpFrame, PassesOverAManagementAddressOfAnUnknownFamily)
{
    std::vector<uint8_t> pdu = mandatoryFields();
    appendField(pdu, 8, {3, 99, 0xAA, 0xBB, 2, 0, 0, 0, 1, 0});
    appendEnd(pdu);

    Frame frame;
    ASSERT_TRUE(lldp::parse(pdu, frame));
    EXPECT_TRUE(frame.managementAddressIPv4.empty());
    EXPECT_TRUE(frame.managementAddressIPv6.empty());
    EXPECT_TRUE(frame.managementAddressMAC.empty());
}

TEST(LldpFrame, PassesOverAFieldThisInterfaceDoesNotCarry)
{
    std::vector<uint8_t> pdu = mandatoryFields();
    appendField(pdu, 4, {'u', 'p', 'l', 'i', 'n', 'k'});
    appendField(pdu, 5, {'s', 'w', '1'});
    appendEnd(pdu);

    Frame frame;
    ASSERT_TRUE(lldp::parse(pdu, frame));
    EXPECT_EQ(frame.systemName, "sw1");
}

// A short unit travels inside a frame padded out to the link's minimum
// length, so the zeros after it run to tens of bytes rather than three.
TEST(LldpFrame, AcceptsTheZerosAShortFramePadsWith)
{
    std::vector<uint8_t> pdu = minimalUnit();
    pdu.resize(60 - 14, 0);

    Frame frame;
    ASSERT_TRUE(lldp::parse(pdu, frame));
    EXPECT_EQ(frame.portId, "eth0");
}

TEST(LldpFrame, StopsTextAtTheFirstNull)
{
    std::vector<uint8_t> pdu = mandatoryFields();
    appendField(pdu, 5, {'s', 'w', '1', 0, 0});
    appendEnd(pdu);

    Frame frame;
    ASSERT_TRUE(lldp::parse(pdu, frame));
    EXPECT_EQ(frame.systemName, "sw1");
}

// D-Bus carries only well-formed UTF-8, so text that is not is dropped here
// rather than left to fail where it is published.
TEST(LldpFrame, DropsTextThatIsNotUtf8)
{
    std::vector<uint8_t> pdu = mandatoryFields();
    appendField(pdu, 5, {'s', 'w', 0xFF});
    appendField(pdu, 6, {'a', ' ', 's', 'w', 'i', 't', 'c', 'h'});
    appendEnd(pdu);

    Frame frame;
    ASSERT_TRUE(lldp::parse(pdu, frame));
    EXPECT_TRUE(frame.systemName.empty());
    EXPECT_EQ(frame.systemDescription, "a switch");
}

TEST(LldpFrame, KeepsTextOutsideTheAsciiRange)
{
    std::vector<uint8_t> pdu = mandatoryFields();
    // "swä" encoded as UTF-8.
    appendField(pdu, 5, {'s', 'w', 0xC3, 0xA4});
    appendEnd(pdu);

    Frame frame;
    ASSERT_TRUE(lldp::parse(pdu, frame));
    EXPECT_EQ(frame.systemName, "sw\xc3\xa4");
}

} // namespace
