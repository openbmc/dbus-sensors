/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <vector>

// The chassis and port identifier subtypes of IEEE 802.1AB-2009 tables 8-2 and
// 8-3. The two tables give the same numeric value different meanings, so a
// subtype is only meaningful next to the identifier it was decoded from.
enum class LldpIdSubtype : uint8_t
{
    NotTransmitted,
    AgentId,
    ChassisComp,
    IfAlias,
    IfName,
    LocalAssign,
    MacAddr,
    NetworkAddr,
    PortComp,
};

// The primary functions a device advertises, per IEEE 802.1AB-2009 table 8-4.
enum class LldpSystemCapability : uint8_t
{
    None,
    Bridge,
    DOCSISCableDevice,
    Other,
    Repeater,
    Router,
    Station,
    Telephone,
    WLANAccessPoint,
};

// The part of a Link Layer Discovery Protocol Data Unit that
// xyz.openbmc_project.Network.LLDP.TLVs exposes. A field the unit did not carry
// keeps its default.
struct LldpFrame
{
    std::string chassisId;
    LldpIdSubtype chassisIdSubtype{LldpIdSubtype::NotTransmitted};
    std::string managementAddressIPv4;
    std::string managementAddressIPv6;
    std::string managementAddressMAC;
    std::string portId;
    LldpIdSubtype portIdSubtype{LldpIdSubtype::NotTransmitted};
    std::vector<LldpSystemCapability> systemCapabilities;
    std::string systemDescription;
    std::string systemName;
};

// Decodes one protocol data unit into frame, replacing everything it held.
//
// Returns EINVAL, leaving frame in an unspecified state, when the unit as a
// whole is not what a conforming sender would have written: a truncated
// type-length-value field, a mandatory field missing or out of order, no end
// marker, or trailing bytes that are not the zeros padding is made of.
//
// An optional field the sender got wrong costs only itself: it is left out and
// the rest of the unit is still decoded, because a neighbor missing one value
// is of far more use to a consumer than no neighbor at all.
//
// pdu is the protocol data unit alone. A transport that frames the unit in an
// Ethernet header strips that header before calling this.
int parseLldpFrame(std::span<const uint8_t> pdu, LldpFrame& frame);
